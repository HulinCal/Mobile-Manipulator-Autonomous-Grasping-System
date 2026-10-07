#!/usr/bin/env python3
"""Keyword/command matching node for ASR results.

- Subscribes to ``/asr/final_text`` (std_msgs/msg/String).
- Compares the recognized text with a configurable list of command groups
  (canonical command + synonyms), using:
    * character-level similarity (difflib.SequenceMatcher), and
    * pinyin-level similarity (handles mispronounced / confusable
      characters and tone differences; pypinyin if available).
- The command whose best similarity is at least ``threshold`` and is the
  highest is published, using its canonical command text, to
  ``/asr_command`` (std_msgs/msg/String).

Commands are loaded from a YAML file (default: config/keywords.yaml in
this package) so new commands and synonyms (e.g. future robotic-arm
commands) can be added without changing code.
"""

import re
from difflib import SequenceMatcher
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

try:
    from pypinyin import lazy_pinyin
    _HAS_PYPINYIN = True
except ImportError:  # pinyin similarity is optional
    _HAS_PYPINYIN = False


# Common Chinese and English punctuation / whitespace
_PUNCT_RE = re.compile(
    r'[\s，。！？、；：“”‘’（）《》【】,.!?;:\'"()\[\]{}<>~～·…\-_=+*/\\|@#$%^&`]')


def normalize(text: str) -> str:
    """Remove punctuation and whitespace for comparison."""
    return _PUNCT_RE.sub('', text or '')


def to_pinyin(text: str):
    """Convert Chinese text to a list of tone-insensitive pinyin syllables.

    Matching at the syllable level (rather than character level on a joined
    string) means that confusable characters with the same pronunciation
    (e.g. 前/钱, 向/象) match, but unrelated syllables that merely
    share letters (e.g. tian vs ting) do not.
    """
    if not _HAS_PYPINYIN:
        return []
    return lazy_pinyin(text, errors='ignore') or []


def similarity(a: str, b: str) -> float:
    """SequenceMatcher ratio in [0, 1]."""
    if not a or not b:
        return 0.0
    return SequenceMatcher(a=a, b=b).ratio()


def pinyin_similarity(a, b) -> float:
    """SequenceMatcher ratio on two pinyin-syllable lists."""
    if not a or not b:
        return 0.0
    return SequenceMatcher(a=list(a), b=list(b)).ratio()


def best_match_score(text: str, phrase: str) -> float:
    """Best similarity of *phrase* against *text*.

    The text may contain filler words around the command, so we compare
    against the whole text and sliding windows whose length is close to the
    phrase length. Exact containment scores 1.0.
    """
    if not text or not phrase:
        return 0.0

    # Exact occurrence (e.g. "请小车前进" contains "小车前进")
    if phrase in text:
        return 1.0

    candidates = [text]
    n = len(text)
    # Windows of the same length as the phrase or one character longer.
    # Shorter texts are covered by the whole-text comparison. Restricting
    # the lengths prevents short windows from inflating the ratio
    # (e.g. a single matching syllable scoring 0.67 against a
    # two-syllable phrase).
    for length in (len(phrase), len(phrase) + 1):
        if length <= 0 or length > n:
            continue
        for start in range(0, n - length + 1):
            candidates.append(text[start:start + length])

    char_score = max(similarity(c, phrase) for c in candidates)

    if not _HAS_PYPINYIN:
        return char_score

    phrase_py = to_pinyin(phrase)
    py_score = 0.0
    for c in candidates:
        py_score = max(py_score,
                        pinyin_similarity(to_pinyin(c), phrase_py))

    # Take the better of character / pinyin similarity so that
    # confusable pronunciations can still match.
    return max(char_score, py_score)


def load_commands(path: Path):
    """Load and validate the keyword configuration.

    Returns a list of (canonical_command, [normalized phrases]) groups.
    """
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not data or 'commands' not in data:
        raise RuntimeError(f'Invalid keywords config: missing "commands" in {path}')

    groups = []
    for item in data['commands']:
        if not item or not item.get('command'):
            continue
        canonical = normalize(str(item['command']))
        phrases = {canonical}
        for syn in item.get('synonyms', []) or []:
            syn_n = normalize(str(syn))
            if syn_n:
                phrases.add(syn_n)
        groups.append((canonical, sorted(phrases, key=len, reverse=True)))
    if not groups:
        raise RuntimeError(f'No valid command groups found in {path}')
    return groups


def default_keywords_file() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('asr_keywords')) / \
        'config' / 'keywords.yaml'


class KeywordMatcherNode(Node):

    def __init__(self):
        super().__init__('asr_keywords')

        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('keywords_file', '')

        keywords_param = str(self.get_parameter('keywords_file').value)
        keywords_path = Path(keywords_param) if keywords_param \
            else default_keywords_file()
        if not keywords_path.is_file():
            raise RuntimeError(f'Keywords file not found: {keywords_path}')

        self.threshold = float(self.get_parameter('threshold').value)
        if not 0.0 <= self.threshold <= 1.0:
            raise RuntimeError('threshold must be between 0.0 and 1.0')

        self.command_groups = load_commands(keywords_path)

        self.subscription = self.create_subscription(
            String, '/asr/final_text', self._on_text, 10)
        self.publisher = self.create_publisher(
            String, '/asr_command', 10)

        commands_desc = ', '.join(c for c, _ in self.command_groups)
        self.get_logger().info(
            f'Loaded {len(self.command_groups)} command groups: {commands_desc}')
        self.get_logger().info(
            f'Similarity threshold: {self.threshold:.2f} | '
            f'pinyin matching: {"on" if _HAS_PYPINYIN else "off"}')
        self.get_logger().info('Waiting for /asr/final_text ...')

    def _on_text(self, msg: String):
        text = normalize(msg.data)
        if not text:
            return

        # 选择规则：先比相似度；同分时，命中短语越长（说法越具体）越优先，
        # 例如“机械臂停止/手停止”与裸词“停止”同为满分时，应归到机械臂停止
        # 而不是小车停止；分数与短语长度都相同则保持配置中的组顺序。
        best_command = None
        best_score = 0.0
        best_phrase_len = -1
        for canonical, phrases in self.command_groups:
            score = 0.0
            phrase_len = 0
            for p in phrases:
                s = best_match_score(text, p)
                if (s > score + 1e-9
                        or (abs(s - score) <= 1e-9 and len(p) > phrase_len)):
                    score = s
                    phrase_len = len(p)
            if (score > best_score + 1e-9
                    or (abs(score - best_score) <= 1e-9
                        and phrase_len > best_phrase_len)):
                best_score = score
                best_phrase_len = phrase_len
                best_command = canonical

        if best_command is not None and best_score >= self.threshold:
            self.get_logger().info(
                f'"{msg.data}" -> {best_command} '
                f'(score={best_score:.2f})')
            out = String()
            out.data = best_command
            self.publisher.publish(out)
        else:
            self.get_logger().info(
                f'"{msg.data}" matched no command '
                f'(best score={best_score:.2f} < {self.threshold:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = KeywordMatcherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
