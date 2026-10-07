# asr_keywords

ASR 语音识别结果的**关键字 / 命令匹配** ROS 2 功能包。

订阅 `/asr/final_text` 取得语音识别出的原始文字，将其与可配置的命令关键字（标准命令 + 同义词）做相似度比较，命中后把**统一的标准命令文字**发布到 `/asr_command`。

针对实际语音识别中常见的**模糊音、相似音、某个字读不准、声调差异**等情况，节点同时使用：

- **字符相似度**（`difflib`）；
- **拼音音节相似度**（`pypinyin`）——同音不同字也能正确匹配。

关键字通过 YAML 文件以参数形式导入，以后新增命令（包括未来的机械臂命令）只需改配置、无需改代码。

---

## 1. 工作逻辑

```
/asr/final_text (std_msgs/String)
        │  识别出的原始文字
        ▼
文字归一化（去标点、空格）
        ▼
对每个命令组（标准命令 + 同义词列表）计算相似度
   ├── 精确包含           -> 1.0
   ├── 字符相似度(SequenceMatcher)
   └── 拼音音节相似度(pypinyin，无声调)
        ▼
取全部分组中最高分；最高分 ≥ threshold
        │ 是                          │ 否
        ▼                             ▼
发布标准命令到 /asr_command      仅记录日志，不发布
```

具体步骤：

1. **归一化**：去除中文/英文标点和空白，避免标点影响比较。
2. **精确包含**：文字中若直接包含某个词条（如"请帮我小车停下来"含"小车停下来"），该词条得 1.0 分。
3. **滑窗匹配**：识别文字可能在命令前后带有语气词、插入字，因此除整句外，还会取与词条等长或长 1 个字的滑动窗口分别比较。
4. **字符相似度**：用 `SequenceMatcher` 比较窗口与词条。
5. **拼音相似度**：把窗口与词条转为无声调拼音**音节序列**后比较。同音不同字（如 前/钱、向/象、潜/钳）得到高分；而只是字母相近但发音不同的音节（如 tian 与 ting）不会误匹配。
6. **取最高分判定**：每个命令组取组内最高分，再在所有组中取最高；若最高分达到阈值 `threshold`（默认 0.6），则发布该组的**标准命令**（同义词统一归一），否则不发布。

> 匹配取字符分与拼音分中的较大值，因此既支持错别字/漏字（字符相似度），也支持读错音（拼音相似度）。

---

## 2. 话题

| 方向 | 话题名 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/asr/final_text` | `std_msgs/msg/String` | ASR 节点输出的原始识别文字 |
| 发布 | `/asr_command` | `std_msgs/msg/String` | 匹配成功后的标准命令文字 |

---

## 3. 关键字配置

默认配置文件：[config/keywords.yaml](config/keywords.yaml)。

每个命令组包含两个字段：

- `command`：识别成功后**统一输出**的标准命令；
- `synonyms`：同义词 / 近义说法列表（识别到这些说法时映射到 `command`）。

默认命令组（已带"小车"前缀，为以后机械臂命令预留区分）：

| 标准命令 | 内置相似说法（部分） |
|---|---|
| 小车向前 | 小车前进、小车往前、向前、前进、往前走、往前行、直行… |
| 小车向后 | 小车后退、小车倒退、小车倒车、向后、后退、往后退、倒退、倒车… |
| 小车左转 | 小车左拐、向左转、往左拐、往左、左转、左拐 |
| 小车右转 | 小车右拐、向右转、往右拐、往右、右转、右拐 |
| 小车停止 | 小车停下、小车停车、小车刹车、停止、停下、停车、刹车、停… |

### 扩展命令

直接在 `config/keywords.yaml` 中新增一组即可（文件内已附机械臂示例），例如：

```yaml
commands:
  - command: 机械臂归位
    synonyms: [机械臂回零, 回到初始位置, 回原点]

  - command: 机械臂抓取
    synonyms: [机械臂抓住, 夹起来, 拿起来, 抓起]

  - command: 机械臂释放
    synonyms: [机械臂松开, 放下, 放开, 松手]
```

也可以使用**外部配置文件**，通过 `keywords_file` 参数指定路径（见第 6 节），这样升级本包时自定义配置不会被覆盖。

> 提示：单字同义词（如"停"）会使任何含该字的句子命中（例如"停在楼下"也会触发小车停止）。若希望更严格，可删除单字词条，仅保留"停下/停车"等多字词。

---

## 4. 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `threshold` | double | `0.6` | 命令命中的最小相似度，范围 0.0~1.0 |
| `keywords_file` | string | `''` | 关键字 YAML 路径；为空时使用包内 `config/keywords.yaml` |

阈值调节建议：

- **经常误触发**（把无关话识别成命令）：调高阈值，如 `0.7` ~ `0.8`；
- **该识别的命令识别不到**：调低阈值，如 `0.4` ~ `0.5`；
- 一般中文命令场景 `0.6` 是较平衡的取值。

---

## 5. 环境依赖与编译

- ROS 2（开发环境为 **Jazzy**，系统 Python，**不使用 conda**）
- Python 依赖：`pypinyin`（模糊音匹配）、`pyyaml`（读取配置）

安装依赖：

```bash
pip3 install --break-system-packages pypinyin pyyaml
```

编译：

```bash
cd ~/wheeltec/wheeltec_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_keywords
source install/setup.bash
```

> `pypinyin` 为可选依赖：若未安装，节点会自动退化为仅字符相似度匹配（拼音匹配关闭，启动日志会显示 `pinyin matching: off`）。

---

## 6. 使用方法

### 6.1 单独启动本节点

```bash
# 使用默认配置与阈值
ros2 launch asr_keywords asr_keywords.launch.py

# 自定义阈值
ros2 launch asr_keywords asr_keywords.launch.py threshold:=0.7

# 使用外部关键字配置
ros2 launch asr_keywords asr_keywords.launch.py keywords_file:=/path/to/keywords.yaml
```

也可以直接运行节点：

```bash
ros2 run asr_keywords keyword_node --ros-args \
  -p threshold:=0.6 \
  -p keywords_file:=/path/to/keywords.yaml
```

### 6.2 与 asr_sherpa 一起启动（整条语音链路）

本包提供总启动文件 [asr_all.launch.py](launch/asr_all.launch.py)，同时启动语音识别与命令匹配：

```bash
ros2 launch asr_keywords asr_all.launch.py
```

总启动文件还可传入两侧参数：

```bash
ros2 launch asr_keywords asr_all.launch.py \
  use_int8:=true \
  input_device:="Yundea" \
  threshold:=0.6
```

### 6.3 查看结果

```bash
ros2 topic echo /asr_command       # 最终命令
ros2 topic echo /asr/final_text   # 原始识别文字（调试用）
```

---

## 7. 匹配效果示例

| ASR 识别文字 | 发布命令 | 说明 |
|---|---|---|
| 小车前进 | 小车向前 | 同义词归一 |
| 小车后退 / 倒车 | 小车向后 | 同义词归一 |
| 小车左拐 | 小车左转 | 同义词归一 |
| 向右转 | 小车右转 | 省略"小车"的说法 |
| 刹车 / 停下 | 小车停止 | 同义词归一 |
| 小车钱进 | 小车向前 | 钱/前同音（模糊音） |
| 小车象后 | 小车向后 | 象/向同音（模糊音） |
| 小车向潜 | 小车向前 | 潜/前同音（读错字） |
| 请帮我小车停下来 | 小车停止 | 句中带语气词 |
| 今天天气怎么样 | （不发布） | 无匹配命令 |
| 你好 / 目前的情况 | （不发布） | 低于阈值 |

---

## 8. 功能包结构

```
src/asr_keywords/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── asr_keywords
├── config/
│   └── keywords.yaml           # 关键字配置（可扩展）
├── launch/
│   ├── asr_keywords.launch.py  # 仅启动命令匹配节点
│   └── asr_all.launch.py       # 同时启动 asr_sherpa + 命令匹配
└── asr_keywords/
    ├── __init__.py
    └── keyword_node.py         # 核心匹配节点
```

核心函数（`keyword_node.py`）：

| 函数 | 作用 |
|---|---|
| `normalize()` | 去标点空白 |
| `to_pinyin()` | 转无声调拼音音节序列 |
| `similarity()` / `pinyin_similarity()` | 字符 / 拼音音节相似度 |
| `best_match_score()` | 滑窗计算文字与某词条的最高分 |
| `load_commands()` | 读取并校验 YAML 命令配置 |

---

## 9. 常见问题

1. **命令没识别出来**：先 `ros2 topic echo /asr/final_text` 确认原始文字；再适当调低 `threshold`，或把该说法加入对应命令组的 `synonyms`。
2. **经常误触发**：调高 `threshold`；删除过短 / 过宽的同义词（如单字"停"）。
3. **启动报 `Keywords file not found`**：检查 `keywords_file` 路径；不传该参数会使用包内默认配置。
4. **拼音模糊匹配不生效**：确认已安装 `pypinyin`，且启动日志显示 `pinyin matching: on`。
5. **修改 YAML 后不生效**：配置在 `colcon build` 时安装到 `install/`，改完源码配置需重新编译；或直接用 `keywords_file` 指向外部文件（无需编译）。
