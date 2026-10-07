# 1. 语音指令节点 (voice_command_node)

## 功能

订阅 `/asr_command` 话题（`std_msgs/String`），按 **指令表**（`config/voice_commands.yaml`）
匹配文本并分发动作。收到"把苹果拿过来"时发布 `/pick_place/trigger`，启动抓取流程。

该节点是**多指令框架**：添加新语音指令只需修改 YAML 配置，无需改代码。

## 多指令框架（添加新指令流程）

### 步骤

1. 编辑 `config/voice_commands.yaml`，在 `commands` 列表追加条目：

```yaml
commands:
  - name: pick_banana                 # 指令唯一名
    patterns: ["把香蕉拿过来"]         # 触发文本（可多个别名）
    match_mode: exact                 # exact=完全一致 | contains=包含即触发
    enabled: true
    actions:                          # 依次执行的动作
      - type: topic                   # 动作1：发布话题
        topic: /pick_place/trigger
        msg_type: std_msgs/Bool
        data: true
      - type: service                 # 动作2（可选）：调用 Trigger 服务
        service: /some_extra_service
```

2. 热重载（无需重启节点）：

```bash
ros2 service call /pick_place/reload_commands std_srvs/srv/Trigger
```

3. 查看当前已加载指令：

```bash
ros2 service call /pick_place/list_commands std_srvs/srv/Trigger
```

### 匹配规则

- 收到文本先去除首尾空白
- 按表中顺序逐条匹配，**第一个命中的指令生效**（可用顺序控制优先级）
- `exact`：`text == pattern`。asr_keywords 发布的是标准命中的关键词，
  所以 exact 匹配该关键词即可
- `contains`：`pattern in text`，适合对原始句子做模糊匹配（pattern 要短，如"苹果"）

### 动作类型

| type | 字段 | 说明 |
|------|------|------|
| `topic` | `topic` `msg_type` `data` | 动态导入消息类并发布（消息需含 `data` 字段，如 Bool/String/Int32/Float64） |
| `service` | `service` | 调用 `std_srvs/Trigger` 服务（阻塞等待结果，超时 10s） |

> 扩展点：如需调用非 Trigger 类型服务或其他动作（如播放 TTS 反馈），在
> `_dispatch_action()` 中增加分支即可，指令表格式不变。

## 话题与服务

| 类型 | 名称 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `/asr_command` | `std_msgs/String` | 识别后的指令文本 |
| 发布 | `/pick_place/trigger` | `std_msgs/Bool` | 抓取流程触发信号（由指令表配置） |
| 发布 | `/pick_place/status` | `std_msgs/String` | 人类可读状态 |
| 服务 | `/pick_place/trigger_manual` | `std_srvs/Trigger` | 手动触发 `manual_trigger_command` 指定的指令 |
| 服务 | `/pick_place/reload_commands` | `std_srvs/Trigger` | 热重载指令表 |
| 服务 | `/pick_place/list_commands` | `std_srvs/Trigger` | 列出已加载指令 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sim_mode` | `true` | 模拟模式 |
| `commands_file` | `<pkg>/config/voice_commands.yaml` | 指令表路径 |
| `manual_trigger_command` | `pick_apple` | trigger_manual 触发的指令名 |

## 数据流

```
麦克风 → asr_sherpa → /asr/final_text
         → asr_keywords (关键词匹配) → /asr_command
         → voice_command_node (查指令表)
         → actions: /pick_place/trigger (Bool=True)
```

## 模拟测试方法

```bash
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=true

# 查看已加载指令
ros2 service call /pick_place/list_commands std_srvs/srv/Trigger

# 手动触发
ros2 service call /pick_place/trigger_manual std_srvs/srv/Trigger

# 发布模拟 ASR 指令（exact 或别名）
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '把苹果带过来'}"

# 非匹配指令（日志显示 No command matched，不触发）
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '天气怎么样'}"
```

## 真实使用方法

```bash
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=false
# 对着麦克风说"把苹果拿过来"即可触发
```

> 真实模式需确保麦克风已连接、ASR 模型在 `wheeltec_ws/src/models/` 中。
> 若语音栈的 asr_keywords 还没有收录你的新关键词，需要将其加入
> `speech_controller/config/commands.yaml`（属于 speech 包的配置）。
