# xr-session-integration-go

统一的 **会话与追踪集成层**。把来自不同来源（头显、手柄、手部追踪、眼部追踪）、
不同格式与更新频率的设备数据归一为一致的设备/姿态模型，供下游在持续更新与输入
交错的情况下稳定取用。

* 纯进程内实现：输入来自带种子的模拟输入流，不依赖真实设备或外部服务。
* 确定性：相同输入流 + 相同种子 + 相同配置 ⇒ 逐条一致的快照序列与哈希清单。
* 仅使用 Go 标准库。

## 快速开始

```go
cfg := xr.DefaultConfig()
cfg.Seed = 42

e  := xr.NewEngine(cfg)                 // 集成引擎（会话 + 空间 + 姿态）
sim := xr.NewSimulator(xr.DefaultSimConfig(42))
rec := xr.NewRecorder()                 // 逐快照确定性指纹

manifest, _ := sim.Run(e, rec)
fmt.Println(manifest.Hash)              // 同种子/配置下恒定
```

直接运行演示（两次同种子运行哈希一致，并演示算法版本不一致时的明确判定）：

```
go run ./cmd/xrdemo
```

## 架构

```
FrameInput(各来源)                生命周期事件
      │                              │
      ▼                              ▼
 registry ── 归一身份/绑定 ──▶ Session(确定状态机)
      │
      ▼
 poseBuffer(每设备有序历史：去重/乱序/迟到窗口)
      │
      ▼
 predictor(插值 / 常速度预测 / 丢失无效化)
      │
      ▼
 SpaceManager(local / floor / stage 换算 + 锚点)
      │
      ▼
 Engine.Advance() ──▶ 不可变 Snapshot（深拷贝给慢读取方）
                          │
                          ▼
                     Recorder 哈希 / Manifest
```

关键文件：

| 文件 | 职责 |
| --- | --- |
| `pkg/xr/types.go` | 设备、姿态、快照等归一模型 |
| `pkg/xr/session.go` | 会话生命周期状态机 |
| `pkg/xr/registry.go` | 设备归一身份、热插拔、绑定持久化 |
| `pkg/xr/posebuffer.go` | 每设备有序姿态历史与送达纪律 |
| `pkg/xr/predictor.go` | 插值、预测、追踪丢失语义 |
| `pkg/xr/space.go` | 参考空间换算与锚点 |
| `pkg/xr/engine.go` | 并发数据通路与一致快照 |
| `pkg/xr/simulator.go` | 确定性多源模拟输入流（含故障注入） |
| `pkg/xr/reproducibility.go` | 快照指纹、复现清单与可复现判定 |

## 会话模型

状态与**唯一合法转移**（其余转移被拒绝，且不产生任何副作用）：

```
                 begin        focus         pause          resume
  idle ───────────────▶ running ───────▶ focused ───────▶ paused ───────▶ running
                           │  ▲                          │                  │
                           │  └──────── pause ───────────┘                  │
                           └──────────────────── end ───────────────────────┴──▶ ended
```

* `running --pause--> paused` 同样合法（未获焦也可挂起）。
* `ended` 为终态，其后除重复 `end` 外的一切事件都被拒绝。
* 失败可区分原因（`errors.Is`）：
  * `ErrIllegalTransition`：无此合法边，附带机器可读 `Reason`
    （`not_started`、`focus_while_paused`、`unknown_event` …）；
  * `ErrAlreadyInState`：重复请求（如已 `focused` 再 `focus`）；
  * `ErrSessionEnded`：终态后的事件。
* 所有调用经互斥串行化：并发/重复转移最多只有一次真正生效，
  每次尝试（含被拒绝的）都记录在 `Session.History()`，便于复现排查。
* `paused`/`idle` 期间输入仍被接收和保留，但 `Advance` 不发布新快照。

## 参考空间语义

内部只保留一个不可变的 **base 物理坐标系**（y-up，流开始时建立）。设备姿态
永远以 base 存储，因此物理轨迹连续，与参考空间调整无关。

* **local**：等同 base。
* **floor**：base 向下平移估计的地面高度，`floorInBase = T(0, floorHeight, 0)`；
  物理上位于地面的点在 floor 空间 y=0。
* **stage**：完全可调的帧，`stageInBase` 为其变换。

换算恒等关系（同一物理姿态跨空间自洽）：

```
p_space = (spaceInBase)^-1 ∘ p_base
p_base  =  spaceInBase     ∘ p_space      // 往返误差为 0
```

**原点调整不错位/不跳变**：锚点（`SpaceManager.Anchor`）在锚定时即转换并
以 base 物理姿态保存。调整地面高度或 stage 原点只改变其输出坐标表示，
其物理姿态逐位不变；重新渲染后的 stage 坐标变化量恰好等于原点位移。

## 姿态送达纪律与丢失语义

每设备一个按时间戳有序的历史缓冲：

* **重复帧**（时间戳已存在）拒绝；
* **迟到但在窗口内**（`ResendMaxLate`，默认 500ms）插入正确的历史槽位，
  可用于更准确的插值；**超出窗口**拒绝；
* 因此时间戳较旧的帧永远无法覆盖最新姿态。

追踪丢失：

* 丢失样本以 `TrackingState = TrackInvalid` 表示，并携带**最后一个有效姿态**，
  绝不返回零值；
* 预测/插值区间内任何位置出现丢失帧，区间输出都判为无效；
* 恢复后因输出沿最后有效姿态与物理轨迹连续，不会出现姿态跳变（测试以容差
  断言恢复前后的位置差）。

## 预测与插值（可配置）

* `PredictionEnabled=false`：输出退化为未经处理的最新原始样本，输出时间戳
  即样本时间戳。
* 开启后：已知样本之间做线性位置插值 + 四元数 slerp；超出最新样本时使用
  常速度模型（由相邻样本或样本自带速度估计）外推。
* 输出时间戳恒为请求目标时间 `now + PredictAhead`，与所用预测步长一致。
* `Engine.Advance` 的时钟单调：倒退时间被拒绝（`ok=false`），已发布快照
  不变，因此输出不会倒退。

## 热插拔

* 设备身份由逻辑角色（kind + handedness）规范化为稳定 ID
  （如 `controller/right`），源键只用于映射；重新插拔身份不变。
* 绑定/配置保存在设备身份上，拔除期间不丢失，重插后原样可用。
* 拔除期间到达该源的数据被直接丢弃，永远不会错位到其他设备；其他设备的
  更新与在线状态不受影响。

## 并发模型

一把 `sync.RWMutex` 守护全部可变状态；`Advance` 在写锁下一次性评估所有
设备，构建完整快照后作为不可变值发布；读取方 `Snapshot()` 获得深拷贝，
可按任意慢速节奏处理而不持有锁，因此**慢读取不阻塞更新推进**。
快照内所有设备帧来自同一评估时刻，不会出现新旧混合。

## 确定性与复现

* 模拟器使用合成单调时钟（非墙上时间）与种子 PRNG，轨迹是种子与时间的
  纯函数；更新频率默认 头显 90Hz / 手柄 72Hz / 手部 30Hz / 眼动 60Hz。
* `Recorder` 对每个快照生成与 map 遍历顺序无关的规范化字符串哈希
  （设备按注册顺序，绑定按键排序）。
* `Manifest` 记录种子、算法版本、空间、预测/插值配置、快照数与总哈希；
  `CompareManifolds` 给出 `Reproducible` 与可区分原因
  （`algorithm_version_mismatch`、`seed_mismatch`、`config_mismatch`、
  `output_hash_mismatch` …）。算法迭代后旧结果可复现，或被**明确判定为
  不可复现**，而不是静默产生差异。

## 配置项（`xr.Config`）

| 字段 | 默认值 | 含义 |
| --- | --- | --- |
| `Space` | `floor` | 输出参考空间：`local` / `floor` / `stage` |
| `PredictionEnabled` | `true` | 是否启用预测；关闭后输出原始最新样本 |
| `PredictAhead` | `20ms` | 预测步长，输出时间戳 = now + 该值 |
| `InterpolationEnabled` | `true` | 相邻样本间是否插值 |
| `ResendMaxLate` | `500ms` | 迟到帧可被接受的最大年龄 |
| `BufferLength` | `64` | 每设备姿态历史长度 |
| `Seed` | `1` | 模拟器/引擎随机行为种子 |
| `AlgorithmVersion` | `xr-algo-1` | 算法版本戳，用于复现判定 |

模拟器配置（`xr.SimConfig`）：`Seed`、`Duration`(3s)、`Tick`(10ms)、
四个来源各自的频率，以及 `InjectFaults`（在固定时刻注入重复/迟到/乱序帧、
追踪丢失窗口、手柄热插拔循环、stage recenter、pause/resume）。

## 测试

```
go test -race ./...
```

覆盖：非法/重复/并发会话转移；四元数与三空间自洽及原点调整锚点不跳变；
乱序/重复/迟到缓冲；追踪丢失无效化（非零）与恢复不跳变；预测关闭退化为
原始输出、输出时间戳与单调性；插值；热插拔身份/绑定/不错位；缺失来源；
快照跨设备一致；并发慢读取无撕裂（race）；同种子确定性复现与配置/版本
不可复现的明确判定。
