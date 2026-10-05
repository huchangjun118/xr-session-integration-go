# xr-session-integration-go

统一的会话与追踪集成层：把来自不同来源（头显、手柄、手部追踪、眼部追踪）的设备数据
归一成一致的设备与姿态模型，供下游在持续更新与输入交错的情况下稳定取用。

整套能力只在单进程内运行：设备数据由进程内模拟输入流（`SimulatedSource` / `SliceSource`）
提供，不依赖真实设备与外部服务；相同输入流、相同种子与相同配置下输出逐条一致，可本地稳定复现。

## 包结构

```
xr/
  math.go        向量/四元数/姿态运算（Compose、Inverse、Slerp、速度外推）
  device.go      统一设备模型：DeviceType / TrackingState / DeviceState
  session.go     会话生命周期状态机
  space.go       参考空间换算与锚点（SpaceManager）
  predict.go     可配置姿态预测与恢复平滑
  stream.go      进程内模拟输入流（确定性、种子化）
  integrator.go  集成核心：接入、归一、热插拔、一致快照
  config.go      配置与复现性判定
```

## 会话模型

会话状态机：`Idle → Running → Focused → Paused → Running → Ended`。

合法转移：

| 当前状态 | 可转移到的状态 |
|---|---|
| Idle | Running, Ended |
| Running | Focused, Paused, Ended |
| Focused | Running, Paused, Ended |
| Paused | Running, Ended |
| Ended | （拒绝一切转移） |

- 非法转移返回 `*TransitionError`，`Reason` 区分
  `TransitionReasonIllegalTransition` 与 `TransitionReasonSessionEnded`，且不产生副作用。
- 转移到当前状态是幂等空操作（返回 nil）；并发转移调用只生效一次。

## 设备模型与追踪状态

所有来源归一为 `DeviceState{Info, Attached, Tracking, Pose}`：

- `Tracking == TrackingStateTracked` 时 `Pose` 有效；
- 追踪丢失时 `Tracking == TrackingStateNotTracked`，`Pose` 为零值且**不得**被当作有效姿态读取——
  无效结果通过标志位区分，而不是伪造的零值姿态。
- 缺失某类来源（如没有眼部追踪）不影响其余设备工作。

## 参考空间语义

支持 `SpaceLocal`（本地）、`SpaceFloor`（地面）、`SpaceStage`（舞台）三个参考空间。
所有姿态的世界表示为 `world = origin[space] * poseInSpace`，换算公式：

```
poseInTo = origin[to]^-1 * origin[from] * poseInFrom
```

- 同一物理姿态在不同空间下的换算自洽：`Convert(Convert(p, a, b), b, a) == p`。
- `RelocateSpace` / `SpaceManager.Relocate` 调整空间原点后：
  - 已锚定内容（`CreateAnchor`）的世界姿态不变，不错位、不跳变；
  - 设备的世界姿态保持连续（本地空间重居中时集成层自动施加传感器校正）。

## 输入规则（乱序 / 重复 / 迟到 / 丢失）

`Integrator.Ingest` 对每帧应用确定规则：

1. 未接入或已移除设备的帧直接丢弃，不会错位到其他设备；
2. 重复帧（来源序号不递增）丢弃；
3. 时间戳不晚于当前最新采样的迟到帧丢弃，旧帧不得覆盖新姿态；
4. 追踪丢失帧只更新追踪状态，不触碰姿态采样；
5. 追踪恢复时进入 `RecoveryBlend` 平滑窗口，输出从恢复时刻的预测姿态
   平滑收敛到新姿态，不出现跳变。

## 姿态预测与插值

`PredictConfig`：

| 配置项 | 含义 | 默认值 |
|---|---|---|
| `Enabled` | 关闭后输出退化为未经处理的原始姿态 | true |
| `Horizon` | 最大预测步长（秒），超出即截断 | 0.05 |
| `RecoveryBlend` | 追踪恢复后的平滑时长（秒） | 0.1 |

- 预测基于最新采样的线速度/角速度外推，读取时间早于采样时间时不倒退；
- 输出时间戳与实际所用预测步长一致（截断后时间戳反映截断步长）。

## 热插拔

- `Attach` / `Detach` 运行期接入、移除设备，其他设备状态不受影响；
- 设备以 `DeviceUID` 为稳定身份，重新接入后绑定（`SetBinding`）与配置不丢失；
- 移除期间到达的数据被丢弃，重新接入后只接受新数据。

## 并发模型

`Ingest` / `Attach` / `Detach` / `RelocateSpace` 与 `Snapshot` 可并发调用。
`Snapshot(space, now)` 在短临界区内一次性构建全部设备状态，返回不可变快照：
任一时刻读到的都是自洽的单个快照，不会新旧混杂；慢读取方不阻塞更新推进。

## 确定性复现

- 全部时间戳来自输入数据，不读墙钟；
- `SimulatedSource` 由种子驱动，相同种子产出逐条一致的帧流；
- `Config.CompatibleWith` 判定两份配置下旧输入是否可复现：
  版本或任一配置项不同即明确判定为不可复现。

## 测试

```sh
go test ./...          # 单元与集成测试
go test ./... -race    # 并发读取竞态检测
```

覆盖：乱序/重复/迟到输入、追踪丢失与恢复、参考空间换算与重定位、
热插拔、并发一致读取、确定性复现、预测开关与截断。

## 快速示例

```go
cfg := xr.DefaultConfig()
it := xr.NewIntegrator(cfg)
it.Attach(xr.DeviceInfo{UID: "head", Type: xr.DeviceTypeHead})

src := xr.NewSimulatedSource(xr.SimSourceConfig{
    UID: "head", Type: xr.DeviceTypeHead, Seed: 1, RateHz: 90, Duration: 1.0,
})
for f, ok := src.Next(); ok; f, ok = src.Next() {
    it.Ingest(f)
}
snap := it.Snapshot(xr.SpaceStage, 500_000_000)
head, _ := snap.Device("head")
if head.Tracking == xr.TrackingStateTracked {
    // 使用 head.Pose ...
}
```
