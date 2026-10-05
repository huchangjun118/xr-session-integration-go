package xr

// DeviceType 标识设备类别。
type DeviceType int

const (
	DeviceTypeHead DeviceType = iota
	DeviceTypeController
	DeviceTypeHand
	DeviceTypeEye
)

func (t DeviceType) String() string {
	switch t {
	case DeviceTypeHead:
		return "head"
	case DeviceTypeController:
		return "controller"
	case DeviceTypeHand:
		return "hand"
	case DeviceTypeEye:
		return "eye"
	default:
		return "unknown"
	}
}

// TrackingState 标识设备当前的追踪状态。
type TrackingState int

const (
	// TrackingStateNotTracked 表示追踪丢失，此时姿态无效（不是零值语义）。
	TrackingStateNotTracked TrackingState = iota
	// TrackingStateTracked 表示位姿有效。
	TrackingStateTracked
)

func (s TrackingState) String() string {
	switch s {
	case TrackingStateTracked:
		return "tracked"
	case TrackingStateNotTracked:
		return "not-tracked"
	default:
		return "unknown"
	}
}

// DeviceUID 是设备的稳定身份标识，热插拔重连后保持不变。
type DeviceUID string

// DeviceInfo 描述一台已接入设备的静态信息。
type DeviceInfo struct {
	UID  DeviceUID
	Type DeviceType
	Name string
}

// DeviceState 是设备在某一时刻的统一状态模型。
type DeviceState struct {
	Info     DeviceInfo
	Attached bool
	Tracking TrackingState
	// Pose 仅在 Tracking == TrackingStateTracked 时有效；
	// 追踪丢失时保持零值且不得被当作有效姿态读取。
	Pose TimedPose
}
