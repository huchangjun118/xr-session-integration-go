package xr

import "sync"

// SpaceID 标识参考空间。
type SpaceID int

const (
	// SpaceLocal 本地空间，跟随会话的本地原点（可被重定位/重居中）。
	SpaceLocal SpaceID = iota
	// SpaceFloor 地面空间，原点位于地面，重力对齐。
	SpaceFloor
	// SpaceStage 舞台空间，原点固定于用户定义的物理舞台中心。
	SpaceStage
)

func (id SpaceID) String() string {
	switch id {
	case SpaceLocal:
		return "local"
	case SpaceFloor:
		return "floor"
	case SpaceStage:
		return "stage"
	default:
		return "unknown"
	}
}

// AnchorID 标识一个锚定内容。
type AnchorID uint64

// anchor 以世界姿态存储，参考空间原点调整不影响其世界位姿。
type anchor struct {
	world Pose
}

// SpaceManager 维护各参考空间相对世界根的偏移，
// 支持空间间姿态换算与原点重定位（重定位不改变已锚定内容的世界姿态）。
//
// 语义：所有姿态的世界表示为 world = origin[space] * poseInSpace。
// 换算公式：poseInTo = origin[to]^-1 * origin[from] * poseInFrom。
type SpaceManager struct {
	mu      sync.RWMutex
	origins map[SpaceID]Pose
	anchors map[AnchorID]anchor
	next    AnchorID
}

func NewSpaceManager() *SpaceManager {
	return &SpaceManager{
		origins: map[SpaceID]Pose{
			SpaceLocal: IdentityPose(),
			SpaceFloor: IdentityPose(),
			SpaceStage: IdentityPose(),
		},
		anchors: map[AnchorID]anchor{},
		next:    1,
	}
}

// SetOrigin 设置某参考空间原点相对世界根的姿态。
func (m *SpaceManager) SetOrigin(id SpaceID, origin Pose) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.origins[id] = origin
}

// Origin 返回某参考空间原点相对世界根的姿态。
func (m *SpaceManager) Origin(id SpaceID) Pose {
	m.mu.RLock()
	defer m.mu.RUnlock()
	if p, ok := m.origins[id]; ok {
		return p
	}
	return IdentityPose()
}

// Convert 把姿态从 from 空间换算到 to 空间。
// 自洽性：对任意 p，Convert(Convert(p, a, b), b, a) == p（数值误差内）。
func (m *SpaceManager) Convert(p Pose, from, to SpaceID) Pose {
	if from == to {
		return p
	}
	m.mu.RLock()
	of, okF := m.origins[from]
	ot, okT := m.origins[to]
	m.mu.RUnlock()
	if !okF {
		of = IdentityPose()
	}
	if !okT {
		ot = IdentityPose()
	}
	world := of.Compose(p)
	return ot.Inverse().Compose(world)
}

// Relocate 调整某空间原点；已锚定内容的世界姿态保持不变，
// 其在被调整空间中的表达按原点差值平移，不发生错位或跳变。
func (m *SpaceManager) Relocate(id SpaceID, newOrigin Pose) {
	m.SetOrigin(id, newOrigin)
}

// CreateAnchor 在指定空间中锚定内容，返回锚点 ID。
// 锚点记录世界姿态，之后任意空间原点调整都不会使其错位。
func (m *SpaceManager) CreateAnchor(space SpaceID, p Pose) AnchorID {
	m.mu.Lock()
	defer m.mu.Unlock()
	o, ok := m.origins[space]
	if !ok {
		o = IdentityPose()
	}
	id := m.next
	m.next++
	m.anchors[id] = anchor{world: o.Compose(p)}
	return id
}

// AnchorPose 读取锚点在指定空间中的当前姿态。
func (m *SpaceManager) AnchorPose(id AnchorID, space SpaceID) (Pose, bool) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	a, ok := m.anchors[id]
	if !ok {
		return Pose{}, false
	}
	o, ok2 := m.origins[space]
	if !ok2 {
		o = IdentityPose()
	}
	return o.Inverse().Compose(a.world), true
}

// RemoveAnchor 移除锚点。
func (m *SpaceManager) RemoveAnchor(id AnchorID) {
	m.mu.Lock()
	defer m.mu.Unlock()
	delete(m.anchors, id)
}
