package xr

import "sync"

// SpaceManager converts normalized poses (kept in one fixed internal "base"
// physical frame) into the configured reference space and tracks the floor
// height and the adjustable stage origin.
//
// Semantics:
//
//   - Base frame: fixed physical frame established when streaming starts,
//     y-up. Device poses are always stored in this frame, so their physical
//     trajectory is continuous regardless of reference-space adjustments.
//   - Local space: identical to the base frame.
//   - Floor space: the base frame translated down by the estimated floor
//     height; floorInBase = T(0, floorHeight, 0).
//   - Stage space: a fully adjustable frame; stageInBase is its transform.
//
// Because every conversion goes through the immutable base frame, one
// physical pose produces self-consistent results in every space, and moving
// the stage origin never teleports a physically stationary device or anchor.
type SpaceManager struct {
	mu          sync.RWMutex
	space       ReferenceSpace
	floorHeight float64
	stageInBase Pose
	anchors     map[string]Pose // anchor id -> physical base-frame pose
}

func NewSpaceManager(space ReferenceSpace) *SpaceManager {
	return &SpaceManager{
		space:       space,
		stageInBase: Pose{Orientation: identityQuat()},
		anchors:     make(map[string]Pose),
	}
}

func (m *SpaceManager) Space() ReferenceSpace {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.space
}

// SetSpace switches the output space. Base-frame storage is unaffected.
func (m *SpaceManager) SetSpace(space ReferenceSpace) {
	m.mu.Lock()
	m.space = space
	m.mu.Unlock()
}

// SetFloorHeight establishes the local->floor relationship. Poses already
// stored/anchored in the base frame keep their exact physical pose.
func (m *SpaceManager) SetFloorHeight(y float64) {
	m.mu.Lock()
	m.floorHeight = y
	m.mu.Unlock()
}

func (m *SpaceManager) FloorHeight() float64 {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.floorHeight
}

// AdjustStageOrigin moves the stage frame relative to the fixed base frame.
// Anchors are stored physically (in base), so they are never displaced.
func (m *SpaceManager) AdjustStageOrigin(origin Pose) {
	m.mu.Lock()
	m.stageInBase = origin
	m.mu.Unlock()
}

// StageOrigin returns the current stage frame expressed in base coordinates.
func (m *SpaceManager) StageOrigin() Pose {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.stageInBase
}

// frameTransform returns spaceInBase for the configured space.
func (m *SpaceManager) frameTransform() Pose {
	switch m.space {
	case SpaceFloor:
		return Pose{Position: Vec3{Y: m.floorHeight}, Orientation: identityQuat()}
	case SpaceStage:
		return m.stageInBase
	default:
		return Pose{Orientation: identityQuat()}
	}
}

// ToOutput converts one base-frame pose to the configured output space:
// p_space = spaceInBase^-1 ∘ p_base.
func (m *SpaceManager) ToOutput(p Pose) Pose {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return poseMultiply(poseInverse(m.frameTransform()), p)
}

// FromOutput converts an output-space pose back into the base frame:
// p_base = spaceInBase ∘ p_space.
func (m *SpaceManager) FromOutput(p Pose) Pose {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return poseMultiply(m.frameTransform(), p)
}

// Frame transforms a full pose frame into the output space without changing
// its tracking state, timestamps or velocity validity semantics.
func (m *SpaceManager) Frame(f PoseFrame) PoseFrame {
	out := f
	out.Pose = m.ToOutput(f.Pose)
	if f.HasVelocity {
		// Angular velocity is invariant under translation; linear velocity is
		// rotated into the output frame's axes.
		m.mu.RLock()
		q := m.frameTransform().Orientation
		m.mu.RUnlock()
		qInv := quatInverse(q)
		out.Velocity = Velocity{
			Linear:  quatRotate(qInv, f.Velocity.Linear),
			Angular: quatRotate(qInv, f.Velocity.Angular),
		}
	}
	return out
}

// Anchor records content given in current output coordinates and stores it as
// a physical base-frame pose, so later origin adjustments cannot displace it.
func (m *SpaceManager) Anchor(id string, outputPose Pose) Pose {
	base := m.FromOutput(outputPose)
	m.mu.Lock()
	m.anchors[id] = base
	m.mu.Unlock()
	return base
}

// AnchorBase returns the physical (base-frame) pose of an anchor.
func (m *SpaceManager) AnchorBase(id string) (Pose, bool) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	p, ok := m.anchors[id]
	return p, ok
}

// AnchorOutput renders an anchor in the current output space. Its physical
// pose is identical before and after any origin adjustment.
func (m *SpaceManager) AnchorOutput(id string) (Pose, bool) {
	base, ok := m.AnchorBase(id)
	if !ok {
		return Pose{}, false
	}
	return m.ToOutput(base), true
}
