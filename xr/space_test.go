package xr

import (
	"math"
	"testing"
)

func testSpaceManager() *SpaceManager {
	m := NewSpaceManager()
	m.SetOrigin(SpaceLocal, IdentityPose())
	m.SetOrigin(SpaceFloor, Pose{Position: Vec3{0, -1.6, 0}, Orientation: QuatIdentity()})
	m.SetOrigin(SpaceStage, Pose{
		Position:    Vec3{2, -1.6, 1},
		Orientation: QuatFromAxisAngle(Vec3{Y: 1}, math.Pi/4),
	})
	return m
}

func TestSpaceConvertRoundTrip(t *testing.T) {
	m := testSpaceManager()
	p := Pose{Position: Vec3{0.3, 1.2, -0.4}, Orientation: QuatFromAxisAngle(Vec3{X: 1, Y: 1}, 0.9)}
	spaces := []SpaceID{SpaceLocal, SpaceFloor, SpaceStage}
	for _, a := range spaces {
		for _, b := range spaces {
			back := m.Convert(m.Convert(p, a, b), b, a)
			if !poseAlmost(back, p, 1e-9) {
				t.Fatalf("round trip %s->%s->%s failed: got %+v want %+v", a, b, a, back, p)
			}
		}
	}
}

func TestSpaceConvertWorldConsistency(t *testing.T) {
	m := testSpaceManager()
	p := Pose{Position: Vec3{1, 1.5, 0.5}, Orientation: QuatFromAxisAngle(Vec3{Y: 1}, 0.3)}
	// 同一物理姿态：origin[a]*p == origin[b]*convert(p,a,b)
	for _, b := range []SpaceID{SpaceLocal, SpaceFloor, SpaceStage} {
		worldA := m.Origin(SpaceLocal).Compose(p)
		worldB := m.Origin(b).Compose(m.Convert(p, SpaceLocal, b))
		if !poseAlmost(worldA, worldB, 1e-9) {
			t.Fatalf("world pose inconsistent via %s: %+v vs %+v", b, worldA, worldB)
		}
	}
}

func TestRelocateKeepsAnchorsStable(t *testing.T) {
	m := testSpaceManager()
	anchorLocal := Pose{Position: Vec3{1, 1, 1}, Orientation: QuatFromAxisAngle(Vec3{Y: 1}, 0.5)}
	id := m.CreateAnchor(SpaceLocal, anchorLocal)

	worldBefore := m.Origin(SpaceStage).Compose(mustAnchor(t, m, id, SpaceStage))

	// 重定位本地空间原点
	m.Relocate(SpaceLocal, Pose{Position: Vec3{5, 0, -2}, Orientation: QuatFromAxisAngle(Vec3{Y: 1}, 1.0)})

	worldAfter := m.Origin(SpaceStage).Compose(mustAnchor(t, m, id, SpaceStage))
	if !poseAlmost(worldBefore, worldAfter, 1e-9) {
		t.Fatalf("anchor world pose shifted after relocate: %+v -> %+v", worldBefore, worldAfter)
	}
	// 在未调整的空间中锚点表达完全不变
	if !poseAlmost(mustAnchor(t, m, id, SpaceStage), m.Origin(SpaceStage).Inverse().Compose(worldBefore), 1e-9) {
		t.Fatal("anchor expression in stage space changed unexpectedly")
	}
}

func mustAnchor(t *testing.T, m *SpaceManager, id AnchorID, space SpaceID) Pose {
	t.Helper()
	p, ok := m.AnchorPose(id, space)
	if !ok {
		t.Fatalf("anchor %d missing", id)
	}
	return p
}
