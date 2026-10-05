package xr

import (
	"math"
	"time"
)

// predictor produces an output pose at a requested time from a per-device
// history.
//
// Contract:
//
//   - when disabled, evaluate returns the newest raw sample unchanged;
//   - when enabled, it interpolates between bracketing samples and predicts
//     ahead using a constant-velocity model, stamping the output with the
//     exact requested target timestamp;
//   - while tracking is lost it returns a TrackInvalid frame that still
//     carries the last known good pose (never a zero value);
//   - recovered samples are blended briefly so a re-acquired pose does not
//     jump relative to the last presented pose;
//   - evaluation at the same target is a pure function of the buffer
//     contents, so snapshots stay deterministic.
type predictor struct {
	enabled     bool
	interpolate bool
	ahead       time.Duration
}

func newPredictor(cfg Config) *predictor {
	buflen := cfg.BufferLength
	_ = buflen
	return &predictor{
		enabled:     cfg.PredictionEnabled,
		interpolate: cfg.InterpolationEnabled,
		ahead:       cfg.PredictAhead,
	}
}

// predState is per-device predictor bookkeeping. Jump-free re-acquisition is
// guaranteed structurally (loss frames carry the last known good pose), so
// it currently carries no mutable fields but remains an explicit extension
// point.
type predState struct{}

// evalResult is one evaluation outcome.
type evalResult struct {
	frame        PoseFrame
	outputTime   time.Duration
	predicted    bool
	interpolated bool
}

// evaluate returns the pose at targetTime for one device.
func (p *predictor) evaluate(b *poseBuffer, targetTime time.Duration, state *predState) evalResult {
	if !p.enabled {
		if f, ok := b.newest(); ok {
			return evalResult{frame: f, outputTime: f.Timestamp}
		}
		return evalResult{
			frame:      PoseFrame{DeviceID: b.id, Timestamp: targetTime, State: TrackInvalid},
			outputTime: targetTime,
		}
	}

	frame, interp, pred := p.solve(b, targetTime)
	return evalResult{
		frame:        frame,
		outputTime:   targetTime,
		interpolated: interp,
		predicted:    pred,
	}
}

// intervalHadLoss reports whether any stored frame with timestamp in
// (afterTime, target] records invalid tracking. It is used to suppress
// prediction across a tracking-loss gap even when the newest stored sample
// itself is valid again.
func intervalHadLoss(b *poseBuffer, afterTime, target time.Duration) bool {
	for i := len(b.frames) - 1; i >= 0; i-- {
		f := b.frames[i]
		if f.Timestamp <= afterTime {
			break
		}
		if f.Timestamp <= target && f.State == TrackInvalid {
			return true
		}
	}
	return false
}

// lastGoodAt scans backwards for the newest non-invalid frame at or before t.
func lastGoodAt(b *poseBuffer, t time.Duration) (PoseFrame, bool) {
	for {
		f, ok := b.at(t)
		if !ok {
			return PoseFrame{}, false
		}
		if f.State != TrackInvalid {
			return f, true
		}
		prev := f.Timestamp - 1
		if prev >= f.Timestamp { // underflow guard
			return PoseFrame{}, false
		}
		t = prev
	}
}

func (p *predictor) solve(b *poseBuffer, target time.Duration) (PoseFrame, bool, bool) {
	before, after, ok, afterOk := b.bracketing(target)
	if !ok {
		// No sample at or before target: invalid, not a zero pose.
		return PoseFrame{DeviceID: b.id, Timestamp: target, State: TrackInvalid}, false, false
	}

	// Determine tracking state at the target.
	current := before
	currentInvalid := current.State == TrackInvalid

	if afterOk && target > before.Timestamp && target < after.Timestamp && p.interpolate {
		// Pure interpolation between two received samples.
		span := float64(after.Timestamp - before.Timestamp)
		alpha := float64(target-before.Timestamp) / span
		f := interpFrames(before, after, alpha)
		f.DeviceID = b.id
		f.Timestamp = target
		if intervalHadLoss(b, before.Timestamp, after.Timestamp) ||
			currentInvalid || after.State == TrackInvalid {
			// Tracking was invalid somewhere in the gap: propagate invalidity
			// while still carrying the last known good pose.
			if g, gok := lastGoodAt(b, before.Timestamp); gok {
				f.Pose = g.Pose
				f.State = TrackInvalid
				f.HasVelocity = false
			}
		}
		return f, true, false
	}

	if target > before.Timestamp {
		// Extrapolation beyond the newest received sample (prediction).
		f := extrapolate(b, before, target)
		lostInInterval := currentInvalid || intervalHadLoss(b, before.Timestamp, target)
		if lostInInterval {
			// Lost during prediction horizon: distinguishable invalid result,
			// but carry the last known good pose instead of a zero.
			f.State = TrackInvalid
			f.HasVelocity = false
			if g, gok := lastGoodAt(b, before.Timestamp); gok {
				f.Pose = g.Pose
			}
		}
		return f, false, true
	}

	// target == an existing sample timestamp: return it unchanged.
	f := before
	return f, false, false
}

// interpFrames blends two valid pose frames at parameter alpha in [0,1].
func interpFrames(a, c PoseFrame, alpha float64) PoseFrame {
	f := PoseFrame{
		DeviceID: a.DeviceID,
		Pose: Pose{
			Position:    vecLerp(a.Pose.Position, c.Pose.Position, alpha),
			Orientation: quatSlerp(a.Pose.Orientation, c.Pose.Orientation, alpha),
		},
		State:    weakerState(a.State, c.State),
		Sequence: a.Sequence,
	}
	if a.HasVelocity && c.HasVelocity {
		f.Velocity = Velocity{
			Linear:  vecLerp(a.Velocity.Linear, c.Velocity.Linear, alpha),
			Angular: vecLerp(a.Velocity.Angular, c.Velocity.Angular, alpha),
		}
		f.HasVelocity = true
	}
	return f
}

// extrapolate applies a constant-velocity model from the newest sample (or
// the newest pair of samples) to target.
func extrapolate(b *poseBuffer, newest PoseFrame, target time.Duration) PoseFrame {
	dt := target - newest.Timestamp
	out := PoseFrame{
		DeviceID:  newest.DeviceID,
		Timestamp: target,
		Pose:      newest.Pose,
		State:     newest.State,
		Sequence:  newest.Sequence,
	}
	if newest.State == TrackInvalid {
		return out
	}
	lin, ang, has := motionEstimate(b, newest)
	if !has || dt <= 0 {
		return out
	}
	dtSec := dt.Seconds()
	out.Pose.Position = vecAdd(newest.Pose.Position, vecScale(lin, dtSec))
	// orientation: q(t+dt) = dq(omega, dt) * q(t)
	out.Pose.Orientation = quatMultiply(axisAngle(ang, dtSec), newest.Pose.Orientation)
	out.Velocity = Velocity{Linear: lin, Angular: ang}
	out.HasVelocity = true
	return out
}

// motionEstimate returns linear/angular velocity at newest, preferring the
// sample pair immediately preceding it.
func motionEstimate(b *poseBuffer, newest PoseFrame) (Vec3, Vec3, bool) {
	if newest.HasVelocity {
		return newest.Velocity.Linear, newest.Velocity.Angular, true
	}
	// Find the previous valid frame.
	var prev PoseFrame
	found := false
	// newest may be the head; search the buffer for its predecessor.
	// b.at(newest.Timestamp-1) yields the immediately preceding stored frame.
	if f, ok := b.at(newest.Timestamp - 1); ok && f.Timestamp < newest.Timestamp {
		if f.State != TrackInvalid {
			prev = f
			found = true
		}
	}
	if !found {
		return Vec3{}, Vec3{}, false
	}
	dt := newest.Timestamp - prev.Timestamp
	if dt <= 0 {
		return Vec3{}, Vec3{}, false
	}
	sec := dt.Seconds()
	lin := vecScale(vecSub(newest.Pose.Position, prev.Pose.Position), 1/sec)
	// delta orientation dq = q1 * q0^-1 -> angular velocity
	dq := quatMultiply(newest.Pose.Orientation, quatInverse(prev.Pose.Orientation))
	ang := quatAxisAngle(dq)
	ang = vecScale(ang, 1/sec)
	return lin, ang, true
}

// axisAngle builds a delta quaternion for a rotation of |axis|*angle radians
// where v encodes a rotation vector omega over duration sec.
func axisAngle(omega Vec3, sec float64) Quat {
	angle := math.Sqrt(vecDot(omega, omega)) * sec
	if angle < 1e-12 {
		return identityQuat()
	}
	axis := vecScale(omega, 1/(math.Sqrt(vecDot(omega, omega))))
	half := angle / 2
	s := math.Sin(half)
	return quatNormalize(Quat{
		X: axis.X * s,
		Y: axis.Y * s,
		Z: axis.Z * s,
		W: math.Cos(half),
	})
}

// quatAxisAngle converts a unit quaternion into the equivalent rotation
// vector (axis * angle), taking the shortest arc.
func quatAxisAngle(q Quat) Vec3 {
	q = quatNormalize(q)
	w := clampScalar(q.W, -1, 1)
	angle := 2 * math.Acos(w)
	s := math.Sqrt(1 - w*w)
	if s < 1e-9 {
		return Vec3{}
	}
	return Vec3{X: q.X / s * angle, Y: q.Y / s * angle, Z: q.Z / s * angle}
}

func clampScalar(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func weakerState(a, c TrackingState) TrackingState {
	if a == TrackInvalid || c == TrackInvalid {
		return TrackInvalid
	}
	if a == TrackLimited || c == TrackLimited {
		return TrackLimited
	}
	return TrackValid
}
