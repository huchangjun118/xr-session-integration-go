// Package xr implements a unified session and tracking integration layer for
// head-mounted displays, controllers, hand tracking and eye tracking.
//
// All processing runs in-process: inputs come from simulated device streams,
// no real hardware or external services are required and every run with the
// same seed and configuration produces identical results.
package xr

import (
	"errors"
	"time"
)

// Kind identifies the role of a tracked device.
type Kind uint8

const (
	KindHeadset Kind = iota
	KindController
	KindHand
	KindEyes
)

func (k Kind) String() string {
	switch k {
	case KindHeadset:
		return "headset"
	case KindController:
		return "controller"
	case KindHand:
		return "hand"
	case KindEyes:
		return "eyes"
	default:
		return "unknown"
	}
}

// Handedness distinguishes the left/right member of a paired device.
type Handedness uint8

const (
	HandNone Handedness = iota
	HandLeft
	HandRight
)

func (h Handedness) String() string {
	switch h {
	case HandLeft:
		return "left"
	case HandRight:
		return "right"
	default:
		return "none"
	}
}

// ReferenceSpace identifies the coordinate frame a pose is reported in.
type ReferenceSpace uint8

const (
	// SpaceLocal: origin at the device position at stream start, y-up, no
	// stage origin concept.
	SpaceLocal ReferenceSpace = iota
	// SpaceFloor: same axes as local but origin placed on the estimated floor.
	SpaceFloor
	// SpaceStage: application-defined play space with a movable origin.
	SpaceStage
)

func (s ReferenceSpace) String() string {
	switch s {
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

// TrackingState is the validity state of a normalized pose.
type TrackingState uint8

const (
	// TrackInvalid means no usable pose exists (no zero-value pose is ever
	// reported as valid).
	TrackInvalid TrackingState = iota
	// TrackLimited means a pose is available but tracking quality is degraded.
	TrackLimited
	// TrackValid means a full six-degree-of-freedom pose is available.
	TrackValid
)

func (t TrackingState) String() string {
	switch t {
	case TrackValid:
		return "valid"
	case TrackLimited:
		return "limited"
	default:
		return "invalid"
	}
}

// Quat is a unit quaternion (real part W).
type Quat struct {
	X, Y, Z, W float64
}

// Vec3 is a 3D vector.
type Vec3 struct {
	X, Y, Z float64
}

// Pose is a rigid transform: position plus orientation.
type Pose struct {
	Position    Vec3
	Orientation Quat
}

// Velocity is an optional linear/angular motion estimate.
type Velocity struct {
	Linear  Vec3
	Angular Vec3
}

// DeviceID is the stable identity of a device. Re-plugging the same physical
// device keeps the same DeviceID.
type DeviceID string

// Device describes the normalized model of a single tracked entity.
type Device struct {
	ID         DeviceID
	Kind       Kind
	Hand       Handedness
	SourceName string
	// Online reflects current hot-plug presence.
	Online bool
	// Bound stores stable per-device binding/configuration that survives
	// unplug/replug.
	Bindings map[string]string
}

// PoseFrame is a normalized pose sample belonging to one device at one time.
type PoseFrame struct {
	DeviceID  DeviceID
	Timestamp time.Duration // monotonic stream time
	Pose      Pose
	Velocity  Velocity
	State     TrackingState
	// HasVelocity reports whether Velocity carries a usable estimate.
	HasVelocity bool
	// Sequence is the per-device monotonically increasing frame number.
	Sequence uint64
}

// DeviceSnapshot is one device's state within a consistent Snapshot.
type DeviceSnapshot struct {
	Device Device
	Frame  PoseFrame
	// Present reports whether a pose frame (valid or not) exists for the device.
	Present bool
}

// Snapshot is a point-in-time, self-consistent view of every device. All
// frames in a snapshot were produced from the same evaluation step, so a
// reader never sees a mix of old and new devices.
type Snapshot struct {
	// Time is the output timestamp of this snapshot.
	Time time.Duration
	// Devices is ordered by the canonical device registration order so that
	// snapshots compare deterministically.
	Devices []DeviceSnapshot
	// Space is the reference space the poses are expressed in.
	Space ReferenceSpace
	// SessionState is the session state at snapshot time.
	SessionState SessionState
	// Index is the monotonically increasing snapshot number.
	Index uint64
}

// ErrXr is the package error marker; all errors returned by this package can
// be matched against it with errors.Is.
var ErrXr = errors.New("xr")

// FrameInput is one raw sample delivered by a simulated device source before
// normalization into the internal device model.
type FrameInput struct {
	DeviceKey   string // source-specific identifier
	Source      SourceKind
	Hand        Handedness
	Timestamp   time.Duration
	Pose        Pose
	Velocity    Velocity
	HasVelocity bool
	State       TrackingState
	// Lost marks an explicit tracking-loss event for this device.
	Lost bool
	// Sequence is the source sequence number when available.
	Sequence uint64
}

// SourceKind mirrors Kind for the raw input layer.
type SourceKind = Kind
