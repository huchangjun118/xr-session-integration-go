package xr

import (
	"math"
	"math/rand"
	"time"
)

// EventKind classifies a simulated input event.
type EventKind uint8

const (
	EvFrame   EventKind = iota // a raw pose frame from one source
	EvAttach                   // a (re)plugged device becomes available
	EvDetach                   // a device is removed at runtime
	EvSession                  // a lifecycle event
	EvFloor                    // floor height estimate changes
	EvStage                    // stage origin is adjusted
	EvTick                     // advance the engine clock and publish a snapshot
)

// Event is one deterministic event produced by the simulated input stream.
type Event struct {
	At        time.Duration
	Kind      EventKind
	DeviceKey string
	DevKind   Kind
	Hand      Handedness
	Source    string
	Input     FrameInput
	Session   string
	Y         float64
	Origin    Pose
}

// Simulator is a seeded, in-process source of device frames covering a
// headset, two controllers, two hands and eye tracking, each at its own
// update rate, plus injected out-of-order/duplicate/late frames, tracking
// loss windows and hot-plug episodes.
//
// The simulator is driven by a synthetic monotonic clock (not wall time), and
// every trajectory value is a pure function of the seed and event index, so
// replaying with the same seed yields the exact same event stream.
type Simulator struct {
	rng    *rand.Rand
	cfg    SimConfig
	cursor time.Duration
	// next emission per source, keyed by canonical source name.
	next      map[string]time.Duration
	seq       map[string]uint64
	faultTick time.Duration
	done      bool
}

// SimConfig configures the simulated stream.
type SimConfig struct {
	Seed         int64
	Duration     time.Duration
	Tick         time.Duration
	HeadsetHz    int
	ControllerHz int
	HandHz       int
	EyeHz        int
	// InjectFaults enables duplicate/late/out-of-order frames, tracking-loss
	// windows and hot-plug episodes at fixed seed-driven moments.
	InjectFaults bool
}

// DefaultSimConfig returns deterministic default rates.
func DefaultSimConfig(seed int64) SimConfig {
	return SimConfig{
		Seed:         seed,
		Duration:     3 * time.Second,
		Tick:         10 * time.Millisecond,
		HeadsetHz:    90,
		ControllerHz: 72,
		HandHz:       30,
		EyeHz:        60,
		InjectFaults: true,
	}
}

// sourceDef describes one simulated logical source.
type sourceDef struct {
	key    string
	kind   Kind
	hand   Handedness
	hz     int
	source string
	phase  float64 // trajectory phase
}

func (s *Simulator) sources() []sourceDef {
	return []sourceDef{
		{"hmd", KindHeadset, HandNone, s.cfg.HeadsetHz, "sim:headset", 0},
		{"ctl-l", KindController, HandLeft, s.cfg.ControllerHz, "sim:controller", 0.5},
		{"ctl-r", KindController, HandRight, s.cfg.ControllerHz, "sim:controller", 1.0},
		{"hand-l", KindHand, HandLeft, s.cfg.HandHz, "sim:hand", 1.5},
		{"hand-r", KindHand, HandRight, s.cfg.HandHz, "sim:hand", 2.0},
		{"eyes", KindEyes, HandNone, s.cfg.EyeHz, "sim:eyes", 2.5},
	}
}

func NewSimulator(cfg SimConfig) *Simulator {
	if cfg.Tick <= 0 {
		d := DefaultSimConfig(cfg.Seed)
		cfg.Tick, cfg.Duration = d.Tick, d.Duration
		if cfg.HeadsetHz == 0 {
			cfg = d
		}
	}
	s := &Simulator{
		rng:  rand.New(rand.NewSource(cfg.Seed)),
		cfg:  cfg,
		next: make(map[string]time.Duration),
		seq:  make(map[string]uint64),
	}
	for _, d := range s.sources() {
		s.next[d.key] = 0
	}
	return s
}

// trajectory is the deterministic physical pose of a source at time t.
func (s *Simulator) trajectory(d sourceDef, t time.Duration) Pose {
	sec := t.Seconds()
	// Small slow head/body motion; each device has a fixed base offset and a
	// shared time phase so all sources describe one coherent physical scene.
	x := 0.05 * math.Sin(2*math.Pi*0.3*sec+d.phase)
	y := 0.02 * math.Sin(2*math.Pi*0.7*sec+d.phase)
	z := 0.05 * math.Cos(2*math.Pi*0.25*sec+d.phase)
	switch d.kind {
	case KindHeadset, KindEyes:
		return Pose{Position: Vec3{X: x, Y: 1.6 + y, Z: z}, Orientation: lookQuat(x)}
	case KindController:
		sign := 1.0
		if d.hand == HandLeft {
			sign = -1
		}
		return Pose{
			Position:    Vec3{X: sign*0.25 + x, Y: 1.1 + y, Z: 0.3 + z},
			Orientation: lookQuat(sign * 0.3),
		}
	case KindHand:
		sign := 1.0
		if d.hand == HandLeft {
			sign = -1
		}
		return Pose{
			Position:    Vec3{X: sign*0.22 + x, Y: 1.05 + y, Z: 0.32 + z},
			Orientation: lookQuat(sign * 0.2),
		}
	}
	return Pose{Orientation: identityQuat()}
}

// lookQuat builds a deterministic unit quaternion with a small yaw/pitch.
func lookQuat(yawComponent float64) Quat {
	yaw := 0.1 * yawComponent
	pitch := 0.03 * math.Sin(yawComponent+1)
	cy, sy := math.Cos(yaw/2), math.Sin(yaw/2)
	cp, sp := math.Cos(pitch/2), math.Sin(pitch/2)
	return quatNormalize(Quat{
		X: 0,
		Y: sy * cp,
		Z: sp * cy,
		W: cy * cp,
	})
}

// trackingLostAt reports whether a source is in a deterministic tracking-loss
// window at time t.
func trackingLostAt(d sourceDef, t time.Duration) bool {
	// Eye tracking and hands lose tracking briefly around 0.8-1.0s and
	// 2.0-2.15s respectively; deterministic, no RNG needed.
	switch d.key {
	case "eyes":
		return t >= 800*time.Millisecond && t < 1000*time.Millisecond
	case "hand-r":
		return t >= 2000*time.Millisecond && t < 2150*time.Millisecond
	}
	return false
}

// faultSchedule is the fixed ordered list of injected anomalies.
var faultSchedule = []time.Duration{
	500 * time.Millisecond,  // duplicate headset frame
	600 * time.Millisecond,  // late left-controller frame
	700 * time.Millisecond,  // out-of-order right-controller frame
	1200 * time.Millisecond, // right controller unplug
	1500 * time.Millisecond, // right controller replug
	1600 * time.Millisecond, // stage recenter
	1700 * time.Millisecond, // floor height (re)set
	2400 * time.Millisecond, // session pause
	2600 * time.Millisecond, // session resume
}

func init() {
	// keep the schedule sorted
	for i := 1; i < len(faultSchedule); i++ {
		if faultSchedule[i] < faultSchedule[i-1] {
			panic("faultSchedule must be sorted")
		}
	}
}

// Next returns the next event in time order, or false when the stream is
// exhausted.
func (s *Simulator) Next() (Event, bool) {
	// Fixed pre-roll events.
	if s.cursor == 0 {
		s.cursor = s.cfg.Tick
		return Event{At: 0, Kind: EvSession, Session: EventBegin}, true
	}
	if s.cursor == s.cfg.Tick {
		s.cursor = 2 * s.cfg.Tick
		return Event{At: s.cfg.Tick, Kind: EvAttach}, true
	}
	if s.cursor == 2*s.cfg.Tick {
		s.cursor = 3 * s.cfg.Tick
		return Event{At: 2 * s.cfg.Tick, Kind: EvSession, Session: EventFocus}, true
	}

	// Earliest of: next due source frame, next scheduled fault, end.
	var nextFrame time.Duration = -1
	var frameKey string
	for _, d := range s.sources() {
		n := s.next[d.key]
		if n <= s.cfg.Duration && (nextFrame < 0 || n < nextFrame) {
			nextFrame = n
			frameKey = d.key
		}
	}

	var nextFault time.Duration = -1
	if s.cfg.InjectFaults {
		for int(s.faultTick) < len(faultSchedule) {
			ft := faultSchedule[s.faultTick]
			if ft >= 3*s.cfg.Tick {
				nextFault = ft
				break
			}
			s.faultTick++
		}
	}

	end := s.cfg.Duration
	switch {
	case nextFault >= 0 && (nextFrame < 0 || nextFault <= nextFrame) && nextFault <= end:
		s.faultTick++
		return s.faultEvent(nextFault), true
	case nextFrame >= 0 && nextFrame <= end:
		d := s.defByKey(frameKey)
		period := time.Duration(float64(time.Second) / float64(d.hz))
		s.next[frameKey] = nextFrame + period
		return s.frameEvent(d, nextFrame, trackingLostAt(d, nextFrame)), true
	default:
		if !s.done {
			s.done = true
			return Event{At: end, Kind: EvSession, Session: EventEnd}, true
		}
		return Event{}, false
	}
}

func (s *Simulator) defByKey(key string) sourceDef {
	for _, d := range s.sources() {
		if d.key == key {
			return d
		}
	}
	return sourceDef{}
}

func (s *Simulator) frameEvent(d sourceDef, t time.Duration, lost bool) Event {
	s.seq[d.key]++
	in := FrameInput{
		DeviceKey: d.key,
		Source:    d.kind,
		Hand:      d.hand,
		Timestamp: t,
		Pose:      s.trajectory(d, t),
		State:     TrackValid,
		Sequence:  s.seq[d.key],
	}
	if lost {
		in.Lost = true
		in.State = TrackInvalid
	} else {
		// Seed-driven sub-millimeter sensor noise. It is part of the event
		// stream, so identical seeds reproduce the identical noise sequence.
		n := 0.0005
		in.Pose.Position.X += (s.rng.Float64() - 0.5) * 2 * n
		in.Pose.Position.Y += (s.rng.Float64() - 0.5) * 2 * n
		in.Pose.Position.Z += (s.rng.Float64() - 0.5) * 2 * n
	}
	return Event{At: t, Kind: EvFrame, Input: in}
}

// faultEvent builds the deterministic anomaly scheduled at time t.
func (s *Simulator) faultEvent(t time.Duration) Event {
	switch {
	case t == 500*time.Millisecond:
		d := s.defByKey("hmd")
		// duplicate: resend a frame with an already-used timestamp/pose
		period := time.Duration(float64(time.Second) / float64(d.hz))
		dupAt := period * (t / period)
		return Event{At: t, Kind: EvFrame, Input: FrameInput{
			DeviceKey: d.key, Source: d.kind, Hand: d.hand,
			Timestamp: dupAt, Pose: s.trajectory(d, dupAt), State: TrackValid,
		}}
	case t == 600*time.Millisecond:
		d := s.defByKey("ctl-l")
		lateAt := t - 80*time.Millisecond // late but within the 500ms window
		return Event{At: t, Kind: EvFrame, Input: FrameInput{
			DeviceKey: d.key, Source: d.kind, Hand: d.hand,
			Timestamp: lateAt, Pose: s.trajectory(d, lateAt), State: TrackValid,
		}}
	case t == 700*time.Millisecond:
		d := s.defByKey("ctl-r")
		oldAt := t - 40*time.Millisecond // out-of-order
		return Event{At: t, Kind: EvFrame, Input: FrameInput{
			DeviceKey: d.key, Source: d.kind, Hand: d.hand,
			Timestamp: oldAt, Pose: s.trajectory(d, oldAt), State: TrackValid,
		}}
	case t == 1200*time.Millisecond:
		return Event{At: t, Kind: EvDetach, DeviceKey: "ctl-r"}
	case t == 1500*time.Millisecond:
		return Event{At: t, Kind: EvAttach, DeviceKey: "ctl-r"}
	case t == 1600*time.Millisecond:
		return Event{At: t, Kind: EvStage, Origin: Pose{
			Position:    Vec3{X: 0.1},
			Orientation: identityQuat(),
		}}
	case t == 1700*time.Millisecond:
		return Event{At: t, Kind: EvFloor, Y: 0.0}
	case t == 2400*time.Millisecond:
		return Event{At: t, Kind: EvSession, Session: EventPause}
	case t == 2600*time.Millisecond:
		return Event{At: t, Kind: EvSession, Session: EventResume}
	}
	return Event{At: t}
}

// Run applies every remaining event to the given engine in deterministic
// order and records every published snapshot.
func (s *Simulator) Run(e *Engine, r *Recorder) (Manifest, error) {
	var lastTick time.Duration
	for {
		ev, ok := s.Next()
		if !ok {
			break
		}
		switch ev.Kind {
		case EvSession:
			if err := e.Session().Apply(ev.Session, ev.At); err != nil {
				// deterministic script: ignore expected duplicate-safe errors
				_ = err
			}
		case EvAttach:
			if ev.DeviceKey == "" {
				for _, d := range s.sources() {
					id := e.RegisterInputDevice(d.key, d.kind, d.hand, d.source)
					e.SetBinding(id, "bind", d.key+"-binding")
				}
			} else {
				d := s.defByKey(ev.DeviceKey)
				id := e.RegisterInputDevice(d.key, d.kind, d.hand, d.source)
				e.Attach(id)
			}
		case EvDetach:
			if d, ok := e.devices.lookupKey(ev.DeviceKey); ok {
				e.Detach(d.ID)
			}
		case EvFrame:
			e.Submit(ev.Input)
		case EvFloor:
			e.SetFloorHeight(ev.Y)
		case EvStage:
			e.AdjustStageOrigin(ev.Origin)
		}

		// Publish at tick boundaries: advance the engine to the largest tick
		// not exceeding this event's time.
		t := ev.At
		for lastTick < t {
			lastTick += s.cfg.Tick
			if snap, ok := e.Advance(lastTick); ok {
				r.Record(snap)
			}
		}
	}
	return r.Manifest(e.Config()), nil
}
