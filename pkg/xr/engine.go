package xr

import (
	"sync"
	"time"
)

// Engine is the unified session and tracking integration layer. It accepts
// normalized pose inputs from arbitrary device sources, hot-plug events and
// session transitions, and serves self-consistent snapshots to slow readers
// without blocking updates.
//
// Concurrency model: one RWMutex guards all mutable state. Advance builds a
// complete snapshot (every device evaluated at the same output time) under
// the write lock, then publishes it as an immutable value; readers receive a
// defensive copy and may process it at their own pace without holding any
// lock, so a slow consumer never blocks the update path.
type Engine struct {
	mu        sync.RWMutex
	cfg       Config
	session   *Session
	space     *SpaceManager
	devices   *registry
	buffers   map[DeviceID]*poseBuffer
	pred      *predictor
	predState map[DeviceID]*predState
	seqs      map[DeviceID]uint64
	now       time.Duration
	snapIndex uint64
	cached    Snapshot
	published bool
}

// SubmitStats reports what happened to an offered input frame.
type SubmitStats struct {
	Accepted bool
	Reason   AcceptReason
	DeviceID DeviceID
	Online   bool
}

func NewEngine(cfg Config) *Engine {
	if cfg.BufferLength == 0 {
		cfg = DefaultConfig()
	}
	return &Engine{
		cfg:       cfg,
		session:   NewSession(),
		space:     NewSpaceManager(cfg.Space),
		devices:   newRegistry(),
		buffers:   make(map[DeviceID]*poseBuffer),
		pred:      newPredictor(cfg),
		predState: make(map[DeviceID]*predState),
		seqs:      make(map[DeviceID]uint64),
	}
}

// Lifecycle -----------------------------------------------------------------

func (e *Engine) Begin() error  { return e.session.Begin(e.now) }
func (e *Engine) Focus() error  { return e.session.Focus(e.now) }
func (e *Engine) Pause() error  { return e.session.Pause(e.now) }
func (e *Engine) Resume() error { return e.session.Resume(e.now) }
func (e *Engine) End() error    { return e.session.End(e.now) }

// ApplySession applies a named session event at the current engine time.
func (e *Engine) ApplySession(event string) error {
	e.mu.RLock()
	at := e.now
	e.mu.RUnlock()
	return e.session.Apply(event, at)
}

func (e *Engine) SessionState() SessionState { return e.session.State() }

func (e *Engine) Session() *Session { return e.session }

// Devices / hot-plug --------------------------------------------------------

// RegisterInputDevice declares a source device and returns its stable ID.
// Registering marks the device online; repeated registration of the same
// logical device is idempotent and preserves identity and bindings.
func (e *Engine) RegisterInputDevice(key string, kind Kind, hand Handedness, source string) DeviceID {
	e.mu.Lock()
	defer e.mu.Unlock()
	d := e.devices.registerID(key, kind, hand, source)
	e.ensureBufferLocked(d.ID)
	return d.ID
}

func (e *Engine) ensureBufferLocked(id DeviceID) *poseBuffer {
	b, ok := e.buffers[id]
	if !ok {
		b = newPoseBuffer(id, e.cfg.BufferLength, e.cfg.ResendMaxLate)
		e.buffers[id] = b
	}
	if _, ok := e.predState[id]; !ok {
		e.predState[id] = &predState{}
	}
	return b
}

func (e *Engine) Attach(id DeviceID) {
	e.mu.Lock()
	defer e.mu.Unlock()
	if _, ok := e.devices.device(id); ok {
		e.devices.attach(id)
		e.ensureBufferLocked(id)
	}
}

// Detach marks a device absent. Its buffer and bindings are retained; data
// arriving via its source key while detached is rejected by Submit.
func (e *Engine) Detach(id DeviceID) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.devices.detach(id)
}

func (e *Engine) SetBinding(id DeviceID, key, value string) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.devices.setBinding(id, key, value)
}

func (e *Engine) Devices() []Device {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.devices.list()
}

// Data path -----------------------------------------------------------------

// Submit pushes one raw input frame into the engine. It returns whether the
// frame was accepted. Frames are dropped when the device is unknown,
// currently detached, duplicated, or too late; they never land on another
// device.
func (e *Engine) Submit(in FrameInput) SubmitStats {
	e.mu.Lock()
	defer e.mu.Unlock()

	d, ok := e.devices.lookupKey(in.DeviceKey)
	if !ok {
		// Auto-registering keeps the "just feed a stream" path simple while
		// still normalizing through stable canonical identities.
		d = e.devices.registerID(in.DeviceKey, in.Source, in.Hand, in.DeviceKey)
	}
	if !d.Online {
		return SubmitStats{Accepted: false, DeviceID: d.ID, Online: false, Reason: RejectTooLate}
	}

	b := e.ensureBufferLocked(d.ID)

	state := in.State
	if in.Lost {
		state = TrackInvalid
	}
	frame := PoseFrame{
		DeviceID:    d.ID,
		Timestamp:   in.Timestamp,
		Pose:        in.Pose,
		State:       state,
		HasVelocity: in.HasVelocity && state != TrackInvalid,
	}
	if in.HasVelocity {
		frame.Velocity = in.Velocity
	}
	if frame.State == TrackInvalid {
		// A loss sample must never look like a zero pose: carry the last
		// known good pose but keep the distinguishable invalid state.
		if g, gok := lastGoodAt(b, in.Timestamp); gok {
			frame.Pose = g.Pose
		} else if f, fok := b.newest(); fok && f.State != TrackInvalid {
			frame.Pose = f.Pose
		}
		frame.HasVelocity = false
	}

	accepted, reason := b.push(frame)
	if !accepted {
		return SubmitStats{Accepted: false, DeviceID: d.ID, Online: true, Reason: reason}
	}
	e.seqs[d.ID]++
	return SubmitStats{Accepted: true, DeviceID: d.ID, Online: true}
}

// Advance moves the engine clock to now and publishes a new consistent
// snapshot. The clock is monotonic: a time earlier than the current clock is
// rejected (ok=false) and the previously published snapshot is returned
// unchanged, so output can never move backwards. While the session is idle
// or paused no new snapshot is published.
func (e *Engine) Advance(now time.Duration) (Snapshot, bool) {
	e.mu.Lock()
	defer e.mu.Unlock()

	if e.published && now < e.now {
		return copySnapshot(e.cached), false
	}
	e.now = now

	st := e.session.State()
	if st == StateIdle || st == StatePaused {
		// Retain incoming data but do not present while idle/paused.
		if e.published {
			return copySnapshot(e.cached), false
		}
		return Snapshot{}, false
	}

	e.snapIndex++
	target := now
	if e.cfg.PredictionEnabled {
		target = now + e.cfg.PredictAhead
	}

	list := e.devices.list()
	snap := Snapshot{
		// When prediction is disabled each frame is reported at its own raw
		// timestamp; the newest received sample time defines the snapshot
		// time so the output stamp matches the unprocessed output.
		Time:         target,
		Space:        e.cfg.Space,
		SessionState: st,
		Index:        e.snapIndex,
		Devices:      make([]DeviceSnapshot, 0, len(list)),
	}
	var newestRaw time.Duration
	for _, d := range list {
		b := e.ensureBufferLocked(d.ID)
		res := e.pred.evaluate(b, target, e.predState[d.ID])
		if f, ok := b.newest(); ok && f.Timestamp > newestRaw {
			newestRaw = f.Timestamp
		}
		// Sequence number is assigned from accepted input order.
		if seq, ok := e.seqs[d.ID]; ok && res.frame.Sequence == 0 {
			res.frame.Sequence = seq
		}
		out := e.space.Frame(res.frame)
		snap.Devices = append(snap.Devices, DeviceSnapshot{
			Device:  d,
			Frame:   out,
			Present: b.len() > 0,
		})
	}
	if !e.cfg.PredictionEnabled {
		snap.Time = newestRaw
	}

	e.cached = snap
	e.published = true
	return copySnapshot(snap), true
}

// Snapshot returns the most recently published consistent snapshot as a copy
// the caller may retain indefinitely.
func (e *Engine) Snapshot() Snapshot {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return copySnapshot(e.cached)
}

// copySnapshot deep-copies slices/maps so callers cannot alias engine state.
func copySnapshot(s Snapshot) Snapshot {
	out := s
	if s.Devices != nil {
		out.Devices = make([]DeviceSnapshot, len(s.Devices))
		for i, ds := range s.Devices {
			cp := ds
			if ds.Device.Bindings != nil {
				cp.Device.Bindings = make(map[string]string, len(ds.Device.Bindings))
				for k, v := range ds.Device.Bindings {
					cp.Device.Bindings[k] = v
				}
			}
			out.Devices[i] = cp
		}
	}
	return out
}

// Reference space -----------------------------------------------------------

func (e *Engine) SetFloorHeight(y float64) {
	e.mu.Lock()
	e.space.SetFloorHeight(y)
	e.mu.Unlock()
}

func (e *Engine) AdjustStageOrigin(origin Pose) {
	e.mu.Lock()
	e.space.AdjustStageOrigin(origin)
	e.mu.Unlock()
}

// Space exposes the reference-space manager for anchor operations.
func (e *Engine) SpaceManager() *SpaceManager { return e.space }

// Config returns the engine configuration.
func (e *Engine) Config() Config { return e.cfg }
