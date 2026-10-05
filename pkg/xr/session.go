package xr

import (
	"errors"
	"fmt"
	"sync"
	"time"
)

// SessionState is a session lifecycle state.
type SessionState uint8

const (
	// StateIdle: session created but not started.
	StateIdle SessionState = iota
	// StateRunning: started, rendering allowed but without input focus.
	StateRunning
	// StateFocused: running and visible/focused, poses are presented.
	StateFocused
	// StatePaused: suspended; updates are retained but not presented.
	StatePaused
	// StateEnded: terminal; no further transitions are legal.
	StateEnded
)

func (s SessionState) String() string {
	switch s {
	case StateRunning:
		return "running"
	case StateFocused:
		return "focused"
	case StatePaused:
		return "paused"
	case StateEnded:
		return "ended"
	default:
		return "idle"
	}
}

// Distinguishable transition failure reasons.
var (
	// ErrIllegalTransition marks an event that has no legal edge from the
	// current state.
	ErrIllegalTransition = errors.New("illegal session transition")
	// ErrAlreadyInState marks a repeated request that would only re-enter the
	// state the session is already in; it changes nothing.
	ErrAlreadyInState = errors.New("session already in target state")
	// ErrSessionEnded marks any event delivered after termination.
	ErrSessionEnded = errors.New("session already ended")
)

// Session event names.
const (
	EventBegin  = "begin"
	EventFocus  = "focus"
	EventPause  = "pause"
	EventResume = "resume"
	EventEnd    = "end"
)

// TransitionError describes a rejected session transition with a
// distinguishable machine-readable reason.
type TransitionError struct {
	From   SessionState
	Event  string
	Reason string
	err    error
}

func (e *TransitionError) Error() string {
	return fmt.Sprintf("xr: transition %q from %s rejected: %s", e.Event, e.From, e.Reason)
}

func (e *TransitionError) Unwrap() error { return e.err }

// TransitionRecord is one entry of the session transition history. Applied
// is false for rejected calls; rejected entries never mutated the state.
type TransitionRecord struct {
	From    SessionState
	To      SessionState
	Event   string
	At      time.Duration
	Applied bool
	Reason  string
}

// Session is the deterministic lifecycle state machine. All methods are safe
// for concurrent use: the mutex serializes transition calls so concurrent or
// repeated requests can produce at most one actual state change.
type Session struct {
	mu      sync.Mutex
	state   SessionState
	history []TransitionRecord
}

func NewSession() *Session {
	return &Session{state: StateIdle}
}

func (s *Session) State() SessionState {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.state
}

// History returns a copy of the transition history.
func (s *Session) History() []TransitionRecord {
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make([]TransitionRecord, len(s.history))
	copy(out, s.history)
	return out
}

func (s *Session) Begin(at time.Duration) error  { return s.Apply(EventBegin, at) }
func (s *Session) Focus(at time.Duration) error  { return s.Apply(EventFocus, at) }
func (s *Session) Pause(at time.Duration) error  { return s.Apply(EventPause, at) }
func (s *Session) Resume(at time.Duration) error { return s.Apply(EventResume, at) }
func (s *Session) End(at time.Duration) error    { return s.Apply(EventEnd, at) }

// Apply dispatches a named lifecycle event. Legal edges:
//
//	idle    --begin-->  running
//	running --focus-->  focused
//	focused --pause-->  paused
//	running --pause-->  paused
//	paused  --resume--> running
//	active  --end---->  ended
//
// Every other event is rejected (see ErrIllegalTransition,
// ErrAlreadyInState and ErrSessionEnded). Repeated and concurrent calls have
// no state-changing side effect.
func (s *Session) Apply(event string, at time.Duration) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	from := s.state
	reject := func(reason string, err error) error {
		terr := &TransitionError{From: from, Event: event, Reason: reason, err: err}
		s.history = append(s.history, TransitionRecord{
			From: from, To: from, Event: event, At: at, Applied: false, Reason: reason,
		})
		return terr
	}

	if from == StateEnded {
		if event == EventEnd {
			return reject("already_ended", ErrAlreadyInState)
		}
		return reject("session_ended", ErrSessionEnded)
	}

	var to SessionState
	reason := ""
	switch event {
	case EventBegin:
		switch from {
		case StateIdle:
			to = StateRunning
		case StateRunning, StateFocused:
			reason = "already_started"
		case StatePaused:
			reason = "cannot_begin_paused_session"
		}
	case EventFocus:
		switch from {
		case StateRunning:
			to = StateFocused
		case StateFocused:
			reason = "already_focused"
		case StateIdle:
			reason = "not_started"
		case StatePaused:
			reason = "focus_while_paused"
		}
	case EventPause:
		switch from {
		case StateRunning, StateFocused:
			to = StatePaused
		case StatePaused:
			reason = "already_paused"
		case StateIdle:
			reason = "not_started"
		}
	case EventResume:
		switch from {
		case StatePaused:
			to = StateRunning
		case StateRunning, StateFocused:
			reason = "not_paused"
		case StateIdle:
			reason = "not_started"
		}
	case EventEnd:
		to = StateEnded
	default:
		reason = "unknown_event"
	}

	if reason != "" {
		// "already_*" cases are duplicate requests; everything else is an
		// illegal edge.
		err := ErrIllegalTransition
		if reason == "already_started" || reason == "already_focused" ||
			reason == "already_paused" || reason == "not_paused" {
			err = ErrAlreadyInState
		}
		return reject(reason, err)
	}

	s.state = to
	s.history = append(s.history, TransitionRecord{
		From: from, To: to, Event: event, At: at, Applied: true,
	})
	return nil
}
