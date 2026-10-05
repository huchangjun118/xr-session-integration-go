package xr

import (
	"fmt"
	"sync"
)

// SessionState 是会话生命周期状态。
type SessionState int

const (
	SessionStateIdle SessionState = iota
	SessionStateRunning
	SessionStateFocused
	SessionStatePaused
	SessionStateEnded
)

func (s SessionState) String() string {
	switch s {
	case SessionStateIdle:
		return "idle"
	case SessionStateRunning:
		return "running"
	case SessionStateFocused:
		return "focused"
	case SessionStatePaused:
		return "paused"
	case SessionStateEnded:
		return "ended"
	default:
		return "unknown"
	}
}

// TransitionReason 区分非法转移的原因。
type TransitionReason int

const (
	TransitionReasonNone TransitionReason = iota
	// TransitionReasonIllegalTransition 表示状态机不允许该转移。
	TransitionReasonIllegalTransition
	// TransitionReasonSessionEnded 表示会话已结束，拒绝一切转移。
	TransitionReasonSessionEnded
)

func (r TransitionReason) String() string {
	switch r {
	case TransitionReasonNone:
		return "none"
	case TransitionReasonIllegalTransition:
		return "illegal-transition"
	case TransitionReasonSessionEnded:
		return "session-ended"
	default:
		return "unknown"
	}
}

// TransitionError 表示一次被拒绝的状态转移。
type TransitionError struct {
	From   SessionState
	To     SessionState
	Reason TransitionReason
}

func (e *TransitionError) Error() string {
	return fmt.Sprintf("xr: session transition %s -> %s rejected: %s", e.From, e.To, e.Reason)
}

// allowed 是状态转移表。
var allowed = map[SessionState]map[SessionState]bool{
	SessionStateIdle:    {SessionStateRunning: true, SessionStateEnded: true},
	SessionStateRunning: {SessionStateFocused: true, SessionStatePaused: true, SessionStateEnded: true},
	SessionStateFocused: {SessionStateRunning: true, SessionStatePaused: true, SessionStateEnded: true},
	SessionStatePaused:  {SessionStateRunning: true, SessionStateEnded: true},
	SessionStateEnded:   {},
}

// Session 管理会话生命周期，转移确定且并发安全。
type Session struct {
	mu    sync.Mutex
	state SessionState
}

func NewSession() *Session { return &Session{state: SessionStateIdle} }

func (s *Session) State() SessionState {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.state
}

// Transition 尝试把会话迁移到目标状态。
// 迁移到当前状态是幂等空操作，返回 nil；
// 非法迁移返回 *TransitionError，且不产生副作用；
// 并发调用下同一转移只生效一次。
func (s *Session) Transition(to SessionState) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.state == to {
		return nil
	}
	if s.state == SessionStateEnded {
		return &TransitionError{From: s.state, To: to, Reason: TransitionReasonSessionEnded}
	}
	if !allowed[s.state][to] {
		return &TransitionError{From: s.state, To: to, Reason: TransitionReasonIllegalTransition}
	}
	s.state = to
	return nil
}
