package xr

import (
	"errors"
	"sync"
	"testing"
)

func TestSessionHappyPath(t *testing.T) {
	s := NewSession()
	if s.State() != SessionStateIdle {
		t.Fatalf("initial state = %s", s.State())
	}
	steps := []SessionState{SessionStateRunning, SessionStateFocused, SessionStatePaused, SessionStateRunning, SessionStateEnded}
	for _, st := range steps {
		if err := s.Transition(st); err != nil {
			t.Fatalf("transition to %s rejected: %v", st, err)
		}
		if s.State() != st {
			t.Fatalf("state = %s, want %s", s.State(), st)
		}
	}
}

func TestSessionIllegalTransitionRejected(t *testing.T) {
	s := NewSession()
	err := s.Transition(SessionStateFocused)
	var te *TransitionError
	if !errors.As(err, &te) {
		t.Fatalf("expected TransitionError, got %v", err)
	}
	if te.Reason != TransitionReasonIllegalTransition {
		t.Fatalf("reason = %s", te.Reason)
	}
	if s.State() != SessionStateIdle {
		t.Fatalf("illegal transition had side effect: %s", s.State())
	}
	// 暂停不能直接到获得焦点
	_ = s.Transition(SessionStateRunning)
	_ = s.Transition(SessionStatePaused)
	if err := s.Transition(SessionStateFocused); !errors.As(err, &te) || te.Reason != TransitionReasonIllegalTransition {
		t.Fatalf("paused->focused should be illegal: %v", err)
	}
}

func TestSessionEndedRejectsEverything(t *testing.T) {
	s := NewSession()
	_ = s.Transition(SessionStateRunning)
	_ = s.Transition(SessionStateEnded)
	for _, st := range []SessionState{SessionStateRunning, SessionStateFocused, SessionStatePaused, SessionStateIdle} {
		err := s.Transition(st)
		var te *TransitionError
		if !errors.As(err, &te) || te.Reason != TransitionReasonSessionEnded {
			t.Fatalf("ended session transition to %s: got %v", st, err)
		}
	}
}

func TestSessionIdempotentAndConcurrent(t *testing.T) {
	s := NewSession()
	_ = s.Transition(SessionStateRunning)
	// 重复迁移到当前状态是幂等空操作
	if err := s.Transition(SessionStateRunning); err != nil {
		t.Fatalf("same-state transition should be no-op: %v", err)
	}
	// 并发同一转移只生效一次，且无副作用
	var wg sync.WaitGroup
	errs := make(chan error, 64)
	for i := 0; i < 64; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			errs <- s.Transition(SessionStateFocused)
		}()
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		if err != nil {
			t.Fatalf("concurrent transition error: %v", err)
		}
	}
	if s.State() != SessionStateFocused {
		t.Fatalf("state = %s", s.State())
	}
}
