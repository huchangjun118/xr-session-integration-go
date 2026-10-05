package xr

import (
	"errors"
	"sync"
	"testing"
	"time"
)

func TestSessionHappyPath(t *testing.T) {
	s := NewSession()
	steps := []struct {
		ev   string
		want SessionState
	}{
		{EventBegin, StateRunning},
		{EventFocus, StateFocused},
		{EventPause, StatePaused},
		{EventResume, StateRunning},
		{EventFocus, StateFocused},
		{EventEnd, StateEnded},
	}
	for i, st := range steps {
		if err := s.Apply(st.ev, time.Duration(i)*time.Millisecond); err != nil {
			t.Fatalf("step %d %s: %v", i, st.ev, err)
		}
		if got := s.State(); got != st.want {
			t.Fatalf("step %d after %s: got %s want %s", i, st.ev, got, st.want)
		}
	}
}

func TestSessionIllegalTransitions(t *testing.T) {
	cases := []struct {
		name   string
		seed   func(*Session)
		ev     string
		want   error
		reason string
	}{
		{"focus before begin", func(s *Session) {}, EventFocus, ErrIllegalTransition, "not_started"},
		{"pause before begin", func(s *Session) {}, EventPause, ErrIllegalTransition, "not_started"},
		{"resume never paused", func(s *Session) { _ = s.Begin(0) }, EventResume, ErrAlreadyInState, "not_paused"},
		{"begin twice", func(s *Session) { _ = s.Begin(0) }, EventBegin, ErrAlreadyInState, "already_started"},
		{"focus twice", func(s *Session) { _ = s.Begin(0); _ = s.Focus(0) }, EventFocus, ErrAlreadyInState, "already_focused"},
		{"resume from focused", func(s *Session) { _ = s.Begin(0); _ = s.Focus(0) }, EventResume, ErrAlreadyInState, "not_paused"},
		{"unknown event", func(s *Session) { _ = s.Begin(0) }, "bogus", ErrIllegalTransition, "unknown_event"},
		{"anything after end", func(s *Session) { _ = s.Begin(0); _ = s.End(0) }, EventFocus, ErrSessionEnded, "session_ended"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := NewSession()
			c.seed(s)
			err := s.Apply(c.ev, 0)
			if err == nil {
				t.Fatal("want error, got nil")
			}
			if !errors.Is(err, c.want) {
				t.Fatalf("want wrap %v, got %v", c.want, err)
			}
			var te *TransitionError
			if !errors.As(err, &te) {
				t.Fatal("want *TransitionError")
			}
			if te.Reason != c.reason {
				t.Fatalf("reason = %q, want %q", te.Reason, c.reason)
			}
		})
	}
}

func TestSessionEndIdempotent(t *testing.T) {
	s := NewSession()
	_ = s.Begin(0)
	if err := s.End(0); err != nil {
		t.Fatal(err)
	}
	before := s.State()
	err := s.End(time.Millisecond)
	if !errors.Is(err, ErrAlreadyInState) {
		t.Fatalf("second end: want ErrAlreadyInState, got %v", err)
	}
	if s.State() != before {
		t.Fatal("repeated end changed state")
	}
}

func TestSessionConcurrentTransitions(t *testing.T) {
	// Many concurrent Begin calls: exactly one is applied, state unchanged
	// afterwards and history records the rest as rejected.
	s := NewSession()
	var wg sync.WaitGroup
	for i := 0; i < 64; i++ {
		wg.Add(1)
		go func() { defer wg.Done(); _ = s.Begin(0) }()
	}
	wg.Wait()
	if s.State() != StateRunning {
		t.Fatalf("state = %s, want running", s.State())
	}
	var applied int
	for _, r := range s.History() {
		if r.Applied {
			applied++
		}
	}
	if applied != 1 {
		t.Fatalf("applied begins = %d, want exactly 1", applied)
	}
}
