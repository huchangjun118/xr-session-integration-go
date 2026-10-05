package xr

import (
	"sort"
	"time"
)

// AcceptReason explains what the buffer did with an offered frame.
type AcceptReason uint8

const (
	AcceptOK        AcceptReason = iota
	RejectDuplicate              // timestamp already present
	RejectTooLate                // older than the late-window
)

// poseBuffer is a per-device history of pose samples kept ordered by
// timestamp. It enforces the ordering guarantees of the integration layer:
//
//   - duplicate frames (same timestamp as an existing frame) are rejected;
//   - frames older than (newestTimestamp - maxLate) are rejected as too late;
//   - frames that are merely late but inside the window are inserted into the
//     correct historical slot;
//   - the newest pose can therefore never be overwritten by an older frame.
type poseBuffer struct {
	id      DeviceID
	frames  []PoseFrame
	maxSize int
	maxLate time.Duration
}

func newPoseBuffer(id DeviceID, maxSize int, maxLate time.Duration) *poseBuffer {
	if maxSize < 2 {
		maxSize = 2
	}
	return &poseBuffer{id: id, maxSize: maxSize, maxLate: maxLate}
}

// push inserts a frame into the ordered history.
func (b *poseBuffer) push(f PoseFrame) (accepted bool, reason AcceptReason) {
	if len(b.frames) > 0 {
		head := b.frames[len(b.frames)-1]
		if f.Timestamp == head.Timestamp {
			return false, RejectDuplicate
		}
		if f.Timestamp < head.Timestamp {
			if b.maxLate > 0 && head.Timestamp-f.Timestamp > b.maxLate {
				return false, RejectTooLate
			}
			// late but within window: reject if the timestamp is already held
			i := sort.Search(len(b.frames), func(i int) bool {
				return b.frames[i].Timestamp >= f.Timestamp
			})
			if i < len(b.frames) && b.frames[i].Timestamp == f.Timestamp {
				return false, RejectDuplicate
			}
			b.frames = append(b.frames, PoseFrame{})
			copy(b.frames[i+1:], b.frames[i:])
			b.frames[i] = f
		} else {
			b.frames = append(b.frames, f)
		}
	} else {
		b.frames = append(b.frames, f)
	}
	b.trim()
	return true, AcceptOK
}

// trim keeps at most maxSize most-recent frames.
func (b *poseBuffer) trim() {
	if len(b.frames) > b.maxSize {
		b.frames = b.frames[len(b.frames)-b.maxSize:]
	}
}

func (b *poseBuffer) newest() (PoseFrame, bool) {
	if len(b.frames) == 0 {
		return PoseFrame{}, false
	}
	return b.frames[len(b.frames)-1], true
}

// at returns the newest frame with timestamp <= t (the frame the device was
// actually in at t; it never returns a future frame).
func (b *poseBuffer) at(t time.Duration) (PoseFrame, bool) {
	i := sort.Search(len(b.frames), func(i int) bool {
		return b.frames[i].Timestamp > t
	})
	if i == 0 {
		return PoseFrame{}, false
	}
	return b.frames[i-1], true
}

// bracketing returns the samples immediately before/after t. If t is beyond
// the newest sample, after is the zero frame and ok is true with afterOk set
// false (caller extrapolates).
func (b *poseBuffer) bracketing(t time.Duration) (before, after PoseFrame, ok, afterOk bool) {
	i := sort.Search(len(b.frames), func(i int) bool {
		return b.frames[i].Timestamp >= t
	})
	if i == 0 {
		if len(b.frames) > 0 && b.frames[0].Timestamp == t {
			return b.frames[0], PoseFrame{}, true, false
		}
		return PoseFrame{}, PoseFrame{}, false, false
	}
	before = b.frames[i-1]
	if i < len(b.frames) {
		after = b.frames[i]
		return before, after, true, true
	}
	return before, PoseFrame{}, true, false
}

func (b *poseBuffer) len() int { return len(b.frames) }
