package xr

import (
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"strings"
	"time"
)

// Recorder accumulates a deterministic fingerprint of every published
// snapshot. Fingerprints never rely on map iteration (snapshots are already
// ordered by canonical registration order; bindings are sorted here), so two
// runs with identical inputs/seed/config produce identical hashes.
type Recorder struct {
	hashes []string
}

func NewRecorder() *Recorder { return &Recorder{} }

// Record fingerprints and stores one snapshot, returning its hash.
func (r *Recorder) Record(s Snapshot) string {
	h := hashBytes([]byte(snapshotCanonical(s)))
	r.hashes = append(r.hashes, h)
	return h
}

func (r *Recorder) Hashes() []string {
	out := make([]string, len(r.hashes))
	copy(out, r.hashes)
	return out
}

func snapshotCanonical(s Snapshot) string {
	var b strings.Builder
	fmt.Fprintf(&b, "snap:%d|t=%d|space=%d|sess=%d|n=%d", s.Index,
		int64(s.Time), s.Space, s.SessionState, len(s.Devices))
	for _, ds := range s.Devices {
		fmt.Fprintf(&b, ";dev=%s,kind=%d,hand=%d,online=%t,present=%t",
			ds.Device.ID, ds.Device.Kind, ds.Device.Hand, ds.Device.Online, ds.Present)
		f := ds.Frame
		fmt.Fprintf(&b, ",f{t=%d,st=%d,p=[%g:%g:%g],q=[%g:%g:%g:%g],seq=%d,hv=%t",
			int64(f.Timestamp), f.State,
			f.Pose.Position.X, f.Pose.Position.Y, f.Pose.Position.Z,
			f.Pose.Orientation.X, f.Pose.Orientation.Y,
			f.Pose.Orientation.Z, f.Pose.Orientation.W,
			f.Sequence, f.HasVelocity)
		// bindings in deterministic key order
		if len(ds.Device.Bindings) > 0 {
			keys := make([]string, 0, len(ds.Device.Bindings))
			for k := range ds.Device.Bindings {
				keys = append(keys, k)
			}
			sortStrings(keys)
			for _, k := range keys {
				fmt.Fprintf(&b, ",b=%s=%s", k, ds.Device.Bindings[k])
			}
		}
		b.WriteString("}")
	}
	return b.String()
}

func sortStrings(a []string) {
	// local insertion sort keeps imports small; binding sets are tiny.
	for i := 1; i < len(a); i++ {
		for j := i; j > 0 && a[j-1] > a[j]; j-- {
			a[j-1], a[j] = a[j], a[j-1]
		}
	}
}

// Manifest describes the exact algorithm/configuration a recording was made
// with.
type Manifest struct {
	Seed             int64
	AlgorithmVersion string
	Space            ReferenceSpace
	PredictionOn     bool
	PredictAhead     time.Duration
	InterpolationOn  bool
	Snapshots        int
	Hash             string
}

// Manifest builds a reproducibility manifest for the recording.
func (r *Recorder) Manifest(cfg Config) Manifest {
	h := sha256.New()
	for _, x := range r.hashes {
		h.Write([]byte(x))
	}
	return Manifest{
		Seed:             cfg.Seed,
		AlgorithmVersion: cfg.AlgorithmVersion,
		Space:            cfg.Space,
		PredictionOn:     cfg.PredictionEnabled,
		PredictAhead:     cfg.PredictAhead,
		InterpolationOn:  cfg.InterpolationEnabled,
		Snapshots:        len(r.hashes),
		Hash:             hex.EncodeToString(h.Sum(nil)),
	}
}

// ReproDecision is the result of comparing two manifests.
type ReproDecision struct {
	Reproducible bool
	Reason       string
}

// CompareManifolds decides whether two recordings are expected to reproduce
// bit-identical output. An algorithm/version or configuration mismatch is
// reported as an explicit non-reproducible result with a distinguishable
// reason rather than failing silently.
func CompareManifolds(want, got Manifest) ReproDecision {
	if want.AlgorithmVersion != got.AlgorithmVersion {
		return ReproDecision{Reproducible: false,
			Reason: "algorithm_version_mismatch:" + want.AlgorithmVersion + "!=" + got.AlgorithmVersion}
	}
	if want.Seed != got.Seed {
		return ReproDecision{Reproducible: false, Reason: "seed_mismatch"}
	}
	if want.Space != got.Space {
		return ReproDecision{Reproducible: false, Reason: "space_mismatch"}
	}
	if want.PredictionOn != got.PredictionOn ||
		want.InterpolationOn != got.InterpolationOn ||
		want.PredictAhead != got.PredictAhead {
		return ReproDecision{Reproducible: false, Reason: "config_mismatch"}
	}
	if want.Snapshots != got.Snapshots {
		return ReproDecision{Reproducible: false, Reason: "snapshot_count_mismatch"}
	}
	if want.Hash != got.Hash {
		return ReproDecision{Reproducible: false, Reason: "output_hash_mismatch"}
	}
	return ReproDecision{Reproducible: true, Reason: "identical"}
}

func hashBytes(b []byte) string {
	h := sha256.Sum256(b)
	return hex.EncodeToString(h[:])
}
