package xr

import "time"

// Config configures the integration engine.
type Config struct {
	// Space is the reference space used for output snapshots.
	Space ReferenceSpace
	// PredictionEnabled controls pose extrapolation. When disabled the output
	// degrades to the latest raw sample without interpolation/prediction.
	PredictionEnabled bool
	// PredictAhead is the prediction horizon when prediction is enabled.
	PredictAhead time.Duration
	// InterpolationEnabled smooths output between adjacent samples.
	InterpolationEnabled bool
	// ResendMaxLate is the maximum age a late sample may have and still be
	// accepted into the history buffer.
	ResendMaxLate time.Duration
	// BufferLength is the per-device pose history length.
	BufferLength int
	// Seed is the deterministic seed for simulated input streams and for any
	// randomized engine behavior.
	Seed int64
	// AlgorithmVersion is stamped into reproducibility manifests so that
	// results produced by an older algorithm can be recognized.
	AlgorithmVersion string
}

// DefaultConfig returns a deterministic default configuration: prediction and
// interpolation enabled with a 20ms horizon, 64-frame history and 500ms late
// tolerance.
func DefaultConfig() Config {
	return Config{
		Space:                SpaceFloor,
		PredictionEnabled:    true,
		PredictAhead:         20 * time.Millisecond,
		InterpolationEnabled: true,
		ResendMaxLate:        500 * time.Millisecond,
		BufferLength:         64,
		Seed:                 1,
		AlgorithmVersion:     AlgorithmVersion,
	}
}

// AlgorithmVersion identifies the pose/session algorithm used to build
// recordings. A different version means recorded outputs are not guaranteed
// bit-comparable.
const AlgorithmVersion = "xr-algo-1"
