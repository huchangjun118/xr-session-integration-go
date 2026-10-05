package xr_test

import (
	"fmt"

	"github.com/huchangjun118/xr-session-integration-go/pkg/xr"
)

// Example shows the minimal deterministic run: two replays of the same seed
// produce identical manifests.
func Example() {
	run := func(seed int64) xr.Manifest {
		cfg := xr.DefaultConfig()
		cfg.Seed = seed
		e := xr.NewEngine(cfg)
		sim := xr.NewSimulator(xr.DefaultSimConfig(seed))
		rec := xr.NewRecorder()
		m, _ := sim.Run(e, rec)
		return m
	}
	a := run(42)
	b := run(42)
	d := xr.CompareManifolds(a, b)
	fmt.Println(d.Reproducible, d.Reason)
	// Output: true identical
}
