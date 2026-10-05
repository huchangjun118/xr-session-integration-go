// Command xrdemo runs the deterministic in-process simulator against the
// unified session/tracking integration layer and prints reproducibility
// manifests.
package main

import (
	"fmt"

	"github.com/huchangjun118/xr-session-integration-go/pkg/xr"
)

func run(seed int64) xr.Manifest {
	cfg := xr.DefaultConfig()
	cfg.Seed = seed
	e := xr.NewEngine(cfg)
	sim := xr.NewSimulator(xr.DefaultSimConfig(seed))
	rec := xr.NewRecorder()
	m, err := sim.Run(e, rec)
	if err != nil {
		panic(err)
	}
	return m
}

func main() {
	m1 := run(42)
	m2 := run(42)

	fmt.Println("== xr session/tracking integration demo ==")
	fmt.Printf("algorithm : %s\n", m1.AlgorithmVersion)
	fmt.Printf("snapshots : %d\n", m1.Snapshots)
	fmt.Printf("run A hash: %s\n", m1.Hash)
	fmt.Printf("run B hash: %s\n", m2.Hash)

	d := xr.CompareManifolds(m1, m2)
	fmt.Printf("reproduce : %t (%s)\n", d.Reproducible, d.Reason)

	// A recording stamped with a newer algorithm version is explicitly judged
	// non-reproducible instead of silently differing.
	older := m1
	older.AlgorithmVersion = "xr-algo-0"
	d2 := xr.CompareManifolds(older, m2)
	fmt.Printf("old algo  : %t (%s)\n", d2.Reproducible, d2.Reason)
}
