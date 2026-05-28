package collider

import (
	"math/rand"
	"sync"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// TestCollider_Stress builds a Collider over a few thousand random
// boxes and runs many random queries. For each query, the recorded
// hit set must exactly equal the brute-force overlap set (linear
// scan over all leaves). Runs under -race to catch any data race
// in the parallel path.
func TestCollider_Stress(t *testing.T) {
	const numLeaves = 2000
	const numQueries = 200
	rng := rand.New(rand.NewSource(0x12345678))

	leaves := make([]geom.Box, numLeaves)
	for i := range leaves {
		cx, cy, cz := rng.Float64()*100, rng.Float64()*100, rng.Float64()*100
		hx, hy, hz := rng.Float64()*0.5+0.1, rng.Float64()*0.5+0.1, rng.Float64()*0.5+0.1
		leaves[i] = geom.Box{
			Min: geom.Vec3{X: cx - hx, Y: cy - hy, Z: cz - hz},
			Max: geom.Vec3{X: cx + hx, Y: cy + hy, Z: cz + hz},
		}
	}
	c, order := buildFrom(leaves)

	// Queries are wider boxes that overlap many leaves.
	queries := make([]geom.Box, numQueries)
	for i := range queries {
		cx, cy, cz := rng.Float64()*100, rng.Float64()*100, rng.Float64()*100
		hx, hy, hz := rng.Float64()*5+1, rng.Float64()*5+1, rng.Float64()*5+1
		queries[i] = geom.Box{
			Min: geom.Vec3{X: cx - hx, Y: cy - hy, Z: cz - hz},
			Max: geom.Vec3{X: cx + hx, Y: cy + hy, Z: cz + hz},
		}
	}

	// Collider path.
	var mu sync.Mutex
	gotHits := make([]map[int]bool, numQueries)
	for i := range gotHits {
		gotHits[i] = map[int]bool{}
	}
	c.CollisionsBox(queries, true, false, func(qi, li int) {
		mu.Lock()
		gotHits[qi][order[li]] = true
		mu.Unlock()
	})

	// Brute force.
	for qi, q := range queries {
		want := map[int]bool{}
		for li, b := range leaves {
			if b.DoesOverlap(q) {
				want[li] = true
			}
		}
		if len(gotHits[qi]) != len(want) {
			t.Fatalf("query %d: |hits|=%d want %d", qi, len(gotHits[qi]), len(want))
		}
		for k := range want {
			if !gotHits[qi][k] {
				t.Errorf("query %d: missing leaf %d", qi, k)
			}
		}
		for k := range gotHits[qi] {
			if !want[k] {
				t.Errorf("query %d: spurious leaf %d", qi, k)
			}
		}
	}
}
