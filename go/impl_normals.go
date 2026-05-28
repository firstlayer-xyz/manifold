package manifold

import (
	"math"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// SetNormalsAndCoplanar is the Go port of C++
// Manifold::Impl::SetNormalsAndCoplanar (src/impl.cpp).
//
// Step 1: per triangle, compute the face normal (cross product of
// edge0 and edge1), record area² for priority, reset coplanarID = -1.
// A boundary tri (Start(3*tri) < 0) gets area² = 0 and no normal.
//
// Step 2: stable sort triangle indices by area² descending.
//
// Step 3: greedy flood-fill on coplanar regions. Starting at the
// highest-area unassigned tri, walk halfedge adjacencies (via Pair),
// and merge neighbors whose 3rd vertex lies within tolerance_ of the
// seed plane.
//
// Step 4: call CalculateVertNormals.
//
// C++ runs step 1 under for_each_n(autoPolicy(numTri), countAt(0),
// numTri) — no-threshold form, defaults to kSeqThreshold (1e4).
// Steps 2 and 3 stay serial as in C++.
func (mi *MutableImpl) SetNormalsAndCoplanar() {
	verts := mi.h.Verts()
	starts := mi.h.HalfedgeStartsRO()
	pairs := mi.h.HalfedgePairsRO()
	numTri := len(starts) / 3
	tolerance := mi.h.GetTolerance()

	mi.h.ResizeFaceNormals(numTri)
	faceNormals := mi.h.FaceNormalsMut()

	type triPriority struct {
		area2 float64
		tri   int
	}
	prio := make([]triPriority, numTri)
	coplanarID := make([]int32, numTri)
	policy := parallel.AutoPolicy(numTri)
	parallel.ForEachN(policy, numTri, func(tri int) {
		coplanarID[tri] = -1
		if starts[3*tri] < 0 {
			prio[tri] = triPriority{0, tri}
			return
		}
		v := verts[starts[3*tri]]
		e1Start := starts[nextHalfedge(3*tri)]
		e2Start := starts[nextHalfedge(3*tri+1)]
		n := verts[e1Start].Sub(v).Cross(verts[e2Start].Sub(v))
		length := math.Sqrt(n.Dot(n))
		if length == 0 || math.IsNaN(length) {
			faceNormals[tri] = geom.Vec3{X: 0, Y: 0, Z: 1}
		} else {
			faceNormals[tri] = geom.Vec3{X: n.X / length, Y: n.Y / length, Z: n.Z / length}
			if math.IsNaN(faceNormals[tri].X) {
				faceNormals[tri] = geom.Vec3{X: 0, Y: 0, Z: 1}
			}
		}
		prio[tri] = triPriority{area2: n.Dot(n), tri: tri}
	})

	// stable_sort by area² descending. C++ uses stable_sort (no policy
	// form, defaults to autoPolicy + parallel TBB merge sort).
	parallel.StableSort(policy, prio, func(a, b triPriority) bool {
		return a.area2 > b.area2
	})

	// Greedy flood-fill — inherently serial (each iteration depends on
	// what previous iterations marked).
	var interior []int
	for _, tp := range prio {
		if coplanarID[tp.tri] >= 0 {
			continue
		}
		coplanarID[tp.tri] = int32(tp.tri)
		if starts[3*tp.tri] < 0 {
			continue
		}
		base := verts[starts[3*tp.tri]]
		normal := faceNormals[tp.tri]
		interior = append(interior[:0],
			3*tp.tri, 3*tp.tri+1, 3*tp.tri+2)
		for len(interior) > 0 {
			top := interior[len(interior)-1]
			h := nextHalfedge(int(pairs[top]))
			interior = interior[:len(interior)-1]
			if coplanarID[h/3] >= 0 {
				continue
			}
			vEnd := verts[starts[nextHalfedge(h)]]
			if math.Abs(vEnd.Sub(base).Dot(normal)) < tolerance {
				tri := h / 3
				coplanarID[tri] = int32(tp.tri)
				faceNormals[tri] = normal

				if len(interior) == 0 || h != int(pairs[interior[len(interior)-1]]) {
					interior = append(interior, h)
				} else {
					interior = interior[:len(interior)-1]
				}
				interior = append(interior, nextHalfedge(h))
			}
		}
	}

	mi.h.SetCoplanarIDs(coplanarID)
	mi.CalculateVertNormals()
}

// CalculateVertNormals is the Go port of C++
// Manifold::Impl::CalculateVertNormals (src/impl.cpp).
//
// Three parallel passes:
//  1. Init vertHalfedgeMap[v] = MaxInt32 — parallel.Fill.
//  2. For every halfedge i, atomic-min vertHalfedgeMap[start_i] = i.
//     C++ uses a compare_exchange_strong loop; we use the same
//     primitive on []int32 + atomic.CompareAndSwapInt32.
//  3. For every vert, walk the ForVert fan from the cached first
//     edge and accumulate angle-weighted face normals.
//
// C++ uses policy = autoPolicy(NumTri()) — no-threshold form,
// defaults to 1e4.
func (mi *MutableImpl) CalculateVertNormals() {
	verts := mi.h.Verts()
	starts := mi.h.HalfedgeStartsRO()
	pairs := mi.h.HalfedgePairsRO()
	faceNormals := mi.h.FaceNormalsMut()
	numVert := len(verts)
	numHalfedge := len(starts)
	policy := parallel.AutoPolicy(len(starts) / 3)

	mi.h.ResizeVertNormals(numVert)
	vertNormals := mi.h.VertNormals()

	const sentinel int32 = math.MaxInt32
	vertHalfedgeMap := make([]int32, numVert)
	// Pass 1: initialize to sentinel. parallel.Fill is parallel-safe.
	parallel.Fill(policy, vertHalfedgeMap, sentinel)

	// Pass 2: atomic-min. Multiple halfedges with the same startVert
	// race; the winner is the smaller halfedge index.
	parallel.ForEachN(policy, numHalfedge, func(i int) {
		v := starts[i]
		if v < 0 {
			return
		}
		newVal := int32(i)
		for {
			old := atomic.LoadInt32(&vertHalfedgeMap[v])
			if old <= newVal {
				return
			}
			if atomic.CompareAndSwapInt32(&vertHalfedgeMap[v], old, newVal) {
				return
			}
		}
	})

	// Pass 3: per-vert normal via the vertex-fan walk. Mirrors the
	// C++ `ForVert(firstEdge, [&](int edge) { ... })` lambda body —
	// for every edge in the fan, accumulate the angle-weighted face
	// normal contribution from the (prevEdge, currEdge) corner.
	parallel.ForEachN(policy, numVert, func(vert int) {
		fe := int(vertHalfedgeMap[vert])
		if fe == int(sentinel) {
			vertNormals[vert] = geom.Vec3{}
			return
		}
		var normal geom.Vec3
		forVert(fe, pairs, func(edge int) {
			v0 := starts[edge]
			v1 := starts[nextHalfedge(edge)]
			v2 := starts[nextHalfedge(nextHalfedge(edge))]
			currEdge := verts[v1].Sub(verts[v0]).SafeNormalize()
			prevEdge := verts[v0].Sub(verts[v2]).SafeNormalize()
			if !currEdge.IsFinite() || !prevEdge.IsFinite() {
				return
			}
			phi := geom.AngleBetween(prevEdge.Scale(-1), currEdge)
			fn := faceNormals[edge/3]
			normal = normal.Add(geom.Vec3{X: phi * fn.X, Y: phi * fn.Y, Z: phi * fn.Z})
		})
		vertNormals[vert] = normal.SafeNormalize()
	})
}
