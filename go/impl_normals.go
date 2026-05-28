package manifold

import (
	"math"
	"sort"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// nextHalfedge mirrors the C++ inline in src/shared.h:
//
//	current += current % 3 == 2 ? -2 : 1
//
// Walks to the next halfedge of the same triangle.
func nextHalfedge(current int) int {
	if current%3 == 2 {
		return current - 2
	}
	return current + 1
}

// setNormalsAndCoplanar is the Go port of C++
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
// Step 4: call calculateVertNormals.
func setNormalsAndCoplanar(mi *bridge.MutableImpl) {
	verts := mi.Verts()
	starts := mi.HalfedgeStartsRO()
	pairs := mi.HalfedgePairsRO()
	numTri := len(starts) / 3
	tolerance := mi.GetTolerance()

	mi.ResizeFaceNormals(numTri)
	faceNormals := mi.FaceNormalsMut()

	type triPriority struct {
		area2 float64
		tri   int
	}
	prio := make([]triPriority, numTri)
	coplanarID := make([]int32, numTri)
	for tri := 0; tri < numTri; tri++ {
		coplanarID[tri] = -1
		if starts[3*tri] < 0 {
			prio[tri] = triPriority{0, tri}
			continue
		}
		v := verts[starts[3*tri]]
		// End(idx) = Start(NextHalfedge(idx)).
		e1Start := starts[nextHalfedge(3*tri)]
		e2Start := starts[nextHalfedge(3*tri+1)]
		// n = cross(End(3*tri) - v, End(3*tri+1) - v)
		n := verts[e1Start].Sub(v).Cross(verts[e2Start].Sub(v))
		length := math.Sqrt(n.Dot(n))
		if length == 0 || math.IsNaN(length) {
			// Degenerate tri: C++ uses {0,0,1} fallback.
			faceNormals[tri] = geom.Vec3{X: 0, Y: 0, Z: 1}
		} else {
			faceNormals[tri] = geom.Vec3{X: n.X / length, Y: n.Y / length, Z: n.Z / length}
			if math.IsNaN(faceNormals[tri].X) {
				faceNormals[tri] = geom.Vec3{X: 0, Y: 0, Z: 1}
			}
		}
		prio[tri] = triPriority{area2: n.Dot(n), tri: tri}
	}

	// stable_sort by area² descending.
	sort.SliceStable(prio, func(a, b int) bool {
		return prio[a].area2 > prio[b].area2
	})

	// Greedy flood-fill: seed tri propagates its coplanarID + normal to
	// neighbors within tolerance of its plane.
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
			// v = vertPos_[End(h)] = vertPos_[Start(NextHalfedge(h))]
			vEnd := verts[starts[nextHalfedge(h)]]
			if math.Abs(vEnd.Sub(base).Dot(normal)) < tolerance {
				tri := h / 3
				coplanarID[tri] = int32(tp.tri)
				faceNormals[tri] = normal

				// Mirror the C++ dedup logic: avoid pushing h if the
				// top of the stack is already h's pair (i.e. we're
				// about to walk the same edge from both sides).
				if len(interior) == 0 || h != int(pairs[interior[len(interior)-1]]) {
					interior = append(interior, h)
				} else {
					interior = interior[:len(interior)-1]
				}
				interior = append(interior, nextHalfedge(h))
			}
		}
	}

	mi.SetCoplanarIDs(coplanarID)
	calculateVertNormals(mi)
}

// calculateVertNormals is the Go port of C++
// Manifold::Impl::CalculateVertNormals (src/impl.cpp).
//
// For each vertex, finds an incident halfedge, then walks the
// surrounding fan (ForVert), accumulating angle-weighted face normals.
// The C++ uses an atomic int per vertex initialized to INT_MAX, then
// each halfedge atomic-mins itself into vertHalfedgeMap[start_vert].
// Go runs sequentially so we just take the first-seen halfedge per
// vertex (which matches the C++ deterministic outcome for serial
// execution: the smallest halfedge index).
func calculateVertNormals(mi *bridge.MutableImpl) {
	verts := mi.Verts()
	starts := mi.HalfedgeStartsRO()
	pairs := mi.HalfedgePairsRO()
	faceNormals := mi.FaceNormalsMut()
	numVert := len(verts)
	numHalfedge := len(starts)

	mi.ResizeVertNormals(numVert)
	vertNormals := mi.VertNormals()

	// Per-vertex: minimum halfedge index whose Start is that vertex.
	const sentinel = math.MaxInt32
	firstEdge := make([]int, numVert)
	for i := range firstEdge {
		firstEdge[i] = sentinel
	}
	for i := 0; i < numHalfedge; i++ {
		v := int(starts[i])
		if v < 0 {
			continue
		}
		if i < firstEdge[v] {
			firstEdge[v] = i
		}
	}

	for vert := 0; vert < numVert; vert++ {
		fe := firstEdge[vert]
		if fe == sentinel {
			vertNormals[vert] = geom.Vec3{}
			continue
		}
		var normal geom.Vec3
		// ForVert(firstEdge): walk halfedges around vertex.
		// In C++, ForVert follows: edge → next halfedge of pair, until
		// we loop back to firstEdge OR hit a boundary (pair < 0).
		edge := fe
		for {
			triVerts := [3]int32{
				starts[edge],
				starts[nextHalfedge(edge)],
				starts[nextHalfedge(nextHalfedge(edge))],
			}
			currEdge := verts[triVerts[1]].Sub(verts[triVerts[0]])
			prevEdge := verts[triVerts[0]].Sub(verts[triVerts[2]])
			currLen := math.Sqrt(currEdge.Dot(currEdge))
			prevLen := math.Sqrt(prevEdge.Dot(prevEdge))
			if currLen > 0 && prevLen > 0 &&
				!math.IsNaN(currLen) && !math.IsNaN(prevLen) &&
				!math.IsInf(currLen, 0) && !math.IsInf(prevLen, 0) {
				cn := geom.Vec3{X: currEdge.X / currLen, Y: currEdge.Y / currLen, Z: currEdge.Z / currLen}
				pn := geom.Vec3{X: prevEdge.X / prevLen, Y: prevEdge.Y / prevLen, Z: prevEdge.Z / prevLen}
				d := -pn.Dot(cn)
				var phi float64
				switch {
				case d >= 1:
					phi = 0
				case d <= -1:
					phi = math.Pi
				default:
					phi = math.Acos(d)
				}
				fn := faceNormals[edge/3]
				normal = normal.Add(geom.Vec3{X: phi * fn.X, Y: phi * fn.Y, Z: phi * fn.Z})
			}
			// Advance: next = NextHalfedge(Pair(edge)). Stops if pair < 0
			// (boundary) or we return to firstEdge.
			pair := int(pairs[edge])
			if pair < 0 {
				break
			}
			edge = nextHalfedge(pair)
			if edge == fe {
				break
			}
		}
		// SafeNormalize: normalize if finite, else zero.
		length := math.Sqrt(normal.Dot(normal))
		if length > 0 && !math.IsNaN(length) && !math.IsInf(length, 0) {
			vertNormals[vert] = geom.Vec3{X: normal.X / length, Y: normal.Y / length, Z: normal.Z / length}
		} else {
			vertNormals[vert] = geom.Vec3{}
		}
	}
}
