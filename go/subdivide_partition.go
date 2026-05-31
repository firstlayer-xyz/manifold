// Partition is the Go port of the file-local Partition class in src/subdivision.cpp
// (lines 31-380): the purely-topological sub-triangulation of a triangle or quad,
// parameterized only by the number of divisions on each side. Because the
// triangulation depends only on the (sorted) per-side division counts, the results
// are cached and reused across similar faces; the final surface shape is supplied
// later by the barycentric coordinates + tangents in Subdivide.
package manifold

import (
	"math"
	"sync"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

type partition struct {
	// idx maps the input divisions onto the sorted/cached divisions. The cached
	// partitions don't carry idx; GetPartition sets it on the returned copy.
	idx             [4]int
	sortedDivisions [4]int
	vertBary        []geom.Vec4
	triVert         [][3]int
}

func (p *partition) interiorOffset() int {
	return p.sortedDivisions[0] + p.sortedDivisions[1] + p.sortedDivisions[2] + p.sortedDivisions[3]
}

func (p *partition) numInterior() int { return len(p.vertBary) - p.interiorOffset() }

var (
	partitionCacheMu sync.Mutex
	partitionCache   = map[[4]int]partition{}
)

// getPartition is the Go port of Partition::GetPartition (subdivision.cpp:48):
// sort the divisions into the canonical (cached) order, look up / build the cached
// partition, and tag the copy with the permutation idx.
func getPartition(divisions [4]int) partition {
	if divisions[0] == 0 {
		return partition{} // skip wrong side of quad
	}
	sortedDiv := divisions
	triIdx := [4]int{0, 1, 2, 3}
	if divisions[3] == 0 { // triangle
		if sortedDiv[2] > sortedDiv[1] {
			sortedDiv[2], sortedDiv[1] = sortedDiv[1], sortedDiv[2]
			triIdx[2], triIdx[1] = triIdx[1], triIdx[2]
		}
		if sortedDiv[1] > sortedDiv[0] {
			sortedDiv[1], sortedDiv[0] = sortedDiv[0], sortedDiv[1]
			triIdx[1], triIdx[0] = triIdx[0], triIdx[1]
			if sortedDiv[2] > sortedDiv[1] {
				sortedDiv[2], sortedDiv[1] = sortedDiv[1], sortedDiv[2]
				triIdx[2], triIdx[1] = triIdx[1], triIdx[2]
			}
		}
	} else { // quad
		minIdx := 0
		minVal := divisions[0]
		next := divisions[1]
		for _, i := range []int{1, 2, 3} {
			n := divisions[(i+1)%4]
			if divisions[i] < minVal || (divisions[i] == minVal && n < next) {
				minIdx = i
				minVal = divisions[i]
				next = n
			}
		}
		tmp := sortedDiv
		for _, i := range []int{0, 1, 2, 3} {
			triIdx[i] = (i + minIdx) % 4
			sortedDiv[i] = tmp[triIdx[i]]
		}
	}

	p := getCachedPartition(sortedDiv)
	p.idx = triIdx
	return p
}

// reindex is the Go port of Partition::Reindex (subdivision.cpp:94): map the
// cached partition's local vert indices onto the actual mesh vert indices, given
// this face's corner verts, per-edge offsets, edge directions, and interior offset.
func (p *partition) reindex(triVerts [4]int, edgeOffsets [4]int, edgeFwd [4]bool, interiorOffset int) [][3]int {
	newVerts := make([]int, 0, len(p.vertBary))
	triIdx := p.idx
	outTri := [4]int{0, 1, 2, 3}
	if triVerts[3] < 0 && p.idx[1] != geom.Next3(p.idx[0]) {
		triIdx = [4]int{p.idx[2], p.idx[0], p.idx[1], p.idx[3]}
		for i := range edgeFwd {
			edgeFwd[i] = !edgeFwd[i]
		}
		outTri[0], outTri[1] = outTri[1], outTri[0]
	}
	for _, i := range []int{0, 1, 2, 3} {
		if triVerts[triIdx[i]] >= 0 {
			newVerts = append(newVerts, triVerts[triIdx[i]])
		}
	}
	for _, i := range []int{0, 1, 2, 3} {
		n := p.sortedDivisions[i] - 1
		offset := edgeOffsets[p.idx[i]]
		if !edgeFwd[p.idx[i]] {
			offset += n - 1
		}
		for j := 0; j < n; j++ {
			newVerts = append(newVerts, offset)
			if edgeFwd[p.idx[i]] {
				offset++
			} else {
				offset--
			}
		}
	}
	old := len(newVerts)
	for len(newVerts) < len(p.vertBary) {
		newVerts = append(newVerts, 0)
	}
	for k := old; k < len(newVerts); k++ {
		newVerts[k] = interiorOffset + (k - old)
	}

	newTriVert := make([][3]int, len(p.triVert))
	for tri := range p.triVert {
		for _, j := range []int{0, 1, 2} {
			newTriVert[tri][outTri[j]] = newVerts[p.triVert[tri][j]]
		}
	}
	return newTriVert
}

// getCachedPartition is the Go port of Partition::GetCachedPartition
// (subdivision.cpp:142). For tris the input must be sorted n[0]>=n[1]>=n[2]>0.
func getCachedPartition(n [4]int) partition {
	partitionCacheMu.Lock()
	if p, ok := partitionCache[n]; ok {
		partitionCacheMu.Unlock()
		return p
	}
	partitionCacheMu.Unlock()

	var p partition
	p.sortedDivisions = n
	if n[3] > 0 { // quad
		p.vertBary = append(p.vertBary,
			geom.Vec4{X: 1}, geom.Vec4{Y: 1}, geom.Vec4{Z: 1}, geom.Vec4{W: 1})
		var edgeOffsets [4]int
		edgeOffsets[0] = 4
		for _, i := range []int{0, 1, 2, 3} {
			if i > 0 {
				edgeOffsets[i] = edgeOffsets[i-1] + n[i-1] - 1
			}
			nextBary := p.vertBary[(i+1)%4]
			for j := 1; j < n[i]; j++ {
				p.vertBary = append(p.vertBary, p.vertBary[i].Lerp(nextBary, float64(j)/float64(n[i])))
			}
		}
		partitionQuad(&p.triVert, &p.vertBary, [4]int{0, 1, 2, 3}, edgeOffsets,
			[4]int{n[0] - 1, n[1] - 1, n[2] - 1, n[3] - 1}, [4]bool{true, true, true, true})
	} else { // tri
		p.vertBary = append(p.vertBary, geom.Vec4{X: 1}, geom.Vec4{Y: 1}, geom.Vec4{Z: 1})
		for _, i := range []int{0, 1, 2} {
			nextBary := p.vertBary[(i+1)%3]
			for j := 1; j < n[i]; j++ {
				p.vertBary = append(p.vertBary, p.vertBary[i].Lerp(nextBary, float64(j)/float64(n[i])))
			}
		}
		edgeOffsets := [3]int{3, 3 + n[0] - 1, 3 + n[0] - 1 + n[1] - 1}

		f := float64(n[2]*n[2] + n[0]*n[0])
		if n[1] == 1 {
			if n[0] == 1 {
				p.triVert = append(p.triVert, [3]int{0, 1, 2})
			} else {
				partitionFan(&p.triVert, [3]int{0, 1, 2}, n[0]-1, edgeOffsets[0])
			}
		} else if float64(n[1]*n[1]) > f-math.Sqrt(2.0)*float64(n[0]*n[2]) { // acute-ish
			p.triVert = append(p.triVert, [3]int{edgeOffsets[1] - 1, 1, edgeOffsets[1]})
			partitionQuad(&p.triVert, &p.vertBary,
				[4]int{edgeOffsets[1] - 1, edgeOffsets[1], 2, 0},
				[4]int{-1, edgeOffsets[1] + 1, edgeOffsets[2], edgeOffsets[0]},
				[4]int{0, n[1] - 2, n[2] - 1, n[0] - 2}, [4]bool{true, true, true, true})
		} else { // obtuse -> split into two acute
			ns := n[0] - 2
			if r := int(math.Round((f - float64(n[1]*n[1])) / float64(2*n[0]))); r < ns {
				ns = r
			}
			nh := int(math.Round(math.Sqrt(float64(n[2]*n[2] - ns*ns))))
			if nh < 1 {
				nh = 1
			}
			hOffset := len(p.vertBary)
			middleBary := p.vertBary[edgeOffsets[0]+ns-1]
			for j := 1; j < nh; j++ {
				p.vertBary = append(p.vertBary, p.vertBary[2].Lerp(middleBary, float64(j)/float64(nh)))
			}

			p.triVert = append(p.triVert, [3]int{edgeOffsets[1] - 1, 1, edgeOffsets[1]})
			partitionQuad(&p.triVert, &p.vertBary,
				[4]int{edgeOffsets[1] - 1, edgeOffsets[1], 2, edgeOffsets[0] + ns - 1},
				[4]int{-1, edgeOffsets[1] + 1, hOffset, edgeOffsets[0] + ns},
				[4]int{0, n[1] - 2, nh - 1, n[0] - ns - 2}, [4]bool{true, true, true, true})

			if n[2] == 1 {
				partitionFan(&p.triVert, [3]int{0, edgeOffsets[0] + ns - 1, 2}, ns-1, edgeOffsets[0])
			} else if ns == 1 {
				p.triVert = append(p.triVert, [3]int{hOffset, 2, edgeOffsets[2]})
				partitionQuad(&p.triVert, &p.vertBary,
					[4]int{hOffset, edgeOffsets[2], 0, edgeOffsets[0]},
					[4]int{-1, edgeOffsets[2] + 1, -1, hOffset + nh - 2},
					[4]int{0, n[2] - 2, ns - 1, nh - 2}, [4]bool{true, true, true, false})
			} else {
				p.triVert = append(p.triVert, [3]int{hOffset - 1, 0, edgeOffsets[0]})
				partitionQuad(&p.triVert, &p.vertBary,
					[4]int{hOffset - 1, edgeOffsets[0], edgeOffsets[0] + ns - 1, 2},
					[4]int{-1, edgeOffsets[0] + 1, hOffset + nh - 2, edgeOffsets[2]},
					[4]int{0, ns - 2, nh - 1, n[2] - 2}, [4]bool{true, true, false, true})
			}
		}
	}

	partitionCacheMu.Lock()
	partitionCache[n] = p
	partitionCacheMu.Unlock()
	return p
}

// partitionFan is the Go port of Partition::PartitionFan (subdivision.cpp:249):
// side 0 has added edges, sides 1 and 2 do not; the fan spreads from vert 2.
func partitionFan(triVert *[][3]int, cornerVerts [3]int, added, edgeOffset int) {
	last := cornerVerts[0]
	for i := 0; i < added; i++ {
		next := edgeOffset + i
		*triVert = append(*triVert, [3]int{last, next, cornerVerts[2]})
		last = next
	}
	*triVert = append(*triVert, [3]int{last, cornerVerts[1], cornerVerts[2]})
}

// partitionQuad is the Go port of Partition::PartitionQuad (subdivision.cpp:262):
// partitions are parallel to the first edge unless two consecutive edgeAdded are
// zero, in which case a terminal triangulation is performed.
func partitionQuad(triVert *[][3]int, vertBary *[]geom.Vec4, cornerVerts [4]int,
	edgeOffsets [4]int, edgeAdded [4]int, edgeFwd [4]bool) {
	getEdgeVert := func(edge, idx int) int {
		if edgeFwd[edge] {
			return edgeOffsets[edge] + idx
		}
		return edgeOffsets[edge] - idx
	}

	corner := -1
	last := 3
	maxEdge := -1
	for _, i := range []int{0, 1, 2, 3} {
		if corner == -1 && edgeAdded[i] == 0 && edgeAdded[last] == 0 {
			corner = i
		}
		if edgeAdded[i] > 0 {
			if maxEdge == -1 {
				maxEdge = i
			} else {
				maxEdge = -2
			}
		}
		last = i
	}
	if corner >= 0 { // terminate
		if maxEdge >= 0 {
			edge := [4]int{maxEdge % 4, (1 + maxEdge) % 4, (2 + maxEdge) % 4, (3 + maxEdge) % 4}
			middle := edgeAdded[maxEdge] / 2
			*triVert = append(*triVert, [3]int{cornerVerts[edge[2]], cornerVerts[edge[3]], getEdgeVert(maxEdge, middle)})
			lastV := cornerVerts[edge[0]]
			for i := 0; i <= middle; i++ {
				next := getEdgeVert(maxEdge, i)
				*triVert = append(*triVert, [3]int{cornerVerts[edge[3]], lastV, next})
				lastV = next
			}
			lastV = cornerVerts[edge[1]]
			for i := edgeAdded[maxEdge] - 1; i >= middle; i-- {
				next := getEdgeVert(maxEdge, i)
				*triVert = append(*triVert, [3]int{cornerVerts[edge[2]], next, lastV})
				lastV = next
			}
		} else {
			sideVert := cornerVerts[0] // initial value is unused
			for _, j := range []int{1, 2} {
				side := (corner + j) % 4
				if j == 2 && edgeAdded[side] > 0 {
					*triVert = append(*triVert, [3]int{cornerVerts[side], getEdgeVert(side, 0), sideVert})
				} else {
					sideVert = cornerVerts[side]
				}
				for i := 0; i < edgeAdded[side]; i++ {
					nextVert := getEdgeVert(side, i)
					*triVert = append(*triVert, [3]int{cornerVerts[corner], sideVert, nextVert})
					sideVert = nextVert
				}
				if j == 2 || edgeAdded[side] == 0 {
					*triVert = append(*triVert, [3]int{cornerVerts[corner], sideVert, cornerVerts[(corner+j+1)%4]})
				}
			}
		}
		return
	}

	// recursively partition
	partitions := 1 + min(edgeAdded[1], edgeAdded[3])
	newCornerVerts := [4]int{cornerVerts[1], -1, -1, cornerVerts[0]}
	newEdgeOffsets := [4]int{edgeOffsets[1], -1, getEdgeVert(3, edgeAdded[3]+1), edgeOffsets[0]}
	newEdgeAdded := [4]int{0, -1, 0, edgeAdded[0]}
	newEdgeFwd := [4]bool{edgeFwd[1], true, edgeFwd[3], edgeFwd[0]}

	for i := 1; i < partitions; i++ {
		cornerOffset1 := (edgeAdded[1] * i) / partitions
		cornerOffset3 := edgeAdded[3] - 1 - (edgeAdded[3]*i)/partitions
		nextOffset1 := getEdgeVert(1, cornerOffset1+1)
		nextOffset3 := getEdgeVert(3, cornerOffset3+1)
		added := int(math.Round(lerpScalar(float64(edgeAdded[0]), float64(edgeAdded[2]), float64(i)/float64(partitions))))

		newCornerVerts[1] = getEdgeVert(1, cornerOffset1)
		newCornerVerts[2] = getEdgeVert(3, cornerOffset3)
		newEdgeAdded[0] = absInt(nextOffset1-newEdgeOffsets[0]) - 1
		newEdgeAdded[1] = added
		newEdgeAdded[2] = absInt(nextOffset3-newEdgeOffsets[2]) - 1
		newEdgeOffsets[1] = len(*vertBary)
		newEdgeOffsets[2] = nextOffset3

		for j := 0; j < added; j++ {
			*vertBary = append(*vertBary, (*vertBary)[newCornerVerts[1]].Lerp(
				(*vertBary)[newCornerVerts[2]], (float64(j)+1.0)/(float64(added)+1.0)))
		}

		partitionQuad(triVert, vertBary, newCornerVerts, newEdgeOffsets, newEdgeAdded, newEdgeFwd)

		newCornerVerts[0] = newCornerVerts[1]
		newCornerVerts[3] = newCornerVerts[2]
		newEdgeAdded[3] = newEdgeAdded[1]
		newEdgeOffsets[0] = nextOffset1
		newEdgeOffsets[3] = newEdgeOffsets[1] + newEdgeAdded[1] - 1
		newEdgeFwd[3] = false
	}

	newCornerVerts[1] = cornerVerts[2]
	newCornerVerts[2] = cornerVerts[3]
	newEdgeOffsets[1] = edgeOffsets[2]
	newEdgeAdded[0] = edgeAdded[1] - absInt(newEdgeOffsets[0]-edgeOffsets[1])
	newEdgeAdded[1] = edgeAdded[2]
	newEdgeAdded[2] = absInt(newEdgeOffsets[2]-edgeOffsets[3]) - 1
	newEdgeOffsets[2] = edgeOffsets[3]
	newEdgeFwd[1] = edgeFwd[2]

	partitionQuad(triVert, vertBary, newCornerVerts, newEdgeOffsets, newEdgeAdded, newEdgeFwd)
}

// lerpScalar (la::lerp for scalars) lives in impl_levelset.go.

func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
