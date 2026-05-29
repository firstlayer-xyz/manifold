package boolean

import "github.com/firstlayer-xyz/manifold/go/internal/geom"

// absInt is std::abs for int.
func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// duplicateVerts is the Go port of the DuplicateVerts functor
// (boolean_result.cpp:54): scatter source vertex `vert` into vertPosR at
// vertR[vert], repeated |inclusion[vert]| times (inclusion not in [-1,1] means
// the vertex is duplicated for multiple output components).
func duplicateVerts(vertPosR []geom.Vec3, inclusion, vertR []int, vertPosP []geom.Vec3, vert int) {
	n := absInt(inclusion[vert])
	for i := 0; i < n; i++ {
		vertPosR[vertR[vert]+i] = vertPosP[vert]
	}
}

// countVerts is the Go port of the CountVerts functor (boolean_result.cpp:68):
// count[i] += the sum over tri i's three start verts of |inclusion[startVert]|.
func countVerts(h halfedges, count, inclusion []int, i int) {
	for j := 0; j < 3; j++ {
		count[i] += absInt(inclusion[h.Start(3*i+j)])
	}
}

// countNewVerts is the Go port of the CountNewVerts functor
// (boolean_result.cpp:79), non-atomic SEQ form: for crossing idx, add
// |i12[idx]| to the face countQ[faceQ] and to both P-faces incident on edgeP
// (edgeP/3 and Pair(edgeP)/3). `inverted` selects the edge/face slots of pq
// (false: edge=pq[0],face=pq[1]; true: edge=pq[1],face=pq[0]).
func countNewVerts(countP, countQ, i12 []int, pq [][2]int, h halfedges, idx int, inverted bool) {
	edgeSlot, faceSlot := 0, 1
	if inverted {
		edgeSlot, faceSlot = 1, 0
	}
	edgeP := pq[idx][edgeSlot]
	faceQ := pq[idx][faceSlot]
	inclusion := absInt(i12[idx])
	countQ[faceQ] += inclusion
	countP[edgeP/3] += inclusion
	countP[h.Pair(edgeP)/3] += inclusion
}
