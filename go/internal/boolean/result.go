package boolean

import "github.com/firstlayer-xyz/manifold/go/internal/geom"

// absInt is std::abs for int.
func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// edgePos is the Go port of the EdgePos struct (boolean_result.cpp:191): a new
// vertex along a retained edge, with its position along the edge (pos, filled
// later in AppendPartialEdges), output vert index, originating crossing id
// (for deterministic tie-break), and whether it starts a span. Its ordering is
// (pos, then collisionID) — applied in PairUp.
type edgePos struct {
	pos         float64
	vert        int
	collisionID int
	isStart     bool
}

// addNewEdgeVerts is the Go port of AddNewEdgeVerts (boolean_result.cpp:204):
// for each edge(P)/face(Q) crossing, record the new vertex on P's edge
// (edgesP[edgeP]) and on the two new edges where Q's face meets the two P-faces
// sharing edgeP (edgesNew[keyLeft/keyRight]); the span direction comes from the
// crossing inclusion sign, duplicated |inclusion| times, with v12R remapping to
// the output vert index. forward=false mirrors P<->Q. SEQ path: the C++
// per-key mutex locking and its hashes are parallel-only, so they're dropped;
// &edgesP[k] (addressable map value) becomes append-to-map.
func addNewEdgeVerts(edgesP map[int][]edgePos, edgesNew map[[2]int][]edgePos,
	p1q2 [][2]int, i12, v12R []int, halfedgeP halfedges, forward bool, offset int) {
	for i := range p1q2 {
		edgeSlot, faceSlot := 0, 1
		if !forward {
			edgeSlot, faceSlot = 1, 0
		}
		edgeP := p1q2[i][edgeSlot]
		faceQ := p1q2[i][faceSlot]
		vert := v12R[i]
		inclusion := i12[i]

		keyRight := [2]int{halfedgeP.Pair(edgeP) / 3, faceQ}
		keyLeft := [2]int{edgeP / 3, faceQ}
		if !forward {
			keyRight[0], keyRight[1] = keyRight[1], keyRight[0]
			keyLeft[0], keyLeft[1] = keyLeft[1], keyLeft[0]
		}

		direction := inclusion < 0
		// isStart per target (C++ uses bool XOR ^; Go uses !=). The dead
		// `direction = !direction` loop tail is omitted (the targets capture
		// isStart by value).
		push := func(vec []edgePos, isStart bool) []edgePos {
			for j := 0; j < absInt(inclusion); j++ {
				vec = append(vec, edgePos{pos: 0.0, vert: vert + j, collisionID: i + offset, isStart: isStart})
			}
			return vec
		}
		edgesP[edgeP] = push(edgesP[edgeP], direction)
		edgesNew[keyRight] = push(edgesNew[keyRight], direction != !forward)
		edgesNew[keyLeft] = push(edgesNew[keyLeft], direction != forward)
	}
}

// outImpl is the output Manifold::Impl being assembled by Boolean3::Result.
// Fields are filled as the assembly progresses; more are added as later helpers
// land (halfedge, triRef, properties).
type outImpl struct {
	vertPos            []geom.Vec3
	faceNormal         []geom.Vec3
	epsilon, tolerance float64
}

// sizeOutput is the Go port of SizeOutput (boolean_result.cpp:104): count the
// output sides per retained face (from the retained-vertex inclusions i03/i30
// and the new-vertex crossings i12/i21), drop faces with no sides, and build
//   - facePQ2R: old (P|Q) face index -> new face index (inclusive scan of the
//     keep flag), sized numTriP+numTriQ;
//   - faceEdge:  prefix sum of the kept faces' side counts (the per-face edge
//     offset into the output halfedge buffer);
//   - outR.faceNormal: the kept faces' normals (Q's negated when invertQ).
//
// SEQ path (the atomic CountNewVerts variant is the parallel optimization).
func sizeOutput(outR *outImpl, inP, inQ *mesh, i03, i30, i12, i21 []int, p1q2, p2q1 [][2]int, invertQ bool) (faceEdge, facePQ2R []int) {
	numTriP := len(inP.halfedge.starts) / 3
	numTriQ := len(inQ.halfedge.starts) / 3
	sidesPerFacePQ := make([]int, numTriP+numTriQ)
	sidesPerFaceP := sidesPerFacePQ[:numTriP]
	sidesPerFaceQ := sidesPerFacePQ[numTriP:]

	for i := 0; i < numTriP; i++ {
		countVerts(inP.halfedge, sidesPerFaceP, i03, i)
	}
	for i := 0; i < numTriQ; i++ {
		countVerts(inQ.halfedge, sidesPerFaceQ, i30, i)
	}
	for idx := range i12 {
		countNewVerts(sidesPerFaceP, sidesPerFaceQ, i12, p1q2, inP.halfedge, idx, false)
	}
	for idx := range i21 {
		countNewVerts(sidesPerFaceQ, sidesPerFaceP, i21, p2q1, inQ.halfedge, idx, true)
	}

	// facePQ2R = inclusive_scan of keepFace (sides>0 ? 1 : 0), into [1:].
	facePQ2R = make([]int, numTriP+numTriQ+1)
	acc := 0
	for i := 0; i < len(sidesPerFacePQ); i++ {
		if sidesPerFacePQ[i] > 0 {
			acc++
		}
		facePQ2R[i+1] = acc
	}
	numFaceR := facePQ2R[len(facePQ2R)-1]
	facePQ2R = facePQ2R[:numTriP+numTriQ]

	// faceNormal: kept P faces, then kept Q faces (negated if invertQ).
	outR.faceNormal = make([]geom.Vec3, numFaceR)
	w := 0
	for i := 0; i < numTriP; i++ {
		if sidesPerFacePQ[i] > 0 {
			outR.faceNormal[w] = inP.faceNormal[i]
			w++
		}
	}
	for i := 0; i < numTriQ; i++ {
		if sidesPerFacePQ[numTriP+i] > 0 {
			n := inQ.faceNormal[i]
			if invertQ {
				n = n.Scale(-1)
			}
			outR.faceNormal[w] = n
			w++
		}
	}

	// faceEdge = [0] + inclusive_scan of the kept faces' side counts (the C++
	// remove-zeros then scan).
	var nonZero []int
	for _, s := range sidesPerFacePQ {
		if s != 0 {
			nonZero = append(nonZero, s)
		}
	}
	faceEdge = make([]int, len(nonZero)+1)
	acc = 0
	for k, s := range nonZero {
		acc += s
		faceEdge[k+1] = acc
	}
	return faceEdge, facePQ2R
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
