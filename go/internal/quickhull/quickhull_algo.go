package quickhull

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// faceData mirrors C++ QuickHull::FaceData — used during the
// visible-face flood fill.
type faceData struct {
	faceIndex          int
	enteredFromHalfedge int
}

// QuickHull mirrors the C++ QuickHull class. Pass the input points
// to NewQuickHull, then call BuildMesh to get back (halfedges,
// vertices) of the convex hull.
type QuickHull struct {
	epsilon                 float64
	epsilonSquared          float64
	scale                   float64
	planar                  bool
	planarPointCloudTemp    []geom.Vec3
	originalVertexData      []geom.Vec3
	mesh                    meshBuilder
	extremeValues           [6]int
	failedHorizonEdges      int

	// Reusable scratch slices — the same as the C++ Vec<...>
	// members on QuickHull. Storing them on the struct avoids
	// allocations across iterations.
	newFaceIndices          []int
	newHalfedgeIndices      []int
	visibleFaces            []int
	horizonEdgesData        []int
	possiblyVisibleFaces    []faceData
	disabledFacePointVectors [][]int
	faceList                []int // deque<int> — we use a slice + index
}

// NewQuickHull initializes a QuickHull for the given point cloud.
// The points slice is borrowed (not copied) until BuildMesh
// completes.
func NewQuickHull(points []geom.Vec3) *QuickHull {
	return &QuickHull{originalVertexData: points}
}

// BuildMesh runs the algorithm and returns the convex hull as
// (halfedges, vertices). The halfedges are written in face-major
// order — every consecutive triple belongs to the same face, with
// startVert/endVert/pairedHalfedge wired consistently. The
// vertices contain only the points actually used (interior input
// points are dropped).
//
// eps is the minimum plane-distance threshold (defaults to
// DefaultEpsilon). Pass it scaled by the point cloud's scale if
// you want a unit-scale eps — but BuildMesh already scales internally.
func (q *QuickHull) BuildMesh(eps float64) ([]Halfedge, []geom.Vec3) {
	if len(q.originalVertexData) == 0 {
		return nil, nil
	}

	q.extremeValues = q.getExtremeValues()
	q.scale = q.getScale(q.extremeValues)
	q.epsilon = eps * q.scale
	q.epsilonSquared = q.epsilon * q.epsilon
	q.planar = false

	q.createConvexHalfedgeMesh()

	if q.planar {
		// Restore the original coordinate of the extra-point slot
		// that was added by setupInitialTetrahedron's planar branch.
		// Mirrors src/quickhull.cpp:249-250.
		if len(q.planarPointCloudTemp) > 0 {
			q.planarPointCloudTemp[len(q.planarPointCloudTemp)-1] = q.planarPointCloudTemp[0]
		}
	}

	return q.compactMesh()
}

// compactMesh produces the final (halfedges, vertices) pair from
// the in-progress mesh, dropping disabled faces and unused
// vertices and renumbering halfedges/vertices into contiguous
// indices. Mirrors src/quickhull.cpp:254-318.
//
// The output halfedge layout is face-major: every consecutive
// triple is one face, with startVert wired from the previous
// halfedge's endVert in the triple. This is what
// `Halfedges(std::move(...))` consumes in Impl::Hull.
func (q *QuickHull) compactMesh() ([]Halfedge, []geom.Vec3) {
	nHE := len(q.mesh.halfedges)
	outHE := make([]Halfedge, 0, nHE)
	mapping := make([]int, nHE) // old halfedge index → new
	// First pass: for each face (one halfedge per face is enough —
	// we follow halfedgeNext to emit the other two), append three
	// halfedges in face-major order.
	for i := 0; i < nHE; i++ {
		if q.mesh.halfedges[i].PairedHalfedge < 0 {
			continue
		}
		faceIdx := q.mesh.halfedgeToFace[i]
		if q.mesh.faces[faceIdx].isDisabled() {
			continue
		}
		// Only emit when we hit the FIRST halfedge of the face —
		// the face stack pointer (.he) marks it. Otherwise we'd
		// emit each face three times.
		if q.mesh.faces[faceIdx].he != i {
			continue
		}
		base := len(outHE)
		// Emit halfedges in halfedgeNext walk order.
		i0 := i
		i1 := q.mesh.halfedgeNext[i0]
		i2 := q.mesh.halfedgeNext[i1]
		he0 := q.mesh.halfedges[i0]
		he1 := q.mesh.halfedges[i1]
		he2 := q.mesh.halfedges[i2]
		// Set startVert (computed from the previous halfedge's
		// endVert in the triple) — mirrors src/quickhull.cpp:283-285.
		he0.StartVert = he2.EndVert
		he1.StartVert = he0.EndVert
		he2.StartVert = he1.EndVert
		outHE = append(outHE, he0, he1, he2)
		mapping[i0] = base + 0
		mapping[i1] = base + 1
		mapping[i2] = base + 2
	}

	// Fix paired indices via the mapping table.
	for k := range outHE {
		outHE[k].PairedHalfedge = mapping[outHE[k].PairedHalfedge]
	}

	// Remove unused input vertices: count references, exclusive
	// scan, gather.
	nIn := len(q.originalVertexData)
	counts := make([]int, nIn+1)
	for k := range outHE {
		counts[outHE[k].StartVert]++
	}
	saturate := func(c int) int {
		if c > 0 {
			return 1
		}
		return 0
	}
	// Exclusive scan with saturate in place.
	prev := 0
	for i := 0; i <= nIn; i++ {
		curr := saturate(counts[i])
		counts[i] = prev
		prev += curr
	}
	verts := make([]geom.Vec3, counts[nIn])
	for i := 0; i < nIn; i++ {
		if counts[i+1]-counts[i] > 0 {
			verts[counts[i]] = q.originalVertexData[i]
		}
	}
	for k := range outHE {
		outHE[k].StartVert = counts[outHE[k].StartVert]
		outHE[k].EndVert = counts[outHE[k].EndVert]
	}
	return outHE, verts
}

// ============================================================
// Private helpers — mirror src/quickhull.cpp:574-826.

// getExtremeValues returns 6 indices: (max x, min x, max y, min y,
// max z, min z). Mirrors src/quickhull.cpp:574-605.
func (q *QuickHull) getExtremeValues() [6]int {
	var out [6]int
	v0 := q.originalVertexData[0]
	extreme := [6]float64{v0.X, v0.X, v0.Y, v0.Y, v0.Z, v0.Z}
	for i := 1; i < len(q.originalVertexData); i++ {
		p := q.originalVertexData[i]
		if p.X > extreme[0] {
			extreme[0] = p.X
			out[0] = i
		} else if p.X < extreme[1] {
			extreme[1] = p.X
			out[1] = i
		}
		if p.Y > extreme[2] {
			extreme[2] = p.Y
			out[2] = i
		} else if p.Y < extreme[3] {
			extreme[3] = p.Y
			out[3] = i
		}
		if p.Z > extreme[4] {
			extreme[4] = p.Z
			out[4] = i
		} else if p.Z < extreme[5] {
			extreme[5] = p.Z
			out[5] = i
		}
	}
	return out
}

// getScale returns the largest absolute coordinate among the 6
// extreme points. Used to scale epsilon. Mirrors C++ at
// src/quickhull.cpp:634-646: read the axis matching the extreme
// index (i/2 selects x/y/z).
func (q *QuickHull) getScale(extremes [6]int) float64 {
	s := 0.0
	for i := 0; i < 6; i++ {
		p := q.originalVertexData[extremes[i]]
		var v float64
		switch i / 2 {
		case 0:
			v = math.Abs(p.X)
		case 1:
			v = math.Abs(p.Y)
		case 2:
			v = math.Abs(p.Z)
		}
		if v > s {
			s = v
		}
	}
	return s
}

// reorderHorizonEdges rearranges horizonEdges in place so the
// edges form a closed loop. Returns true on success. Mirrors C++
// at src/quickhull.cpp:607-632.
func (q *QuickHull) reorderHorizonEdges(horizonEdges []int) bool {
	n := len(horizonEdges)
	for i := 0; i+1 < n; i++ {
		endVertCheck := q.mesh.halfedges[horizonEdges[i]].EndVert
		found := false
		for j := i + 1; j < n; j++ {
			beginVert := q.mesh.halfedges[q.mesh.halfedges[horizonEdges[j]].PairedHalfedge].EndVert
			if beginVert == endVertCheck {
				horizonEdges[i+1], horizonEdges[j] = horizonEdges[j], horizonEdges[i+1]
				found = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

// addPointToFace assigns pointIndex to f if it lies on the
// positive side of f's plane by more than epsilon*|N|. Returns
// true iff added. Also updates f.mostDistantPoint/dist. Mirrors
// C++ at src/quickhull.cpp:810-826.
func (q *QuickHull) addPointToFace(f *face, pointIndex int) bool {
	D := signedDistanceToPlane(q.originalVertexData[pointIndex], f.P)
	if D > 0 && D*D > q.epsilonSquared*f.P.SqrNLen {
		if f.pointsOnPositiveSide == nil {
			f.pointsOnPositiveSide = make([]int, 0, 16)
		}
		f.pointsOnPositiveSide = append(f.pointsOnPositiveSide, pointIndex)
		if D > f.mostDistantPointDist {
			f.mostDistantPointDist = D
			f.mostDistantPoint = pointIndex
		}
		return true
	}
	return false
}

// setupInitialTetrahedron picks 4 extreme points to form the
// starting tetrahedron, computes each face's plane, and seeds
// the points-on-positive-side lists. Mirrors C++ at
// src/quickhull.cpp:648-790.
func (q *QuickHull) setupInitialTetrahedron() {
	n := len(q.originalVertexData)

	// Degenerate case: <= 4 input points.
	if n <= 4 {
		if n < 4 {
			q.planarPointCloudTemp = append(q.planarPointCloudTemp[:0], q.originalVertexData...)
			for len(q.planarPointCloudTemp) < 4 {
				q.planarPointCloudTemp = append(q.planarPointCloudTemp,
					q.planarPointCloudTemp[len(q.planarPointCloudTemp)-1])
			}
			q.originalVertexData = q.planarPointCloudTemp
		}
		v := [4]int{0, 1, 2, 3}
		N := triangleNormal(q.originalVertexData[v[0]], q.originalVertexData[v[1]], q.originalVertexData[v[2]])
		plane := NewPlane(N, q.originalVertexData[v[0]])
		if plane.IsPointOnPositiveSide(q.originalVertexData[v[3]]) {
			v[0], v[1] = v[1], v[0]
		}
		q.mesh.setup(v[0], v[1], v[2], v[3])
		return
	}

	// Find the two extreme points that are most distant from each
	// other.
	maxD := q.epsilonSquared
	var selectedA, selectedB int
	for i := 0; i < 6; i++ {
		for j := i + 1; j < 6; j++ {
			d := squaredDistance(q.originalVertexData[q.extremeValues[i]], q.originalVertexData[q.extremeValues[j]])
			if d > maxD {
				maxD = d
				selectedA = q.extremeValues[i]
				selectedB = q.extremeValues[j]
			}
		}
	}
	if maxD == q.epsilonSquared {
		// Degenerate: single-point cloud.
		q.mesh.setup(0, 1, 2, 3)
		return
	}

	// Find the most distant point to the line between the two
	// selected extreme points.
	r := NewRay(q.originalVertexData[selectedA],
		q.originalVertexData[selectedB].Sub(q.originalVertexData[selectedA]))
	maxD = q.epsilonSquared
	maxI := math.MaxInt
	for i := 0; i < n; i++ {
		d := squaredDistanceBetweenPointAndRay(q.originalVertexData[i], r)
		if d > maxD {
			maxD = d
			maxI = i
		}
	}
	if maxD == q.epsilonSquared {
		// Degenerate: 1D point cloud. Pick four arbitrary distinct
		// indices.
		firstPoint := selectedA
		secondPoint := selectedB
		thirdPoint := 0
		for thirdPoint == firstPoint || thirdPoint == secondPoint {
			thirdPoint++
		}
		fourthPoint := thirdPoint + 1
		for fourthPoint == firstPoint || fourthPoint == secondPoint {
			fourthPoint++
		}
		q.mesh.setup(firstPoint, secondPoint, thirdPoint, fourthPoint)
		return
	}

	baseTri := [3]int{selectedA, selectedB, maxI}
	bv := [3]geom.Vec3{
		q.originalVertexData[baseTri[0]],
		q.originalVertexData[baseTri[1]],
		q.originalVertexData[baseTri[2]],
	}

	// Find the 4th tetrahedron vertex: farthest point from the
	// base-triangle plane.
	maxD = q.epsilon
	maxI = 0
	N := triangleNormal(bv[0], bv[1], bv[2])
	trianglePlane := NewPlane(N, bv[0])
	for i := 0; i < n; i++ {
		d := math.Abs(signedDistanceToPlane(q.originalVertexData[i], trianglePlane))
		if d > maxD {
			maxD = d
			maxI = i
		}
	}
	if maxD == q.epsilon {
		// All points appear to lie on a 2D subspace. Add an extra
		// point off the plane so the hull has volume; we'll snap it
		// back at the end.
		q.planar = true
		N1 := triangleNormal(bv[1], bv[2], bv[0])
		q.planarPointCloudTemp = append(q.planarPointCloudTemp[:0], q.originalVertexData...)
		extra := N1.Add(q.originalVertexData[0])
		q.planarPointCloudTemp = append(q.planarPointCloudTemp, extra)
		maxI = len(q.planarPointCloudTemp) - 1
		q.originalVertexData = q.planarPointCloudTemp
	}

	// Enforce CCW orientation.
	if trianglePlane.IsPointOnPositiveSide(q.originalVertexData[maxI]) {
		baseTri[0], baseTri[1] = baseTri[1], baseTri[0]
	}

	q.mesh.setup(baseTri[0], baseTri[1], baseTri[2], maxI)
	// Compute each face's plane.
	for fi := range q.mesh.faces {
		f := &q.mesh.faces[fi]
		v := q.mesh.vertexIndicesOfFace(f)
		N1 := triangleNormal(q.originalVertexData[v[0]], q.originalVertexData[v[1]], q.originalVertexData[v[2]])
		f.P = NewPlane(N1, q.originalVertexData[v[0]])
	}

	// Assign every input vertex to the first face whose plane it
	// lies on the positive side of.
	for i := 0; i < n; i++ {
		for fi := range q.mesh.faces {
			if q.addPointToFace(&q.mesh.faces[fi], i) {
				break
			}
		}
	}
}

// createConvexHalfedgeMesh runs the main quickhull iteration.
// Mirrors src/quickhull.cpp:321-568.
func (q *QuickHull) createConvexHalfedgeMesh() {
	q.visibleFaces = q.visibleFaces[:0]
	q.horizonEdgesData = q.horizonEdgesData[:0]
	q.possiblyVisibleFaces = q.possiblyVisibleFaces[:0]

	q.setupInitialTetrahedron()

	q.faceList = q.faceList[:0]
	for i := 0; i < 4; i++ {
		f := &q.mesh.faces[i]
		if len(f.pointsOnPositiveSide) > 0 {
			q.faceList = append(q.faceList, i)
			f.inFaceStack = true
		}
	}

	iter := uint64(0)
	for len(q.faceList) > 0 {
		iter++
		if iter == math.MaxUint64 {
			iter = 0
		}

		topFaceIndex := q.faceList[0]
		q.faceList = q.faceList[1:]
		tf := &q.mesh.faces[topFaceIndex]
		tf.inFaceStack = false

		if tf.pointsOnPositiveSide == nil || tf.isDisabled() {
			continue
		}

		activePointIndex := tf.mostDistantPoint
		activePoint := q.originalVertexData[activePointIndex]

		q.horizonEdgesData = q.horizonEdgesData[:0]
		q.possiblyVisibleFaces = q.possiblyVisibleFaces[:0]
		q.visibleFaces = q.visibleFaces[:0]
		q.possiblyVisibleFaces = append(q.possiblyVisibleFaces, faceData{topFaceIndex, -1})

		for len(q.possiblyVisibleFaces) > 0 {
			fd := q.possiblyVisibleFaces[len(q.possiblyVisibleFaces)-1]
			q.possiblyVisibleFaces = q.possiblyVisibleFaces[:len(q.possiblyVisibleFaces)-1]
			pvf := &q.mesh.faces[fd.faceIndex]

			if pvf.visibilityCheckedOnIteration == iter {
				if pvf.isVisibleFaceOnCurrentIteration {
					continue
				}
			} else {
				P := pvf.P
				pvf.visibilityCheckedOnIteration = iter
				d := P.N.Dot(activePoint) + P.D
				if d > 0 {
					pvf.isVisibleFaceOnCurrentIteration = true
					pvf.horizonEdgesOnCurrentIteration = 0
					q.visibleFaces = append(q.visibleFaces, fd.faceIndex)
					heIndices := q.mesh.halfEdgeIndicesOfFace(pvf)
					for _, heIndex := range heIndices {
						if q.mesh.halfedges[heIndex].PairedHalfedge != fd.enteredFromHalfedge {
							pairFace := q.mesh.halfedgeToFace[q.mesh.halfedges[heIndex].PairedHalfedge]
							q.possiblyVisibleFaces = append(q.possiblyVisibleFaces,
								faceData{pairFace, heIndex})
						}
					}
					continue
				}
			}

			// The face is not visible; the halfedge we entered from
			// is part of the horizon edge.
			pvf.isVisibleFaceOnCurrentIteration = false
			q.horizonEdgesData = append(q.horizonEdgesData, fd.enteredFromHalfedge)
			ownerFace := q.mesh.halfedgeToFace[fd.enteredFromHalfedge]
			heIndices := q.mesh.halfEdgeIndicesOfFace(&q.mesh.faces[ownerFace])
			var ind int
			if heIndices[0] == fd.enteredFromHalfedge {
				ind = 0
			} else if heIndices[1] == fd.enteredFromHalfedge {
				ind = 1
			} else {
				ind = 2
			}
			q.mesh.faces[ownerFace].horizonEdgesOnCurrentIteration |= uint8(1 << ind)
		}
		horizonEdgeCount := len(q.horizonEdgesData)

		// Try to order horizon edges into a loop.
		if !q.reorderHorizonEdges(q.horizonEdgesData) {
			q.failedHorizonEdges++
			// Remove the activePointIndex from this face's points
			// and try the next iteration.
			changeFlag := 0
			pts := tf.pointsOnPositiveSide
			for idx := 0; idx < len(pts); idx++ {
				if pts[idx] == activePointIndex {
					changeFlag = 1
				} else if changeFlag == 1 {
					changeFlag = 2
					pts[idx-1] = pts[idx]
				}
			}
			if changeFlag == 1 {
				tf.pointsOnPositiveSide = pts[:len(pts)-1]
			}
			if len(tf.pointsOnPositiveSide) == 0 {
				tf.pointsOnPositiveSide = nil
			}
			continue
		}

		// Mark all non-horizon halfedges of visible faces as
		// disabled (for reuse) and stash their point lists for
		// re-distribution.
		q.newFaceIndices = q.newFaceIndices[:0]
		q.newHalfedgeIndices = q.newHalfedgeIndices[:0]
		q.disabledFacePointVectors = q.disabledFacePointVectors[:0]
		disableCounter := 0
		for _, faceIndex := range q.visibleFaces {
			df := &q.mesh.faces[faceIndex]
			heIndices := q.mesh.halfEdgeIndicesOfFace(df)
			for j := 0; j < 3; j++ {
				if df.horizonEdgesOnCurrentIteration&(1<<j) == 0 {
					if disableCounter < horizonEdgeCount*2 {
						q.newHalfedgeIndices = append(q.newHalfedgeIndices, heIndices[j])
						disableCounter++
					} else {
						q.mesh.disableHalfedge(heIndices[j])
					}
				}
			}
			pts := q.mesh.disableFace(faceIndex)
			if len(pts) > 0 {
				q.disabledFacePointVectors = append(q.disabledFacePointVectors, pts)
			}
		}
		if disableCounter < horizonEdgeCount*2 {
			needed := horizonEdgeCount*2 - disableCounter
			for i := 0; i < needed; i++ {
				q.newHalfedgeIndices = append(q.newHalfedgeIndices, q.mesh.addHalfedge())
			}
		}

		// Create new faces around each horizon edge.
		for i := 0; i < horizonEdgeCount; i++ {
			AB := q.horizonEdgesData[i]
			heVerts := q.mesh.vertexIndicesOfHalfEdge(q.mesh.halfedges[AB])
			A := heVerts[0]
			B := heVerts[1]
			C := activePointIndex

			newFaceIndex := q.mesh.addFace()
			q.newFaceIndices = append(q.newFaceIndices, newFaceIndex)

			CA := q.newHalfedgeIndices[2*i+0]
			BC := q.newHalfedgeIndices[2*i+1]

			q.mesh.halfedgeNext[AB] = BC
			q.mesh.halfedgeNext[BC] = CA
			q.mesh.halfedgeNext[CA] = AB
			q.mesh.halfedgeToFace[BC] = newFaceIndex
			q.mesh.halfedgeToFace[CA] = newFaceIndex
			q.mesh.halfedgeToFace[AB] = newFaceIndex
			q.mesh.halfedges[CA].EndVert = A
			q.mesh.halfedges[BC].EndVert = C

			newFace := &q.mesh.faces[newFaceIndex]
			N := triangleNormal(q.originalVertexData[A], q.originalVertexData[B], activePoint)
			newFace.P = NewPlane(N, activePoint)
			newFace.he = AB

			var caPair, bcPair int
			if i > 0 {
				caPair = q.newHalfedgeIndices[i*2-1]
			} else {
				caPair = q.newHalfedgeIndices[2*horizonEdgeCount-1]
			}
			bcPair = q.newHalfedgeIndices[((i+1)*2)%(horizonEdgeCount*2)]
			q.mesh.halfedges[CA].PairedHalfedge = caPair
			q.mesh.halfedges[BC].PairedHalfedge = bcPair
		}

		// Re-distribute the points that were on disabled faces to
		// the new faces.
		for _, pts := range q.disabledFacePointVectors {
			for _, point := range pts {
				if point == activePointIndex {
					continue
				}
				for j := 0; j < horizonEdgeCount; j++ {
					if q.addPointToFace(&q.mesh.faces[q.newFaceIndices[j]], point) {
						break
					}
				}
			}
		}

		// Push new faces with assigned points back onto the stack.
		for _, fi := range q.newFaceIndices {
			nf := &q.mesh.faces[fi]
			if len(nf.pointsOnPositiveSide) > 0 && !nf.inFaceStack {
				q.faceList = append(q.faceList, fi)
				nf.inFaceStack = true
			}
		}
	}
}
