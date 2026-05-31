package boolean

import (
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/stdcpp/multimap"
	"github.com/firstlayer-xyz/manifold/go/internal/triangulate"
)

// assembleHalfedges is the Go port of AssembleHalfedges (face_op.cpp:41): walk
// the halfedge range [lo, hi) of one face into vertex-index loops, using the
// halfedge indices (startHalfedgeIdx + local index) rather than vertex indices.
func assembleHalfedges(faceHalfedge []Halfedge, lo, hi, startHalfedgeIdx int) [][]int {
	vertEdge := multimap.New[int, int]()
	for e := lo; e < hi; e++ {
		vertEdge.Emplace(faceHalfedge[e].StartVert, e-lo)
	}

	var polys [][]int
	startEdge := 0
	thisEdge := startEdge
	for {
		if thisEdge == startEdge {
			if vertEdge.Empty() {
				break
			}
			startEdge = vertEdge.BeginValue()
			thisEdge = startEdge
			polys = append(polys, []int{})
		}
		polys[len(polys)-1] = append(polys[len(polys)-1], startHalfedgeIdx+thisEdge)
		key := faceHalfedge[lo+thisEdge].EndVert
		next, _ := vertEdge.FindFirst(key) // DEBUG_ASSERT found (non-manifold edge)
		thisEdge = next
		vertEdge.EraseFirst(key)
	}
	return polys
}

// projectPolygons is the Go port of ProjectPolygons (face_op.cpp:72): attach the
// projected (2D) vertex positions to each indexed polygon, keying each PolyVert
// by its halfedge index.
func projectPolygons(polys [][]int, faceHalfedge []Halfedge, vertPos []geom.Vec3, projection geom.Mat2x3) triangulate.PolygonsIdx {
	var polygons triangulate.PolygonsIdx
	for _, poly := range polys {
		polygons = append(polygons, triangulate.SimplePolygonIdx{})
		for _, edge := range poly {
			polygons[len(polygons)-1] = append(polygons[len(polygons)-1], triangulate.PolyVert{
				Pos: projection.MulVec3(vertPos[faceHalfedge[edge].StartVert]),
				Idx: edge,
			})
		}
	}
	return polygons
}

// writeLocalTriangles is the Go port of WriteLocalTriangles (face_op.cpp:86): emit
// the (at most two) triangles of a tri/quad face directly into the output
// halfedges, pairing the interior diagonal and recording contour edges in
// contour2Tri. triangles holds triples of global edge indices.
func writeLocalTriangles(output halfedges, contour2Tri []int, faceHalfedge []Halfedge, firstTri int, triangles [][3]int, numTri int) {
	// DEBUG_ASSERT(numTri <= 2): local face path only handles tris/quads.
	var localEdges [6][3]int
	firstOut := 3 * firstTri
	numEdge := 0
	for tri := 0; tri < numTri; tri++ {
		for _, i := range []int{0, 1, 2} {
			out := firstOut + numEdge
			start := triangles[tri][i]
			end := triangles[tri][Next3(i)]
			localEdges[numEdge] = [3]int{start, end, out}
			output.SetStart(out, faceHalfedge[start].StartVert)
			output.SetProp(out, faceHalfedge[start].PropVert)
			output.SetPair(out, -1)
			numEdge++
		}
	}

	for i := 0; i < numEdge; i++ {
		edge := localEdges[i]
		pair := -1
		for j := 0; j < numEdge; j++ {
			if localEdges[j][0] == edge[1] && localEdges[j][1] == edge[0] {
				pair = localEdges[j][2]
				break
			}
		}
		if pair >= 0 {
			output.SetPair(edge[2], pair)
		} else {
			contour2Tri[edge[0]] = edge[2]
		}
	}
}

// writeGeneralTriangulation is the Go port of WriteGeneralTriangulation
// (face_op.cpp:124): copy a general HalfedgeTriangulation's interior halfedges
// into the output (remapping the triangulation's contour-relative pair indices to
// output indices) and record each contour edge's triangle halfedge in contour2Tri.
func writeGeneralTriangulation(output halfedges, contour2Tri []int, faceHalfedge []Halfedge, firstTri int, triangulation *triangulate.HalfedgeTriangulation) {
	firstOut := 3 * firstTri
	numTriHalfedge := 3 * triangulation.NumTri()
	for local := 0; local < numTriHalfedge; local++ {
		out := firstOut + local
		edge := triangulation.Halfedges[triangulation.ContourEnd+local]
		output.SetStart(out, faceHalfedge[edge.StartVert].StartVert)
		output.SetProp(out, faceHalfedge[edge.StartVert].PropVert)
		if edge.PairedHalfedge >= triangulation.ContourEnd {
			output.SetPair(out, firstOut+edge.PairedHalfedge-triangulation.ContourEnd)
		} else {
			output.SetPair(out, -1)
		}
	}

	for contour := 0; contour < triangulation.ContourEnd; contour++ {
		edge := triangulation.Halfedges[contour]
		if edge.PairedHalfedge < 0 {
			continue
		}
		// DEBUG_ASSERT: contour paired to interior (>= ContourEnd), boundary in range.
		boundary := edge.EndVert
		contour2Tri[boundary] = firstOut + edge.PairedHalfedge - triangulation.ContourEnd
	}
}

// writeTriRefs is the Go port of WriteTriRefs (face_op.cpp:164): fill the per-tri
// normal and reference for a face's triangles.
func writeTriRefs(triNormal []geom.Vec3, triRef []TriRef, firstTri, numTri int, normal geom.Vec3, ref TriRef) {
	for tri := 0; tri < numTri; tri++ {
		triNormal[firstTri+tri] = normal
		triRef[firstTri+tri] = ref
	}
}

// face2Tri is the Go port of Manifold::Impl::Face2Tri (face_op.cpp:187) as a pure
// function (the C++ method's only call site is Boolean3::Result). It triangulates
// the general faces described by faceEdge + faceHalfedge into a triangle halfedge
// mesh, returning the output halfedges (start/pair/prop arrays; End is derived),
// the per-tri normals, and the per-tri references. SEQ path (the #else branches).
func face2Tri(faceEdge []int, faceHalfedge []Halfedge, halfedgeRef []TriRef, vertPos, faceNormal []geom.Vec3, epsilon float64, allowConvex bool) (halfedges, []geom.Vec3, []TriRef) {
	contour2Tri := make([]int, len(faceHalfedge))
	for i := range contour2Tri {
		contour2Tri[i] = -1
	}

	generalTriangulation := func(face int) *triangulate.HalfedgeTriangulation {
		normal := faceNormal[face]
		projection := geom.GetAxisAlignedProjection(normal)
		polys := projectPolygons(
			assembleHalfedges(faceHalfedge, faceEdge[face], faceEdge[face+1], faceEdge[face]),
			faceHalfedge, vertPos, projection)
		return triangulate.TriangulateIdxHalfedges(polys, epsilon, allowConvex)
	}

	// Precompute the triangle count per face, triangulating complex (>4 edge)
	// faces up front. triOffset has len(faceEdge) entries; the last stays 0.
	triOffset := make([]int, len(faceEdge))
	results := map[int]*triangulate.HalfedgeTriangulation{}
	for face := 0; face < len(faceEdge)-1; face++ {
		numEdge := faceEdge[face+1] - faceEdge[face]
		if numEdge == 0 {
			triOffset[face] = 0
			continue
		}
		// DEBUG_ASSERT(numEdge >= 3): face has less than three edges.
		triOffset[face] = numEdge - 2
		if numEdge > 4 {
			triangulation := generalTriangulation(face)
			triOffset[face] = triangulation.NumTri()
			results[face] = triangulation
		}
	}

	// exclusive_scan in place: triOffset[face] becomes the first tri index of
	// that face; triOffset.back() becomes the total tri count.
	acc := 0
	for i := range triOffset {
		next := acc + triOffset[i]
		triOffset[i] = acc
		acc = next
	}
	total := triOffset[len(triOffset)-1]

	out := halfedges{
		starts:   make([]int32, 3*total),
		pairs:    make([]int32, 3*total),
		propVert: make([]int32, 3*total),
	}
	triNormal := make([]geom.Vec3, total)
	triRef := make([]TriRef, total)

	outputFace := func(face, firstTri int, general *triangulate.HalfedgeTriangulation) {
		firstEdge := faceEdge[face]
		lastEdge := faceEdge[face+1]
		numEdge := lastEdge - firstEdge
		if numEdge == 0 {
			return
		}
		// DEBUG_ASSERT(numEdge >= 3).
		normal := faceNormal[face]
		numTri := numEdge - 2

		switch {
		case numEdge == 3: // Single triangle
			triEdge := [3]int{firstEdge, firstEdge + 1, firstEdge + 2}
			tri := [3]int{faceHalfedge[firstEdge].StartVert, faceHalfedge[firstEdge+1].StartVert, faceHalfedge[firstEdge+2].StartVert}
			ends := [3]int{faceHalfedge[firstEdge].EndVert, faceHalfedge[firstEdge+1].EndVert, faceHalfedge[firstEdge+2].EndVert}
			if ends[0] == tri[2] {
				triEdge[1], triEdge[2] = triEdge[2], triEdge[1]
				tri[1], tri[2] = tri[2], tri[1]
				ends[1], ends[2] = ends[2], ends[1]
			}
			// DEBUG_ASSERT(ends form a triangle).
			writeLocalTriangles(out, contour2Tri, faceHalfedge, firstTri, [][3]int{triEdge}, 1)
		case numEdge == 4: // Pair of triangles
			projection := geom.GetAxisAlignedProjection(normal)
			triCCW := func(t [3]int) bool {
				return geom.CCW(
					projection.MulVec3(vertPos[faceHalfedge[t[0]].StartVert]),
					projection.MulVec3(vertPos[faceHalfedge[t[1]].StartVert]),
					projection.MulVec3(vertPos[faceHalfedge[t[2]].StartVert]),
					epsilon) >= 0
			}

			quad := assembleHalfedges(faceHalfedge, faceEdge[face], faceEdge[face+1], faceEdge[face])[0]

			// la::mat<int,3,2>: two columns, each a triangle of quad indices.
			tris := [2][2][3]int{
				{{quad[0], quad[1], quad[2]}, {quad[0], quad[2], quad[3]}},
				{{quad[1], quad[2], quad[3]}, {quad[0], quad[1], quad[3]}},
			}

			choice := 0
			if !(triCCW(tris[0][0]) && triCCW(tris[0][1])) {
				choice = 1
			} else if triCCW(tris[1][0]) && triCCW(tris[1][1]) {
				diag0 := vertPos[faceHalfedge[quad[0]].StartVert].Sub(vertPos[faceHalfedge[quad[2]].StartVert])
				diag1 := vertPos[faceHalfedge[quad[1]].StartVert].Sub(vertPos[faceHalfedge[quad[3]].StartVert])
				if diag0.Dot(diag0) > diag1.Dot(diag1) {
					choice = 1
				}
			}

			writeLocalTriangles(out, contour2Tri, faceHalfedge, firstTri, [][3]int{tris[choice][0], tris[choice][1]}, 2)
		default: // General triangulation
			// DEBUG_ASSERT(general != nil).
			numTri = general.NumTri()
			writeGeneralTriangulation(out, contour2Tri, faceHalfedge, firstTri, general)
		}

		writeTriRefs(triNormal, triRef, firstTri, numTri, normal, halfedgeRef[firstEdge])
	}

	for face := 0; face < len(faceEdge)-1; face++ {
		outputFace(face, triOffset[face], results[face])
	}

	// Pair up the triangulated boundary edges with their neighbours' triangles.
	for edge := 0; edge < len(faceHalfedge); edge++ {
		triEdge := contour2Tri[edge]
		if triEdge < 0 {
			continue
		}
		pair := faceHalfedge[edge].PairedHalfedge
		if pair < 0 {
			continue
		}
		pairTri := contour2Tri[pair]
		// DEBUG_ASSERT(pairTri >= 0): boundary edge did not triangulate with its pair.
		out.SetPair(triEdge, pairTri)
	}
	return out, triNormal, triRef
}
