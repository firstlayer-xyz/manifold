package quickhull

// face is the C++ MeshBuilder::Face. Stores:
//   - he: index of one halfedge bordering this face (-1 when disabled).
//   - P: plane of the face (normal + signed distance).
//   - mostDistantPoint / mostDistantPointDist: cached farthest
//     point on the positive side of the plane.
//   - visibilityCheckedOnIteration: iteration counter for the
//     visibility-flood-fill phase.
//   - isVisibleFaceOnCurrentIteration: 1 bit.
//   - inFaceStack: 1 bit.
//   - horizonEdgesOnCurrentIteration: 3 bits, one per halfedge of
//     this face — set when that halfedge is part of the iteration's
//     horizon loop.
//   - pointsOnPositiveSide: indices into the input vertex slice of
//     points assigned to this face. Nil when none.
//
// Bit fields collapse to a single uint8 in C++; we use bool/int
// fields in Go for clarity (the memory overhead is negligible
// since the face count is small).
type face struct {
	he                              int
	P                               Plane
	mostDistantPoint                int
	mostDistantPointDist            float64
	visibilityCheckedOnIteration    uint64
	isVisibleFaceOnCurrentIteration bool
	inFaceStack                     bool
	horizonEdgesOnCurrentIteration  uint8
	pointsOnPositiveSide            []int
}

func newFace(he int) face {
	return face{he: he}
}

func (f *face) disable()         { f.he = -1 }
func (f *face) isDisabled() bool { return f.he == -1 }

// meshBuilder mirrors C++ MeshBuilder (src/quickhull.h:119-198).
//
// Faces and halfedges are stored in slices; disabled entries are
// not actually removed — their indices are pushed onto
// disabledFaces / disabledHalfedges so they can be reused by the
// next allocation. This pattern minimizes slice growth during the
// hull iteration.
type meshBuilder struct {
	faces             []face
	halfedges         []Halfedge
	halfedgeToFace    []int
	halfedgeNext      []int
	disabledFaces     []int
	disabledHalfedges []int
}

// addFace returns the index of a fresh (or recycled) face slot.
// Mirrors C++ MeshBuilder::addFace.
func (m *meshBuilder) addFace() int {
	if n := len(m.disabledFaces); n > 0 {
		idx := m.disabledFaces[n-1]
		m.disabledFaces = m.disabledFaces[:n-1]
		// Reset the recycled face — equivalent to C++ setting
		// mostDistantPointDist=0 and overwriting other fields as
		// the algorithm uses them.
		m.faces[idx] = face{he: 0}
		return idx
	}
	m.faces = append(m.faces, face{})
	return len(m.faces) - 1
}

// addHalfedge returns the index of a fresh (or recycled) halfedge
// slot. The slot's fields are NOT cleared on recycle — C++ doesn't
// either; callers always overwrite immediately after.
func (m *meshBuilder) addHalfedge() int {
	if n := len(m.disabledHalfedges); n > 0 {
		idx := m.disabledHalfedges[n-1]
		m.disabledHalfedges = m.disabledHalfedges[:n-1]
		return idx
	}
	m.halfedges = append(m.halfedges, Halfedge{})
	m.halfedgeToFace = append(m.halfedgeToFace, 0)
	m.halfedgeNext = append(m.halfedgeNext, 0)
	return len(m.halfedges) - 1
}

// disableFace marks a face slot recyclable and returns its
// pointsOnPositiveSide (transferring ownership). Mirrors C++
// MeshBuilder::disableFace.
func (m *meshBuilder) disableFace(idx int) []int {
	f := &m.faces[idx]
	f.disable()
	m.disabledFaces = append(m.disabledFaces, idx)
	pts := f.pointsOnPositiveSide
	f.pointsOnPositiveSide = nil
	return pts
}

// disableHalfedge marks a halfedge slot recyclable and clears its
// pair link.
func (m *meshBuilder) disableHalfedge(idx int) {
	m.halfedges[idx].PairedHalfedge = -1
	m.disabledHalfedges = append(m.disabledHalfedges, idx)
}

// setup creates the initial tetrahedron with verts (a, b, c, d).
// The dot product of AB with the normal of triangle ABC should be
// negative (i.e. D is on the negative side of ABC's plane); callers
// ensure this via swap when necessary. Mirrors src/quickhull.cpp:89-155.
//
// The halfedge layout is the canonical tetra layout:
//
//	face 0 (ABC):  AB(0)  BC(1)  CA(2)
//	face 1 (ACD):  AC(3)  CD(4)  DA(5)
//	face 2 (ABD-): BA(6)  AD(7)  DB(8)
//	face 3 (CBD-): CB(9)  BD(10) DC(11)
func (m *meshBuilder) setup(a, b, c, d int) {
	m.faces = m.faces[:0]
	m.halfedges = m.halfedges[:0]
	m.halfedgeToFace = m.halfedgeToFace[:0]
	m.halfedgeNext = m.halfedgeNext[:0]
	m.disabledFaces = m.disabledFaces[:0]
	m.disabledHalfedges = m.disabledHalfedges[:0]

	// Halfedges 0-11, layout matches src/quickhull.cpp:102-148.
	add := func(endVert, pair, next, faceID int) {
		m.halfedges = append(m.halfedges, Halfedge{EndVert: endVert, PairedHalfedge: pair})
		m.halfedgeToFace = append(m.halfedgeToFace, faceID)
		m.halfedgeNext = append(m.halfedgeNext, next)
	}
	add(b, 6, 1, 0)   // 0  AB
	add(c, 9, 2, 0)   // 1  BC
	add(a, 3, 0, 0)   // 2  CA
	add(c, 2, 4, 1)   // 3  AC
	add(d, 11, 5, 1)  // 4  CD
	add(a, 7, 3, 1)   // 5  DA
	add(a, 0, 7, 2)   // 6  BA
	add(d, 5, 8, 2)   // 7  AD
	add(b, 10, 6, 2)  // 8  DB
	add(b, 1, 10, 3)  // 9  CB
	add(d, 8, 11, 3)  // 10 BD
	add(c, 4, 9, 3)   // 11 DC

	m.faces = append(m.faces, newFace(0))
	m.faces = append(m.faces, newFace(3))
	m.faces = append(m.faces, newFace(6))
	m.faces = append(m.faces, newFace(9))
}

// vertexIndicesOfFace returns the 3 endVert indices of f's
// halfedges, in halfedgeNext walk order.
func (m *meshBuilder) vertexIndicesOfFace(f *face) [3]int {
	var v [3]int
	idx := f.he
	v[0] = m.halfedges[idx].EndVert
	idx = m.halfedgeNext[idx]
	v[1] = m.halfedges[idx].EndVert
	idx = m.halfedgeNext[idx]
	v[2] = m.halfedges[idx].EndVert
	return v
}

// vertexIndicesOfHalfEdge returns (startVert, endVert) of he. The
// startVert is the endVert of he's pair.
func (m *meshBuilder) vertexIndicesOfHalfEdge(he Halfedge) [2]int {
	return [2]int{m.halfedges[he.PairedHalfedge].EndVert, he.EndVert}
}

// halfEdgeIndicesOfFace returns the 3 halfedge indices around f.
func (m *meshBuilder) halfEdgeIndicesOfFace(f *face) [3]int {
	return [3]int{
		f.he,
		m.halfedgeNext[f.he],
		m.halfedgeNext[m.halfedgeNext[f.he]],
	}
}
