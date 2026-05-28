// Package quickhull is the Go port of src/quickhull.{h,cpp} —
// Antti Kuukka's public-domain QuickHull3D algorithm.
//
// INPUT: a slice of 3D points.
// OUTPUT: a triangulated convex hull as (halfedges, vertices).
//
// SUMMARY OF THE ALGORITHM (verbatim from C++):
//   - Create initial simplex (tetrahedron) using extreme points. M
//     has 4 faces.
//   - For each input point, assign it to the first face whose plane
//     it lies on the positive side of (at most one per point).
//     Points inside the tetrahedron are dropped.
//   - Add all faces with at least one assigned point to a face
//     stack.
//   - While the stack is non-empty:
//     - Pop a face F.
//     - Pick the point P assigned to F that is farthest from F's
//       plane.
//     - Find all faces that have P on their positive side ("visible
//       faces"). They form a connected region.
//     - Solve the horizon edge loop bounding the visible region.
//     - "Extrude" P: replace the visible region with new triangles
//       connecting P to each horizon edge.
//     - Re-distribute the points that were assigned to disabled
//       (visible) faces to the new faces.
//     - Push any new face that has assigned points onto the stack.
//   - When the stack drains, M is the convex hull.
package quickhull

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// DefaultEpsilon mirrors C++ defaultEps() — minimum distance to a
// plane to consider a point being on positive side of it, for a
// point cloud with scale 1.
const DefaultEpsilon = 0.0000001

// Halfedge is the QuickHull-internal halfedge record. Mirrors the
// C++ Halfedge struct used by quickhull (startVert, endVert,
// pairedHalfedge — propVert isn't used by quickhull).
type Halfedge struct {
	StartVert      int
	EndVert        int
	PairedHalfedge int
}

// ============================================================
// Plane and Ray helpers — mirror src/quickhull.h:87-117.

// Plane is a 3D plane parameterized by its (unnormalized) normal N
// and signed distance D from origin (negative when origin is on the
// positive side). sqrNLength is |N|² for the "is point on positive
// side" check that avoids a sqrt.
type Plane struct {
	N         geom.Vec3
	D         float64
	SqrNLen   float64
}

// NewPlane builds a Plane from a normal N and a point P that lies
// on the plane. D is computed as -N·P; sqrNLength as N·N.
func NewPlane(N, P geom.Vec3) Plane {
	return Plane{
		N:       N,
		D:       -N.Dot(P),
		SqrNLen: N.Dot(N),
	}
}

// IsPointOnPositiveSide returns true iff N·Q + D >= 0.
func (p Plane) IsPointOnPositiveSide(Q geom.Vec3) bool {
	return p.N.Dot(Q)+p.D >= 0
}

// Ray is a parametric line: S + t·V. VInvLengthSquared = 1/(V·V) is
// precomputed for the distance-to-ray formula in
// getSquaredDistanceBetweenPointAndRay.
type Ray struct {
	S, V              geom.Vec3
	VInvLengthSquared float64
}

// NewRay builds a Ray from origin S and direction vector V.
func NewRay(S, V geom.Vec3) Ray {
	return Ray{S: S, V: V, VInvLengthSquared: 1 / V.Dot(V)}
}

// ============================================================
// Geometric helper functions — mirror src/quickhull.cpp:32-60.

// squaredDistanceBetweenPointAndRay returns |p - (S + t·V)|² where
// t is the projection of (p - S) onto V. Mirrors C++
// getSquaredDistanceBetweenPointAndRay.
func squaredDistanceBetweenPointAndRay(p geom.Vec3, r Ray) float64 {
	s := p.Sub(r.S)
	t := s.Dot(r.V)
	return s.Dot(s) - t*t*r.VInvLengthSquared
}

// squaredDistance returns |p1 - p2|².
func squaredDistance(p1, p2 geom.Vec3) float64 {
	d := p1.Sub(p2)
	return d.Dot(d)
}

// signedDistanceToPlane returns N·v + D — the signed distance from
// v to the plane (in units of |N|; divide by |N| for true distance).
func signedDistanceToPlane(v geom.Vec3, p Plane) float64 {
	return p.N.Dot(v) + p.D
}

// triangleNormal computes (a - c) × (b - c) and normalizes the
// result. Mirrors C++ getTriangleNormal — written out to avoid
// constructing temporary Vec3s.
func triangleNormal(a, b, c geom.Vec3) geom.Vec3 {
	x := a.X - c.X
	y := a.Y - c.Y
	z := a.Z - c.Z
	rhsx := b.X - c.X
	rhsy := b.Y - c.Y
	rhsz := b.Z - c.Z
	n := geom.Vec3{
		X: y*rhsz - z*rhsy,
		Y: z*rhsx - x*rhsz,
		Z: x*rhsy - y*rhsx,
	}
	length := math.Sqrt(n.Dot(n))
	if length == 0 {
		return geom.Vec3{}
	}
	return geom.Vec3{X: n.X / length, Y: n.Y / length, Z: n.Z / length}
}
