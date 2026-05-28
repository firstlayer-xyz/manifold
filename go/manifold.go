// Package manifold is the Go port of the Manifold mesh library.
//
// The implementation is being moved from C++ to Go one function at a time,
// outside in. Functions that haven't been ported yet route through cgo via
// the internal bridge package. The package presents a cgo-free import
// surface to callers; over time the bridge dependency shrinks toward zero.
package manifold

import (
	"math"
	"runtime"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/handle"
)

// Vec2 is a 2D vector with double-precision components.
type Vec2 = geom.Vec2

// Vec3 is a 3D vector with double-precision components.
type Vec3 = geom.Vec3

// SimplePolygon is one closed polygon contour in the XY plane (matches
// C++ SimplePolygon = vector<vec2>).
type SimplePolygon = []Vec2

// Polygons is a set of closed polygon contours forming a possibly
// holey cross section (matches C++ Polygons = vector<SimplePolygon>).
type Polygons = []SimplePolygon

// Extrude sweeps a Polygons cross-section along +Z by height, with
// optional subdivisions, twist, and top scaling. Ported as a single
// bridge to Manifold::Extrude for now; top-down drill comes later.
func Extrude(crossSection Polygons, height float64, nDivisions int,
	twistDegrees float64, scaleTop Vec2,
) *Manifold {
	return wrap(bridge.Extrude(crossSection, height, nDivisions,
		twistDegrees, scaleTop.X, scaleTop.Y))
}

// Box is an axis-aligned bounding box defined by its Min and Max corners.
type Box = geom.Box

// Manifold is a watertight 3D mesh with manifold topology.
type Manifold struct {
	h *handle.Manifold
}

// Mat3x4 is a 3-row, 4-column affine transform in column-major order.
type Mat3x4 = geom.Mat3x4

// NumVert returns the number of vertices in the mesh.
func (m *Manifold) NumVert() int { return bridge.NumVert(m.h) }

// NumTri returns the number of triangles in the mesh. Ported from C++
// Impl::NumTri (halfedge_.size() / 3); the division happens in Go.
func (m *Manifold) NumTri() int {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return impl.HalfedgeCount() / 3
}

// NumEdge returns the number of unique edges in the mesh. Ported from
// C++ Impl::NumEdge (halfedge_.size() / 2).
func (m *Manifold) NumEdge() int {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return impl.HalfedgeCount() / 2
}

// IsEmpty reports whether the Manifold has no geometry. Ported from C++
// Impl::IsEmpty (NumTri() == 0, i.e. halfedge_.size() == 0).
func (m *Manifold) IsEmpty() bool {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return impl.HalfedgeCount() == 0
}

// Volume returns the signed volume of the closed mesh.
//
// Ported from C++ Impl::GetProperty(Volume): for each triangle take the
// signed tetrahedron volume between the triangle and the origin and sum
// them with Kahan compensation. The data (vertex positions and the
// halfedge start array) crosses cgo as read-only buffers; the loop runs
// in Go.
func (m *Manifold) Volume() float64 {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	verts := impl.Verts()
	starts := impl.HalfedgeStarts()
	numTri := len(starts) / 3
	if numTri == 0 {
		return 0
	}
	var sum, comp float64
	for t := 0; t < numTri; t++ {
		v0 := verts[starts[3*t]]
		v1 := verts[starts[3*t+1]]
		v2 := verts[starts[3*t+2]]
		cross := v1.Sub(v0).Cross(v2.Sub(v0))
		val := cross.Dot(v0) / 6.0
		y := val - comp
		t1 := sum + y
		comp = (t1 - sum) - y
		sum = t1
	}
	return sum + comp
}

// SurfaceArea returns the total surface area of the mesh.
//
// Same shape as Volume: iterate triangles, accumulate areas with Kahan
// compensation.
func (m *Manifold) SurfaceArea() float64 {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	verts := impl.Verts()
	starts := impl.HalfedgeStarts()
	numTri := len(starts) / 3
	if numTri == 0 {
		return 0
	}
	var sum, comp float64
	for t := 0; t < numTri; t++ {
		v0 := verts[starts[3*t]]
		v1 := verts[starts[3*t+1]]
		v2 := verts[starts[3*t+2]]
		cross := v1.Sub(v0).Cross(v2.Sub(v0))
		val := math.Sqrt(cross.Dot(cross)) / 2.0
		y := val - comp
		t1 := sum + y
		comp = (t1 - sum) - y
		sum = t1
	}
	return sum + comp
}

// NumProp returns the number of properties per vertex (3 = position only).
// Ported via Impl::numProp_.
func (m *Manifold) NumProp() int {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return impl.Scalars().NumProp
}

// NumPropVert returns the number of property vertices. Ported from
// Impl::NumPropVert: NumProp == 0 ? NumVert : properties_.size() / NumProp.
func (m *Manifold) NumPropVert() int {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	s := impl.Scalars()
	if s.NumProp == 0 {
		return len(impl.Verts())
	}
	return s.PropertiesSize / s.NumProp
}

// Genus returns the topological genus. Ported from Impl::Genus:
// 1 - (V - E + F) / 2. All three counts are already Go-native.
func (m *Manifold) Genus() int {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	v := len(impl.Verts())
	he := impl.HalfedgeCount()
	e := he / 2
	f := he / 3
	return 1 - (v-e+f)/2
}

// GetTolerance returns the geometric tolerance of the mesh.
// Ported via Impl::tolerance_.
func (m *Manifold) GetTolerance() float64 {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return impl.Scalars().Tolerance
}

// OriginalID returns -1 for derived Manifolds, otherwise the ID of the
// originating Manifold. Ported via Impl::meshRelation_.originalID.
func (m *Manifold) OriginalID() int {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return impl.Scalars().OriginalID
}

// AsOriginal forgets the CSG history and returns a copy with a fresh ID.
//
// Ported top-down from C++ Manifold::AsOriginal, line for line:
//   - GetCsgLeafNode().GetImpl()   — bridge.GetImpl
//   - PropagateStatus on error      — bridge.PropagateStatus
//   - make_shared<Impl>(*oldImpl)   — impl.Copy()  → MutableImpl
//   - newImpl->InitializeOriginal() — newImpl.InitializeOriginal()
//   - newImpl->SetNormalsAndCoplanar() — newImpl.SetNormalsAndCoplanar()
//   - Manifold(make_shared<CsgLeafNode>(newImpl)) — newImpl.ToManifold()
// Each line of the C++ body has a matching Go statement that calls into
// the corresponding inner C++ function unchanged. Future drilling will
// replace InitializeOriginal / SetNormalsAndCoplanar with Go ports.
func (m *Manifold) AsOriginal() *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if status := impl.Scalars().Status; Error(status) != NoError {
		return wrap(bridge.PropagateStatus(status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	// hadNormals reflects whether the source impl's meshIDtransform
	// already records normals — for AsOriginal we currently lose that
	// distinction and pass false. (A future drill that reads
	// AllHaveNormals on the mutable side will lift this.)
	numTri := len(newImpl.HalfedgeStartsRO()) / 3
	initializeOriginal(newImpl, numTri, false)
	setNormalsAndCoplanar(newImpl)
	return wrap(newImpl.ToManifold())
}

// Simplify returns a copy whose mesh is simplified to the given tolerance
// (or the existing tolerance if tolerance == 0). The actual tolerance
// field is restored at the end — only the simplification used the new
// value.
//
// Ported top-down from C++ Manifold::Simplify.
func (m *Manifold) Simplify(tolerance float64) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	s := impl.Scalars()
	if Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	oldTolerance := s.Tolerance
	if tolerance == 0 {
		tolerance = oldTolerance
	}
	if tolerance > oldTolerance {
		newImpl.SetToleranceValue(tolerance)
		setNormalsAndCoplanar(newImpl)
	}
	newImpl.SimplifyTopology()
	newImpl.SortGeometry()
	newImpl.SetToleranceValue(oldTolerance)
	return wrap(newImpl.ToManifold())
}

// CalculateCurvature returns a copy with Gaussian / mean curvature
// values written into the given property slots (use -1 to skip a slot).
//
// Ported top-down from C++ Manifold::CalculateCurvature.
func (m *Manifold) CalculateCurvature(gaussianIdx, meanIdx int) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	newImpl.CalculateCurvature(gaussianIdx, meanIdx)
	return wrap(newImpl.ToManifold())
}

// CalculateNormals returns a copy with per-vertex normals stored
// starting at property slot normalIdx. Edges sharper than
// minSharpAngle (degrees) get distinct normals on each side.
//
// Ported top-down from C++ Manifold::CalculateNormals.
func (m *Manifold) CalculateNormals(normalIdx int, minSharpAngle float64) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	newImpl.SetNormals(normalIdx, minSharpAngle)
	if normalIdx == 0 {
		newImpl.MarkAllMeshIDHasNormals()
	}
	return wrap(newImpl.ToManifold())
}

// SmoothByNormals fills in halfedge tangents using vertex normals stored
// at the given property slot. Later Refine* calls will use these to
// place new verts on the implied smooth surface.
//
// Ported top-down from C++ Manifold::SmoothByNormals.
func (m *Manifold) SmoothByNormals(normalIdx int) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	if !m.IsEmpty() {
		newImpl.CreateTangentsIdx(normalIdx)
	}
	return wrap(newImpl.ToManifold())
}

// SmoothOut fills in halfedge tangents from triangle geometry, sharpening
// edges above minSharpAngle and applying minSmoothness elsewhere.
//
// Ported top-down from C++ Manifold::SmoothOut.
func (m *Manifold) SmoothOut(minSharpAngle, minSmoothness float64) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	if !m.IsEmpty() {
		sv := newImpl.SharpenEdges(minSharpAngle, minSmoothness)
		defer sv.Delete()
		newImpl.CreateTangentsFromSmoothness(sv)
	}
	return wrap(newImpl.ToManifold())
}

// ReserveIDs reserves a contiguous block of n mesh IDs from the global
// counter and returns the first ID.
//
// Ported from C++ static Manifold::ReserveIDs.
func ReserveIDs(n uint32) uint32 {
	return bridge.ImplReserveIDs(n)
}

// SetTolerance returns a copy of m with the specified geometric tolerance.
//
// Ported top-down from C++ Manifold::SetTolerance, line for line:
//   - GetCsgLeafNode().GetImpl()    — bridge.GetImpl
//   - status check / PropagateStatus — bridge.PropagateStatus
//   - make_shared<Impl>(*leafImpl)   — impl.Copy() → MutableImpl
//   - if tol > impl->tolerance_      — read from source via Scalars()
//       set tolerance / Normals / SimplifyTopology / SortGeometry
//   - else                            — set tol to max(epsilon, tol)
//   - return Manifold(impl)           — newImpl.ToManifold()
func (m *Manifold) SetTolerance(tol float64) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	s := impl.Scalars()
	if Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	if tol > s.Tolerance {
		newImpl.SetToleranceValue(tol)
		setNormalsAndCoplanar(newImpl)
		newImpl.SimplifyTopology()
		newImpl.SortGeometry()
	} else {
		// For reducing tolerance, keep it at least equal to epsilon.
		newImpl.SetToleranceValue(math.Max(s.Epsilon, tol))
	}
	return wrap(newImpl.ToManifold())
}

// halfspace builds a cuboidal "cutter" Manifold whose +X face lies on
// the plane through `normal * originOffset` with the given normal,
// large enough to cover bBox. Ported from C++ Halfspace in
// src/manifold.cpp.
func halfspace(bBox Box, normal Vec3, originOffset float64) *Manifold {
	normal = normal.Normalize()
	cutter := Cube(Vec3{X: 2, Y: 2, Z: 2}, true).Translate(Vec3{X: 1})
	size := bBox.Center().Sub(normal.Scale(originOffset)).Length() + 0.5*bBox.Size().Length()
	cutter = cutter.Scale(Vec3{X: size, Y: size, Z: size}).Translate(Vec3{X: originOffset})
	yDeg := geom.Degrees(-math.Asin(normal.Z))
	zDeg := geom.Degrees(math.Atan2(normal.Y, normal.X))
	return cutter.Rotate(0, yDeg, zDeg)
}

// TrimByPlane returns the half of m on the side of the plane opposite the
// normal, with the cut sealed.
//
// Ported top-down from C++:
//   Manifold Manifold::TrimByPlane(vec3 normal, double originOffset) const {
//     return *this ^ Halfspace(BoundingBox(), normal, originOffset);
//   }
func (m *Manifold) TrimByPlane(normal Vec3, offset float64) *Manifold {
	return m.Intersection(halfspace(m.BoundingBox(), normal, offset))
}

// MinGap returns the minimum surface distance between m and other, or 0
// when they overlap. The search is bounded by searchLength.
//
// Ported top-down from C++:
//   double Manifold::MinGap(const Manifold& other, double searchLength) const {
//     auto intersect = *this ^ other;
//     if (!intersect.IsEmpty()) return 0.0;
//     return GetCsgLeafNode().GetImpl()->MinGap(*other...->GetImpl(), searchLength);
//   }
func (m *Manifold) MinGap(other *Manifold, searchLength float64) float64 {
	if !m.Intersection(other).IsEmpty() {
		return 0
	}
	aImpl := bridge.GetImpl(m.h)
	defer aImpl.Delete()
	bImpl := bridge.GetImpl(other.h)
	defer bImpl.Delete()
	return aImpl.MinGap(bImpl, searchLength)
}

// OpType selects which boolean operation Boolean performs. Mirrors the
// C++ enum class OpType.
type OpType int

const (
	OpAdd       OpType = 0 // Union
	OpSubtract  OpType = 1 // Difference
	OpIntersect OpType = 2 // Intersection
)

// Boolean performs the given boolean operation between m and other.
//
// Ported top-down from C++:
//   Manifold Manifold::Boolean(const Manifold& second, OpType op) const {
//     return Manifold(LoadPNode()->Boolean(second.LoadPNode(), op));
//   }
func (m *Manifold) Boolean(other *Manifold, op OpType) *Manifold {
	a := bridge.LoadPNode(m.h)
	defer a.Delete()
	b := bridge.LoadPNode(other.h)
	defer b.Delete()
	result := a.Boolean(b, int(op))
	defer result.Delete()
	return wrap(result.ToManifold())
}

// Union returns the boolean union of m and other. Ported from C++
// Manifold::operator+: delegates to Boolean with OpAdd.
func (m *Manifold) Union(other *Manifold) *Manifold {
	return m.Boolean(other, OpAdd)
}

// Difference returns m with the geometry of other subtracted. Ported from
// C++ Manifold::operator-: delegates to Boolean with OpSubtract.
func (m *Manifold) Difference(other *Manifold) *Manifold {
	return m.Boolean(other, OpSubtract)
}

// Intersection returns the boolean intersection of m and other. Ported
// from C++ Manifold::operator^: delegates to Boolean with OpIntersect.
func (m *Manifold) Intersection(other *Manifold) *Manifold {
	return m.Boolean(other, OpIntersect)
}

// Split cuts m using the cutter manifold, returning (intersection,
// difference). Sharing a single Boolean3 evaluation between both
// results is cheaper than running Intersection and Difference
// separately.
//
// Ported top-down from C++ Manifold::Split:
//   auto impl1 = GetCsgLeafNode().GetImpl();
//   auto impl2 = cutter.GetCsgLeafNode().GetImpl();
//   Boolean3 boolean(*impl1, *impl2, OpType::Subtract);
//   auto result1 = ...Impl(boolean.Result(OpType::Intersect));
//   auto result2 = ...Impl(boolean.Result(OpType::Subtract));
//   return std::make_pair(Manifold(result1), Manifold(result2));
func (m *Manifold) Split(cutter *Manifold) (*Manifold, *Manifold) {
	impl1 := bridge.GetImpl(m.h)
	defer impl1.Delete()
	impl2 := bridge.GetImpl(cutter.h)
	defer impl2.Delete()
	boolean := bridge.NewBoolean3(impl1, impl2, int(OpSubtract))
	defer boolean.Delete()
	result1 := boolean.Result(int(OpIntersect))
	result2 := boolean.Result(int(OpSubtract))
	return wrap(result1), wrap(result2)
}

// SplitByPlane is a half-space-cut convenience over Split: the first
// result is the half on the normal side, the second on the opposite.
//
// Ported top-down from C++ Manifold::SplitByPlane:
//   auto leafImpl = GetCsgLeafNode().GetImpl();
//   if (leafImpl->status_ != Error::NoError) {
//     Manifold err = PropagateStatus(leafImpl->status_);
//     return {err, err};
//   }
//   if (IsEmpty()) return {Manifold(), Manifold()};
//   return Split(Halfspace(BoundingBox(), normal, originOffset));
func (m *Manifold) SplitByPlane(normal Vec3, originOffset float64) (*Manifold, *Manifold) {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if status := impl.Scalars().Status; Error(status) != NoError {
		err := wrap(bridge.PropagateStatus(status))
		return err, err
	}
	if m.IsEmpty() {
		return wrap(bridge.Invalid()), wrap(bridge.Invalid())
	}
	return m.Split(halfspace(m.BoundingBox(), normal, originOffset))
}

// MatchesTriNormals reports whether every face normal agrees with the
// cross product of its triangle's halfedges. Used as a sanity check in
// tests.
//
// Ported top-down from C++:
//   bool Manifold::MatchesTriNormals() const {
//     return GetCsgLeafNode().GetImpl()->MatchesTriNormals();
//   }
//
// The inner Impl::MatchesTriNormals is drilled to native Go — see
// impl_props.go.
func (m *Manifold) MatchesTriNormals() bool {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return implMatchesTriNormals(impl)
}

// NumDegenerateTris returns the count of triangles with zero or
// near-zero area. Used as a sanity check in tests.
//
// Ported top-down from C++:
//   size_t Manifold::NumDegenerateTris() const {
//     return GetCsgLeafNode().GetImpl()->NumDegenerateTris();
//   }
//
// The inner Impl::NumDegenerateTris is drilled to native Go — see
// impl_props.go.
func (m *Manifold) NumDegenerateTris() int {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return implNumDegenerateTris(impl)
}

// GetEpsilon returns the precision used in this Manifold's boolean
// operations and tolerance-based geometry tests (Impl::epsilon_).
//
// Ported top-down from C++:
//   double Manifold::GetEpsilon() const {
//     return GetCsgLeafNode().GetImpl()->epsilon_;
//   }
func (m *Manifold) GetEpsilon() float64 {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return impl.Scalars().Epsilon
}

// Sphere returns a sphere of the given radius. circularSegments controls
// tessellation; pass 0 to use the global Quality default.
//
// Ported top-down from C++ Manifold::Sphere: subdivide an octahedron and
// project verts onto the sphere via per-axis cosine + radius·normalize.
func Sphere(radius float64, circularSegments int) *Manifold {
	if radius <= 0 {
		return wrap(bridge.Invalid())
	}
	var n int
	if circularSegments > 0 {
		n = (circularSegments + 3) / 4
	} else {
		n = GetCircularSegments(radius) / 4
	}
	identity := Mat3x4{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}}
	impl := newImplFromShape(shapeOctahedron, identity)
	defer impl.Delete()
	impl.SubdivideN(n)

	// Per-vertex projection: matches the lambda inside the C++
	// for_each_n on vertPos_. Mutates the Impl's buffer in place via
	// the Go slice returned by Verts.
	verts := impl.Verts()
	half := math.Pi / 2
	for i := range verts {
		v := verts[i]
		v = Vec3{
			X: math.Cos(half * (1 - v.X)),
			Y: math.Cos(half * (1 - v.Y)),
			Z: math.Cos(half * (1 - v.Z)),
		}
		length := math.Sqrt(v.Dot(v))
		if length == 0 {
			verts[i] = Vec3{}
			continue
		}
		inv := radius / length
		v = Vec3{X: v.X * inv, Y: v.Y * inv, Z: v.Z * inv}
		if math.IsNaN(v.X) {
			verts[i] = Vec3{}
		} else {
			verts[i] = v
		}
	}

	// Finalize — drilled steps mirror C++ Sphere's tail. NumTri is
	// derived from the halfedge count.
	numTri := len(impl.HalfedgeStartsRO()) / 3
	initializeOriginal(impl, numTri, false)
	calculateBBox(impl)
	setEpsilon(impl, -1, false)
	impl.SortGeometry()
	setNormalsAndCoplanar(impl)
	return wrap(impl.ToManifold())
}

// Cylinder returns a cylinder of the given height. If radiusLow != radiusHigh
// the result is a frustum. circularSegments controls tessellation; pass 0
// for the global default. If center is true the cylinder spans
// [-height/2, height/2] along Z; otherwise [0, height].
//
// Ported top-down from C++ Manifold::Cylinder.
func Cylinder(height, radiusLow, radiusHigh float64, circularSegments int, center bool) *Manifold {
	if height <= 0 || radiusLow < 0 {
		return wrap(bridge.Invalid())
	}
	if radiusLow == 0 {
		if radiusHigh <= 0 {
			return wrap(bridge.Invalid())
		}
		// Cone with apex at bottom: build the centered apex-at-top version
		// and mirror it.
		cone := Cylinder(height, radiusHigh, 0, circularSegments, true)
		cone = cone.Mirror(Vec3{Z: 1})
		if !center {
			cone = cone.Translate(Vec3{Z: height / 2})
		}
		return cone.AsOriginal()
	}
	scale := 1.0
	if radiusHigh >= 0 {
		scale = radiusHigh / radiusLow
	}
	radius := math.Max(radiusLow, radiusHigh)
	n := circularSegments
	if n <= 2 {
		n = GetCircularSegments(radius)
	}

	circle := make(SimplePolygon, n)
	dPhi := 360.0 / float64(n)
	for i := 0; i < n; i++ {
		circle[i] = Vec2{
			X: radiusLow * geom.Cosd(dPhi*float64(i)),
			Y: radiusLow * geom.Sind(dPhi*float64(i)),
		}
	}

	cylinder := Extrude(Polygons{circle}, height, 0, 0, Vec2{X: scale, Y: scale})
	if center {
		cylinder = cylinder.Translate(Vec3{Z: -height / 2}).AsOriginal()
	}
	return cylinder
}

// Error categorizes the validity of a Manifold. Mirrors the C++
// Manifold::Error enum.
type Error int

const (
	NoError Error = iota
	NonFiniteVertex
	NotManifold
	VertexIndexOutOfBounds
	PropertiesWrongLength
	MissingPositionProperties
	MergeVectorsDifferentLengths
	MergeIndexOutOfBounds
	TransformWrongLength
	RunIndexWrongLength
	FaceIDWrongLength
	InvalidConstruction
	ResultTooLarge
	InvalidTangents
	Cancelled
)

// Status returns the validity status of the Manifold. Ported via
// Impl::status_. NOTE: the C++ Status also routes through any attached
// ExecutionContext; the Go port does not yet support ctx_, so this reads
// status_ directly.
func (m *Manifold) Status() Error {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return Error(impl.Scalars().Status)
}

// HullPts returns the convex hull of a free-standing set of points.
//
// Ported top-down from C++:
//   Manifold Manifold::Hull(const std::vector<vec3>& pts) {
//     std::shared_ptr<Impl> impl = std::make_shared<Impl>();
//     impl->Hull(Vec<vec3>(pts));
//     return Manifold(std::make_shared<CsgLeafNode>(impl));
//   }
func HullPts(pts []Vec3) *Manifold {
	impl := bridge.NewMutableImpl()
	defer impl.Delete()
	impl.Hull(pts)
	return wrap(impl.ToManifold())
}

// BatchBoolean performs the given boolean op across a list of manifolds.
// Empty input returns a default Manifold; single input is returned as-is.
//
// Ported top-down from C++ Manifold::BatchBoolean.
func BatchBoolean(manifolds []*Manifold, op OpType) *Manifold {
	if len(manifolds) == 0 {
		return wrap(bridge.Empty())
	}
	if len(manifolds) == 1 {
		return manifolds[0]
	}
	children := make([]*bridge.CsgNode, len(manifolds))
	for i, m := range manifolds {
		children[i] = bridge.LoadPNode(m.h)
	}
	defer func() {
		for _, c := range children {
			c.Delete()
		}
	}()
	opNode := bridge.NewCsgOpNode(children, int(op))
	defer opNode.Delete()
	return wrap(opNode.ToManifold())
}

// Compose unions a list of manifolds — equivalent to BatchBoolean(OpAdd).
//
// Ported top-down from C++ Manifold::Compose.
func Compose(manifolds []*Manifold) *Manifold {
	return BatchBoolean(manifolds, OpAdd)
}

// BatchHull returns the convex hull enveloping a set of manifolds.
//
// Ported top-down from C++ Manifold::Hull(vector<Manifold>).
func BatchHull(manifolds []*Manifold) *Manifold {
	for _, m := range manifolds {
		if status := m.Status(); status != NoError {
			return wrap(bridge.PropagateStatus(int(status)))
		}
	}
	var total int
	for _, m := range manifolds {
		total += m.NumVert()
	}
	if total == 0 {
		return wrap(bridge.Empty())
	}
	vertPos := make([]Vec3, 0, total)
	for _, m := range manifolds {
		impl := bridge.GetImpl(m.h)
		vertPos = append(vertPos, impl.Verts()...)
		impl.Delete()
	}
	out := bridge.NewMutableImpl()
	defer out.Delete()
	out.Hull(vertPos)
	return wrap(out.ToManifold())
}

// MeshGL is the float32 / uint32 variant of MeshGL64 — same fields, less
// precision, smaller indices. Mirrors the C++ manifold::MeshG (= MeshGLP<float>).
type MeshGL struct {
	NumProp         int
	VertProperties  []float32
	TriVerts        []uint32
	MergeFromVert   []uint32
	MergeToVert     []uint32
	RunIndex        []uint32
	RunOriginalID   []uint32
	RunTransform    []float32
	RunFlags        []uint8
	FaceID          []uint32
	HalfedgeTangent []float32
	Tolerance       float32
}

// GetMeshGL exports the mesh as a single-precision MeshGL.
//
// Ported top-down from C++:
//   MeshGL Manifold::GetMeshGL(int normalIdx) const {
//     const Impl& impl = *GetCsgLeafNode().GetImpl();
//     if (normalIdx < 0 && impl.AllHaveNormals()) normalIdx = 0;
//     return GetMeshGLImpl<float, uint32_t>(impl, normalIdx);
//   }
func (m *Manifold) GetMeshGL(normalIdx int) MeshGL {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if normalIdx < 0 && impl.AllHaveNormals() {
		normalIdx = 0
	}
	mh := impl.GetMeshGL(normalIdx)
	defer mh.Delete()
	return MeshGL{
		NumProp:         mh.NumProp(),
		VertProperties:  mh.VertProperties(),
		TriVerts:        mh.TriVerts(),
		MergeFromVert:   mh.MergeFromVert(),
		MergeToVert:     mh.MergeToVert(),
		RunIndex:        mh.RunIndex(),
		RunOriginalID:   mh.RunOriginalID(),
		RunTransform:    mh.RunTransform(),
		RunFlags:        mh.RunFlags(),
		FaceID:          mh.FaceID(),
		HalfedgeTangent: mh.HalfedgeTangent(),
		Tolerance:       mh.Tolerance(),
	}
}

// MeshGL64 mirrors the C++ manifold::MeshGL64 mesh-export struct.
// Optional fields are nil when empty; required fields (VertProperties,
// TriVerts, NumProp) are always set.
type MeshGL64 struct {
	NumProp         int
	VertProperties  []float64
	TriVerts        []uint64
	MergeFromVert   []uint64
	MergeToVert     []uint64
	RunIndex        []uint64
	RunOriginalID   []uint32
	RunTransform    []float64
	RunFlags        []uint8
	FaceID          []uint64
	HalfedgeTangent []float64
	Tolerance       float64
}

// NewManifoldFromMeshGL64 constructs a Manifold from a Go-owned
// MeshGL64. Ported from C++ `Manifold(const MeshGL64&)`. Mesh validation
// happens on the C++ side; an invalid mesh produces a Manifold with
// non-NoError Status.
func NewManifoldFromMeshGL64(m MeshGL64) *Manifold {
	return wrap(bridge.ManifoldFromMeshGL64(
		m.NumProp,
		m.VertProperties, m.TriVerts,
		m.MergeFromVert, m.MergeToVert,
		m.RunIndex, m.RunOriginalID, m.RunTransform, m.RunFlags,
		m.FaceID, m.HalfedgeTangent,
		m.Tolerance,
	))
}

// NewManifoldFromMeshGL constructs a Manifold from a Go-owned MeshGL.
// Ported from C++ `Manifold(const MeshGL&)`.
func NewManifoldFromMeshGL(m MeshGL) *Manifold {
	return wrap(bridge.ManifoldFromMeshGL(
		m.NumProp,
		m.VertProperties, m.TriVerts,
		m.MergeFromVert, m.MergeToVert,
		m.RunIndex, m.RunOriginalID, m.RunTransform, m.RunFlags,
		m.FaceID, m.HalfedgeTangent,
		m.Tolerance,
	))
}

// Smoothness mirrors C++ struct manifold::Smoothness. Used by Smooth to
// describe per-halfedge sharpening.
type Smoothness struct {
	// Halfedge index: 3*tri + i, referring to Mesh.triVerts[tri][i].
	Halfedge uint64
	// Smoothness in [0, 1]: 0 = sharp crease, 1 = default smooth.
	Smoothness float64
}

// SmoothFromMeshGL64 constructs a smooth Manifold from a Go-owned
// MeshGL64 by creating tangents. The input must not contain tangents
// already (the C++ side raises an error if it does). Refine() the
// result to actually increase the triangle resolution.
//
// Ported top-down from C++ Manifold::Smooth(const MeshGL64&, const
// std::vector<Smoothness>&):
//   return Manifold(SmoothImpl(meshGL64, sharpenedEdges));
func SmoothFromMeshGL64(m MeshGL64, sharpenedEdges []Smoothness) *Manifold {
	edges := make([]bridge.Smoothness, len(sharpenedEdges))
	for i, s := range sharpenedEdges {
		edges[i] = bridge.Smoothness{Halfedge: s.Halfedge, Smoothness: s.Smoothness}
	}
	return wrap(bridge.SmoothFromMeshGL64(
		m.NumProp,
		m.VertProperties, m.TriVerts,
		m.MergeFromVert, m.MergeToVert,
		m.RunIndex, m.RunOriginalID, m.RunTransform, m.RunFlags,
		m.FaceID, m.HalfedgeTangent,
		m.Tolerance,
		edges,
	))
}

// ExecutionContext observes progress and requests cancellation of a
// long-running Manifold evaluation. Attach to a Manifold via
// WithContext; the next eager op (Status, Refine, RefineToLength,
// RefineToTolerance, Hull, MinkowskiSum, MinkowskiDifference) on the
// returned Manifold reports progress and observes cancellation
// through it. Safe to read/write from any thread.
//
// Cancel is sticky: once Cancel() has been called, every subsequent
// evaluation through this context (or any copy) short-circuits to
// Error::Cancelled. Construct a fresh context to retry.
//
// Construction pairs with Delete; the underlying C++ ExecutionContext
// is freed there. Calling Cancel/Cancelled/Progress on a deleted
// context is undefined behavior.
type ExecutionContext struct {
	c *bridge.ExecutionContext
}

// NewExecutionContext returns a fresh ExecutionContext whose cancel
// flag is clear and Progress is 1.0 (no work scheduled).
//
// Ported top-down from C++:
//   ExecutionContext ctx;
func NewExecutionContext() *ExecutionContext {
	return &ExecutionContext{c: bridge.NewExecutionContext()}
}

// Cancel requests cancellation of any in-progress evaluation using
// this context. Idempotent. Sticky — future evaluations on the same
// context (or any copy) will also short-circuit to Error::Cancelled.
//
// Ported from ExecutionContext::Cancel.
func (ctx *ExecutionContext) Cancel() { ctx.c.Cancel() }

// Cancelled reports whether Cancel() has ever been called on this
// context.
//
// Ported from ExecutionContext::Cancelled.
func (ctx *ExecutionContext) Cancelled() bool { return ctx.c.Cancelled() }

// Progress returns the normalized [0, 1] progress of the current (or
// most recent) evaluation through this context. Returns 1.0 when no
// work has been scheduled.
//
// Ported from ExecutionContext::Progress.
func (ctx *ExecutionContext) Progress() float64 { return ctx.c.Progress() }

// Delete frees the underlying C++ context. Must be called exactly
// once per NewExecutionContext.
func (ctx *ExecutionContext) Delete() { ctx.c.Delete() }

// WithContext returns a copy of m with ctx attached. The next eager
// op on the result reports progress and observes cancellation through
// ctx. The original m is unaffected.
//
// Ported top-down from C++ Manifold::WithContext:
//   Manifold result = *this;
//   std::atomic_store(&result.ctx_, ctx.impl_);
//   return result;
func (m *Manifold) WithContext(ctx *ExecutionContext) *Manifold {
	return wrap(bridge.WithContext(m.h, ctx.c))
}

// SetProperties returns a copy of m with numProp properties per
// vertex, filled by fn. fn receives a writable slice for the new
// properties at this vertex, the vertex position, and a read-only
// slice with the previous properties (length = old NumProp). A nil fn
// mirrors the C++ nullptr-callback path: every new property is
// zero-filled (in parallel).
//
// Ported top-down from C++ Manifold::SetProperties — wraps the public
// static call via a callback trampoline. The for_each_n that walks
// triangles/halfedges still runs in C++.
func (m *Manifold) SetProperties(
	numProp int,
	fn func(newProp []float64, pos Vec3, oldProp []float64),
) *Manifold {
	return wrap(bridge.SetProperties(m.h, numProp, fn))
}

// LevelSet constructs a Manifold from a signed distance function. The
// returned mesh is the iso-surface where sdf(p) == level, restricted
// to bounds. edgeLength sets the voxel grid resolution; tolerance ≤ 0
// uses the C++ default (infinity, i.e. no SDF-aware tightening). When
// canParallel is true, sdf may be called from multiple worker threads
// concurrently — only enable it if the function is safe under that
// guarantee.
//
// Ported top-down: wraps Manifold::LevelSet via a Go-side callback
// trampoline. Future drilling will move the marching-tetrahedra
// algorithm itself to Go.
func LevelSet(
	sdf func(Vec3) float64,
	bounds Box,
	edgeLength, level, tolerance float64,
	canParallel bool,
) *Manifold {
	return wrap(bridge.LevelSet(
		sdf,
		bounds.Min, bounds.Max,
		edgeLength, level, tolerance,
		canParallel,
	))
}

// SmoothFromMeshGL constructs a smooth Manifold from a Go-owned
// MeshGL by creating tangents (float32/uint32 variant of
// SmoothFromMeshGL64).
//
// Ported top-down from C++ Manifold::Smooth(const MeshGL&, const
// std::vector<Smoothness>&):
//   return Manifold(SmoothImpl(meshGL, sharpenedEdges));
func SmoothFromMeshGL(m MeshGL, sharpenedEdges []Smoothness) *Manifold {
	edges := make([]bridge.Smoothness, len(sharpenedEdges))
	for i, s := range sharpenedEdges {
		edges[i] = bridge.Smoothness{Halfedge: s.Halfedge, Smoothness: s.Smoothness}
	}
	return wrap(bridge.SmoothFromMeshGL(
		m.NumProp,
		m.VertProperties, m.TriVerts,
		m.MergeFromVert, m.MergeToVert,
		m.RunIndex, m.RunOriginalID, m.RunTransform, m.RunFlags,
		m.FaceID, m.HalfedgeTangent,
		m.Tolerance,
		edges,
	))
}

// GetMeshGL64 exports the mesh as a MeshGL64. normalIdx selects which
// property slot to fill with vertex normals: pass -1 to use whatever
// the Manifold has (slot 0 if AllHaveNormals is true; otherwise no
// normals).
//
// Ported top-down from C++:
//   MeshGL64 Manifold::GetMeshGL64(int normalIdx) const {
//     const Impl& impl = *GetCsgLeafNode().GetImpl();
//     if (normalIdx < 0 && impl.AllHaveNormals()) normalIdx = 0;
//     return GetMeshGLImpl<double, uint64_t>(impl, normalIdx);
//   }
func (m *Manifold) GetMeshGL64(normalIdx int) MeshGL64 {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if normalIdx < 0 && impl.AllHaveNormals() {
		normalIdx = 0
	}
	mh := impl.GetMeshGL64(normalIdx)
	defer mh.Delete()
	return MeshGL64{
		NumProp:         mh.NumProp(),
		VertProperties:  mh.VertProperties(),
		TriVerts:        mh.TriVerts(),
		MergeFromVert:   mh.MergeFromVert(),
		MergeToVert:     mh.MergeToVert(),
		RunIndex:        mh.RunIndex(),
		RunOriginalID:   mh.RunOriginalID(),
		RunTransform:    mh.RunTransform(),
		RunFlags:        mh.RunFlags(),
		FaceID:          mh.FaceID(),
		HalfedgeTangent: mh.HalfedgeTangent(),
		Tolerance:       mh.Tolerance(),
	}
}

// RayHit mirrors the C++ RayHit struct.
type RayHit = bridge.RayHit

// RayCast intersects a ray segment from origin to endpoint against the
// manifold's surface, returning all hits sorted by distance.
//
// Ported top-down from C++:
//   std::vector<RayHit> Manifold::RayCast(vec3 origin, vec3 endpoint) const {
//     return GetCsgLeafNode().GetImpl()->RayCast(origin, endpoint);
//   }
func (m *Manifold) RayCast(origin, endpoint Vec3) []RayHit {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	return impl.RayCast(origin, endpoint)
}

// Decompose splits the Manifold into a slice of topologically
// disconnected pieces. A fully-connected mesh returns a 1-element slice
// containing a copy of the input.
//
// Ported top-down from C++ Manifold::Decompose: build a DisjointSets
// over the verts, unite vert endpoints of every forward halfedge,
// extract per-component vert and face indices, gather a new Impl for
// each component.
func (m *Manifold) Decompose() []*Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	scalars := impl.Scalars()
	if Error(scalars.Status) != NoError {
		return []*Manifold{wrap(bridge.PropagateStatus(scalars.Status))}
	}

	numVert := len(impl.Verts())
	uf := bridge.NewDisjointSets(numVert)
	defer uf.Delete()

	starts := impl.HalfedgeStarts()
	for edge := 0; edge < len(starts); edge++ {
		startV := int(starts[edge])
		nextInTri := (edge/3)*3 + (edge+1)%3
		endV := int(starts[nextInTri])
		if startV < endV { // IsForward
			uf.Unite(startV, endV)
		}
	}
	numComponents, vertLabel := uf.ConnectedComponents()

	if numComponents == 1 {
		return []*Manifold{m}
	}

	numTri := len(starts) / 3
	srcVerts := impl.Verts()
	srcNormals := impl.VertNormals()
	var meshes []*Manifold
	for comp := 0; comp < numComponents; comp++ {
		newImpl := bridge.NewMutableImpl()
		newImpl.SetEpsilonValue(scalars.Epsilon)
		newImpl.SetToleranceValue(scalars.Tolerance)

		vertNew2Old := make([]int32, 0, numVert)
		for v := 0; v < numVert; v++ {
			if int(vertLabel[v]) == comp {
				vertNew2Old = append(vertNew2Old, int32(v))
			}
		}
		nVert := len(vertNew2Old)
		newImpl.ResizeVerts(nVert)
		newImpl.ResizeVertNormals(len(srcNormals))
		newVerts := newImpl.Verts()
		newNormals := newImpl.VertNormals()
		// gather verts (and normals if present) from old indices.
		hasNormals := len(srcNormals) > 0
		// resize_vert_normals(srcNormals_size) above, then we'll resize to nVert below
		// — match C++ which does .resize(nVert) on vertNormal_ too.
		newImpl.ResizeVertNormals(nVert)
		if hasNormals {
			newNormals = newImpl.VertNormals()
		}
		for i, old := range vertNew2Old {
			newVerts[i] = srcVerts[old]
			if hasNormals {
				newNormals[i] = srcNormals[old]
			}
		}

		faceNew2Old := make([]int32, 0, numTri)
		for face := 0; face < numTri; face++ {
			if int(vertLabel[starts[3*face]]) == comp {
				faceNew2Old = append(faceNew2Old, int32(face))
			}
		}
		if len(faceNew2Old) == 0 {
			newImpl.Delete()
			continue
		}

		newImpl.GatherFaces(impl, faceNew2Old)
		newImpl.ReindexVerts(vertNew2Old, numVert)
		calculateBBox(newImpl)
		newImpl.SortGeometry()
		meshes = append(meshes, wrap(newImpl.ToManifold()))
		newImpl.Delete()
	}
	return meshes
}

// Warp returns a copy of m with each vertex passed through fn. fn
// receives a pointer to the vertex and may mutate it in place. Topology
// is unchanged.
//
// Ported top-down from C++ Manifold::Warp.
func (m *Manifold) Warp(fn func(*Vec3)) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	newImpl.Warp(fn)
	return wrap(newImpl.ToManifold())
}

// WarpBatch returns a copy of m with all vertices passed through fn at
// once as a single slice. fn may mutate the slice in place. Faster
// than Warp for whole-mesh transformations.
//
// Ported top-down from C++ Manifold::WarpBatch.
func (m *Manifold) WarpBatch(fn func([]Vec3)) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	newImpl.WarpBatch(fn)
	return wrap(newImpl.ToManifold())
}

// Revolve constructs a manifold by revolving a 2D crossSection
// (in the XY plane, +X side of the Y-axis) about the Y-axis. The
// resulting manifold has its axis along Z. revolveDegrees controls
// partial revolutions; circularSegments controls tessellation
// (0 = global default).
//
// Ported top-down from C++ Manifold::Revolve in src/constructors.cpp.
// New bridges: bridge.Triangulate (for front/back caps when not a full
// revolution) and MutableImpl.CreateHalfedges (turns the tri-index
// buffer into the Impl's halfedge representation).
func Revolve(crossSection Polygons, circularSegments int, revolveDegrees float64) *Manifold {
	// Polygon massaging: drop negative-x verts, interpolate at x=0 crossings.
	var polygons Polygons
	var radius float64
	for _, poly := range crossSection {
		i := 0
		for i < len(poly) && poly[i].X < 0 {
			i++
		}
		if i == len(poly) {
			continue
		}
		polygons = append(polygons, SimplePolygon{})
		start := i
		for {
			if poly[i].X >= 0 {
				polygons[len(polygons)-1] = append(polygons[len(polygons)-1], poly[i])
				if poly[i].X > radius {
					radius = poly[i].X
				}
			}
			next := i + 1
			if next == len(poly) {
				next = 0
			}
			if (poly[next].X < 0) != (poly[i].X < 0) {
				y := poly[next].Y - poly[next].X*
					(poly[i].Y-poly[next].Y)/
					(poly[i].X-poly[next].X)
				polygons[len(polygons)-1] = append(polygons[len(polygons)-1], Vec2{X: 0, Y: y})
			}
			i = next
			if i == start {
				break
			}
		}
	}

	if len(polygons) == 0 {
		return wrap(bridge.Invalid())
	}

	if revolveDegrees > 360 {
		revolveDegrees = 360
	}
	isFullRevolution := revolveDegrees == 360

	var nDivisions int
	if circularSegments > 2 {
		nDivisions = circularSegments
	} else {
		nDivisions = int(float64(GetCircularSegments(radius)) * revolveDegrees / 360)
	}

	dPhi := revolveDegrees / float64(nDivisions)
	nSlices := nDivisions
	if !isFullRevolution {
		nSlices = nDivisions + 1
	}

	// Build verts and tri-indices in Go slices, mirroring the C++ loops.
	var verts []Vec3
	var tris []int32
	var startPoses, endPoses []int

	for _, poly := range polygons {
		var nPosVerts, nRevolveAxisVerts int
		for _, pt := range poly {
			if pt.X > 0 {
				nPosVerts++
			} else {
				nRevolveAxisVerts++
			}
		}

		for polyVert := 0; polyVert < len(poly); polyVert++ {
			startPosIndex := len(verts)
			if !isFullRevolution {
				startPoses = append(startPoses, startPosIndex)
			}

			currPolyVertex := poly[polyVert]
			prevIdx := polyVert - 1
			if polyVert == 0 {
				prevIdx = len(poly) - 1
			}
			prevPolyVertex := poly[prevIdx]

			prevStartPosIndex := startPosIndex
			if polyVert == 0 {
				prevStartPosIndex += nRevolveAxisVerts + (nSlices * nPosVerts)
			}
			if prevPolyVertex.X == 0 {
				prevStartPosIndex += -1
			} else {
				prevStartPosIndex += -nSlices
			}

			for slice := 0; slice < nSlices; slice++ {
				phi := float64(slice) * dPhi
				if slice == 0 || currPolyVertex.X > 0 {
					verts = append(verts, Vec3{
						X: currPolyVertex.X * geom.Cosd(phi),
						Y: currPolyVertex.X * geom.Sind(phi),
						Z: currPolyVertex.Y,
					})
				}

				if isFullRevolution || slice > 0 {
					lastSlice := slice
					if slice == 0 {
						lastSlice = nDivisions
					}
					lastSlice--
					if currPolyVertex.X > 0 {
						var third int
						if prevPolyVertex.X == 0 {
							third = prevStartPosIndex
						} else {
							third = prevStartPosIndex + lastSlice
						}
						tris = append(tris,
							int32(startPosIndex+slice),
							int32(startPosIndex+lastSlice),
							int32(third))
					}
					if prevPolyVertex.X > 0 {
						var third int
						if currPolyVertex.X == 0 {
							third = startPosIndex
						} else {
							third = startPosIndex + slice
						}
						tris = append(tris,
							int32(prevStartPosIndex+lastSlice),
							int32(prevStartPosIndex+slice),
							int32(third))
					}
				}
			}
			if !isFullRevolution {
				endPoses = append(endPoses, len(verts)-1)
			}
		}
	}

	// Front + back triangulated caps for partial revolution.
	if !isFullRevolution {
		front := bridge.Triangulate(polygons, -1)
		for i := 0; i < len(front); i += 3 {
			tris = append(tris,
				int32(startPoses[front[i]]),
				int32(startPoses[front[i+1]]),
				int32(startPoses[front[i+2]]))
		}
		for i := 0; i < len(front); i += 3 {
			tris = append(tris,
				int32(endPoses[front[i+2]]),
				int32(endPoses[front[i+1]]),
				int32(endPoses[front[i]]))
		}
	}

	// Write into a fresh Impl and finalize — drilled steps mirror the
	// C++ tail of the algorithm.
	newImpl := bridge.NewMutableImpl()
	defer newImpl.Delete()
	newImpl.ResizeVerts(len(verts))
	copy(newImpl.Verts(), verts)
	createHalfedges(newImpl, tris)
	numTri := len(tris) / 3
	initializeOriginal(newImpl, numTri, false)
	calculateBBox(newImpl)
	setEpsilon(newImpl, -1, false)
	newImpl.SortGeometry()
	setNormalsAndCoplanar(newImpl)
	return wrap(newImpl.ToManifold())
}

// Slice returns the outline contours of the manifold at the given Z
// height as a set of 2D polygons.
//
// Ported top-down from C++:
//   Polygons Manifold::Slice(double height) const {
//     return GetCsgLeafNode().GetImpl()->Slice(height);
//   }
func (m *Manifold) Slice(height float64) Polygons {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	ph := impl.Slice(height)
	defer ph.Delete()
	result := make(Polygons, ph.NumPolys())
	for i := 0; i < ph.NumPolys(); i++ {
		result[i] = ph.Poly(i)
	}
	return result
}

// Project returns the XY-plane projection of the manifold as a set of
// 2D polygons. Often self-intersecting; pipe through CrossSection's
// positive fill rule for a clean result.
//
// Ported top-down from C++:
//   Polygons Manifold::Project() const {
//     return GetCsgLeafNode().GetImpl()->Project();
//   }
func (m *Manifold) Project() Polygons {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	ph := impl.Project()
	defer ph.Delete()
	result := make(Polygons, ph.NumPolys())
	for i := 0; i < ph.NumPolys(); i++ {
		result[i] = ph.Poly(i)
	}
	return result
}

// Hull returns the convex hull of the Manifold's vertices.
//
// Ported top-down from C++:
//   Manifold Manifold::Hull() const {
//     auto ctx = std::atomic_load(&ctx_);
//     auto srcImpl = GetCsgLeafNode(ctx.get()).GetImpl();
//     if (srcImpl->status_ != Error::NoError)
//       return PropagateStatus(srcImpl->status_);
//     auto impl = std::make_shared<Impl>();
//     impl->Hull(srcImpl->vertPos_, ctx.get());
//     return Manifold(std::make_shared<CsgLeafNode>(impl));
//   }
//
// The ExecutionContext (ctx_) field is not yet ported to Go; the Go body
// behaves as if ctx is null, which matches the default path in C++.
func (m *Manifold) Hull() *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := bridge.NewMutableImpl()
	defer newImpl.Delete()
	newImpl.Hull(impl.Verts())
	return wrap(newImpl.ToManifold())
}

// Refine subdivides each triangle into n×n smaller triangles.
//
// Ported top-down from C++ Manifold::Refine.
func (m *Manifold) Refine(n int) *Manifold {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	if n > 1 {
		newImpl.RefineN(n)
	}
	return wrap(newImpl.ToManifold())
}

// RefineToLength subdivides until each edge is at most length.
//
// Ported top-down from C++ Manifold::RefineToLength.
func (m *Manifold) RefineToLength(length float64) *Manifold {
	length = math.Abs(length)
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	if s := impl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	newImpl.RefineToLength(length)
	return wrap(newImpl.ToManifold())
}

// RefineToTolerance subdivides until smooth surfaces fit within tolerance.
// No-op when the source Manifold has no halfedge tangents.
//
// Ported top-down from C++ Manifold::RefineToTolerance.
func (m *Manifold) RefineToTolerance(tol float64) *Manifold {
	tol = math.Abs(tol)
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	s := impl.Scalars()
	if Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	newImpl := impl.Copy()
	defer newImpl.Delete()
	if s.HalfedgeTangentSize > 0 {
		newImpl.RefineToTolerance(tol)
	}
	return wrap(newImpl.ToManifold())
}

// MinkowskiSum returns the Minkowski sum of m and other.
//
// Ported top-down from C++ Manifold::MinkowskiSum.
func (m *Manifold) MinkowskiSum(other *Manifold) *Manifold {
	aImpl := bridge.GetImpl(m.h)
	defer aImpl.Delete()
	if s := aImpl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	bImpl := bridge.GetImpl(other.h)
	defer bImpl.Delete()
	if s := bImpl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	return wrap(aImpl.Minkowski(bImpl, false))
}

// MinkowskiDifference returns the Minkowski difference (erosion) of m by other.
//
// Ported top-down from C++ Manifold::MinkowskiDifference — same body as
// MinkowskiSum with the `inset` flag flipped to true.
func (m *Manifold) MinkowskiDifference(other *Manifold) *Manifold {
	aImpl := bridge.GetImpl(m.h)
	defer aImpl.Delete()
	if s := aImpl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	bImpl := bridge.GetImpl(other.h)
	defer bImpl.Delete()
	if s := bImpl.Scalars(); Error(s.Status) != NoError {
		return wrap(bridge.PropagateStatus(s.Status))
	}
	return wrap(aImpl.Minkowski(bImpl, true))
}

// BoundingBox returns the axis-aligned bounding box of the mesh.
//
// Ported from C++ Manifold::BoundingBox: the C++ side returns a cached
// bBox_ field on the Impl, kept in sync with vertPos_. Go computes the
// same answer by iterating the vertex buffer directly. An empty Manifold
// matches the C++ default (+inf min, -inf max).
func (m *Manifold) BoundingBox() Box {
	impl := bridge.GetImpl(m.h)
	defer impl.Delete()
	verts := impl.Verts()
	if len(verts) == 0 {
		inf := math.Inf(1)
		return Box{Min: Vec3{X: inf, Y: inf, Z: inf}, Max: Vec3{X: -inf, Y: -inf, Z: -inf}}
	}
	minV, maxV := verts[0], verts[0]
	for _, v := range verts[1:] {
		if v.X < minV.X {
			minV.X = v.X
		} else if v.X > maxV.X {
			maxV.X = v.X
		}
		if v.Y < minV.Y {
			minV.Y = v.Y
		} else if v.Y > maxV.Y {
			maxV.Y = v.Y
		}
		if v.Z < minV.Z {
			minV.Z = v.Z
		} else if v.Z > maxV.Z {
			maxV.Z = v.Z
		}
	}
	return Box{Min: minV, Max: maxV}
}

// Transform applies an arbitrary 3x4 affine transform.
//
// Ported top-down from C++:
//   Manifold Manifold::Transform(const mat3x4& m) const {
//     return Manifold(LoadPNode()->Transform(m));
//   }
// Each inner call goes through its own bridge: LoadPNode (private,
// reached via ManifoldBridge friend), CsgNode::Transform, and the
// private Manifold(shared_ptr<CsgNode>) constructor.
func (m *Manifold) Transform(t Mat3x4) *Manifold {
	node := bridge.LoadPNode(m.h)
	defer node.Delete()
	newNode := node.Transform(
		t[0][0], t[0][1], t[0][2],
		t[1][0], t[1][1], t[1][2],
		t[2][0], t[2][1], t[2][2],
		t[3][0], t[3][1], t[3][2],
	)
	defer newNode.Delete()
	return wrap(newNode.ToManifold())
}

// Tetrahedron returns a regular tetrahedron with edges of length 2·sqrt(2)
// centered on the origin.
//
// Ported top-down from C++:
//   Manifold Manifold::Tetrahedron() {
//     return Manifold(std::make_shared<Impl>(Impl::Shape::Tetrahedron));
//   }
// The Shape constructor uses the default mat3x4 (identity).
func Tetrahedron() *Manifold {
	identity := Mat3x4{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}}
	mi := newImplFromShape(shapeTetrahedron, identity)
	defer mi.Delete()
	return wrap(mi.ToManifold())
}

// Cube returns an axis-aligned box of the given dimensions. If center is
// true, the box is centered on the origin; otherwise one corner sits at
// the origin. A negative component or all-zero size returns an Invalid
// Manifold.
//
// Ported top-down from C++:
//   Manifold Manifold::Cube(vec3 size, bool center) {
//     if (size.x < 0.0 || size.y < 0.0 || size.z < 0.0 ||
//         la::length(size) == 0.) {
//       return Invalid();
//     }
//     mat3x4 m({{size.x,0,0},{0,size.y,0},{0,0,size.z}},
//              center ? (-size/2.0) : vec3(0.0));
//     return Manifold(std::make_shared<Impl>(Impl::Shape::Cube, m));
//   }
func Cube(size Vec3, center bool) *Manifold {
	if size.X < 0 || size.Y < 0 || size.Z < 0 || size.Length() == 0 {
		return wrap(bridge.Invalid())
	}
	var t Vec3
	if center {
		t = Vec3{X: -size.X / 2, Y: -size.Y / 2, Z: -size.Z / 2}
	}
	mi := newImplFromShape(shapeCube, Mat3x4{
		{size.X, 0, 0},
		{0, size.Y, 0},
		{0, 0, size.Z},
		{t.X, t.Y, t.Z},
	})
	defer mi.Delete()
	return wrap(mi.ToManifold())
}

// Translate returns a new Manifold translated by v. Ported from C++
// CsgNode::Translate: identity 3x3 with v in the translation column,
// then call Transform.
func (m *Manifold) Translate(v Vec3) *Manifold {
	return m.Transform(Mat3x4{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
		{v.X, v.Y, v.Z},
	})
}

// Scale returns a new Manifold scaled component-wise by v. Ported from
// C++ CsgNode::Scale: diagonal 3x3 with zero translation column, then
// call Transform.
func (m *Manifold) Scale(v Vec3) *Manifold {
	return m.Transform(Mat3x4{
		{v.X, 0, 0},
		{0, v.Y, 0},
		{0, 0, v.Z},
		{0, 0, 0},
	})
}

// Mirror returns a new Manifold reflected across the plane through the
// origin with the given normal. Zero-length normals return an empty
// Manifold (matching the C++ behavior).
//
// Ported from C++ Manifold::Mirror: normalize the input, build the
// Householder reflection matrix I - 2·n·nᵀ, call Transform with a
// zero translation column.
func (m *Manifold) Mirror(normal Vec3) *Manifold {
	lenSq := normal.X*normal.X + normal.Y*normal.Y + normal.Z*normal.Z
	if lenSq == 0 {
		return wrap(bridge.Empty())
	}
	invLen := 1.0 / math.Sqrt(lenSq)
	n := Vec3{X: normal.X * invLen, Y: normal.Y * invLen, Z: normal.Z * invLen}
	ni := [3]float64{n.X, n.Y, n.Z}

	var r geom.Mat3
	for col := 0; col < 3; col++ {
		for row := 0; row < 3; row++ {
			d := 0.0
			if col == row {
				d = 1.0
			}
			r[col][row] = d - 2.0*ni[col]*ni[row]
		}
	}
	return m.Transform(Mat3x4{
		{r[0][0], r[0][1], r[0][2]},
		{r[1][0], r[1][1], r[1][2]},
		{r[2][0], r[2][1], r[2][2]},
		{0, 0, 0},
	})
}

// Rotate returns a new Manifold rotated by the given Euler angles (in
// degrees) about the X, Y, then Z axes, in that order. Ported from
// C++ CsgNode::Rotate: compose Rz·Ry·Rx using degree-exact trig, call
// Transform with zero translation.
func (m *Manifold) Rotate(xDeg, yDeg, zDeg float64) *Manifold {
	rX := geom.Mat3{
		{1, 0, 0},
		{0, geom.Cosd(xDeg), geom.Sind(xDeg)},
		{0, -geom.Sind(xDeg), geom.Cosd(xDeg)},
	}
	rY := geom.Mat3{
		{geom.Cosd(yDeg), 0, -geom.Sind(yDeg)},
		{0, 1, 0},
		{geom.Sind(yDeg), 0, geom.Cosd(yDeg)},
	}
	rZ := geom.Mat3{
		{geom.Cosd(zDeg), geom.Sind(zDeg), 0},
		{-geom.Sind(zDeg), geom.Cosd(zDeg), 0},
		{0, 0, 1},
	}
	r := rZ.Mul(rY).Mul(rX)
	return m.Transform(Mat3x4{
		{r[0][0], r[0][1], r[0][2]},
		{r[1][0], r[1][1], r[1][2]},
		{r[2][0], r[2][1], r[2][2]},
		{0, 0, 0},
	})
}

// wrap takes ownership of a raw handle and attaches a finalizer that releases
// the underlying C++ Manifold when the wrapper is garbage-collected. Callers
// must not delete the handle themselves once it has been wrapped.
func wrap(h *handle.Manifold) *Manifold {
	m := &Manifold{h: h}
	runtime.SetFinalizer(m, func(m *Manifold) { bridge.DeleteManifold(m.h) })
	return m
}
