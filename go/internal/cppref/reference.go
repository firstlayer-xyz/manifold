// reference.go is the public-API-oracle layer of package cppref: every function
// here calls straight into the stable public C API of libmanifoldc, providing the
// trusted golden values the differential tests compare against. (Keep these calling
// the public API — never reimplement them on top of the internals shim, or they
// stop being an independent check.)
//
// This file carries NO #cgo directives: the link/include configuration lives once
// in bridge.go and applies package-globally, so libmanifold is linked a single time.
package cppref

// #include <stdlib.h>
// #include <manifold/manifoldc.h>
import "C"

import (
	"unsafe"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/handle"
)

// Tetrahedron constructs a unit tetrahedron via the C++ public API and
// returns an unowned handle to it. Caller must DeleteManifold.
func Tetrahedron() *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_tetrahedron(unsafe.Pointer(mem))
	return handle.NewManifold(unsafe.Pointer(p))
}

func NumVert(h *handle.Manifold) int {
	return int(C.manifold_num_vert((*C.ManifoldManifold)(h.Ptr())))
}

func NumTri(h *handle.Manifold) int {
	return int(C.manifold_num_tri((*C.ManifoldManifold)(h.Ptr())))
}

func NumEdge(h *handle.Manifold) int {
	return int(C.manifold_num_edge((*C.ManifoldManifold)(h.Ptr())))
}

func IsEmpty(h *handle.Manifold) bool {
	return C.manifold_is_empty((*C.ManifoldManifold)(h.Ptr())) != 0
}

func Volume(h *handle.Manifold) float64 {
	return float64(C.manifold_volume((*C.ManifoldManifold)(h.Ptr())))
}

func SurfaceArea(h *handle.Manifold) float64 {
	return float64(C.manifold_surface_area((*C.ManifoldManifold)(h.Ptr())))
}

func NumProp(h *handle.Manifold) int {
	return int(C.manifold_num_prop((*C.ManifoldManifold)(h.Ptr())))
}

func NumPropVert(h *handle.Manifold) int {
	return int(C.manifold_num_prop_vert((*C.ManifoldManifold)(h.Ptr())))
}

func Genus(h *handle.Manifold) int {
	return int(C.manifold_genus((*C.ManifoldManifold)(h.Ptr())))
}

func GetTolerance(h *handle.Manifold) float64 {
	return float64(C.manifold_get_tolerance((*C.ManifoldManifold)(h.Ptr())))
}

func OriginalID(h *handle.Manifold) int {
	return int(C.manifold_original_id((*C.ManifoldManifold)(h.Ptr())))
}

func AsOriginal(h *handle.Manifold) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_as_original(unsafe.Pointer(mem), (*C.ManifoldManifold)(h.Ptr()))
	return handle.NewManifold(unsafe.Pointer(p))
}

func SetTolerance(h *handle.Manifold, tol float64) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_set_tolerance(unsafe.Pointer(mem), (*C.ManifoldManifold)(h.Ptr()), C.double(tol))
	return handle.NewManifold(unsafe.Pointer(p))
}

func TrimByPlane(h *handle.Manifold, nx, ny, nz, offset float64) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_trim_by_plane(unsafe.Pointer(mem), (*C.ManifoldManifold)(h.Ptr()),
		C.double(nx), C.double(ny), C.double(nz), C.double(offset))
	return handle.NewManifold(unsafe.Pointer(p))
}

func Union(a, b *handle.Manifold) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_union(unsafe.Pointer(mem), (*C.ManifoldManifold)(a.Ptr()), (*C.ManifoldManifold)(b.Ptr()))
	return handle.NewManifold(unsafe.Pointer(p))
}

func Difference(a, b *handle.Manifold) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_difference(unsafe.Pointer(mem), (*C.ManifoldManifold)(a.Ptr()), (*C.ManifoldManifold)(b.Ptr()))
	return handle.NewManifold(unsafe.Pointer(p))
}

func Intersection(a, b *handle.Manifold) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_intersection(unsafe.Pointer(mem), (*C.ManifoldManifold)(a.Ptr()), (*C.ManifoldManifold)(b.Ptr()))
	return handle.NewManifold(unsafe.Pointer(p))
}

func Sphere(radius float64, circularSegments int) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_sphere(unsafe.Pointer(mem), C.double(radius), C.int(circularSegments))
	return handle.NewManifold(unsafe.Pointer(p))
}

func Hull(h *handle.Manifold) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_hull(unsafe.Pointer(mem), (*C.ManifoldManifold)(h.Ptr()))
	return handle.NewManifold(unsafe.Pointer(p))
}

func Refine(h *handle.Manifold, n int) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_refine(unsafe.Pointer(mem), (*C.ManifoldManifold)(h.Ptr()), C.int(n))
	return handle.NewManifold(unsafe.Pointer(p))
}

func RefineToLength(h *handle.Manifold, length float64) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_refine_to_length(unsafe.Pointer(mem), (*C.ManifoldManifold)(h.Ptr()), C.double(length))
	return handle.NewManifold(unsafe.Pointer(p))
}

func RefineToTolerance(h *handle.Manifold, tol float64) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_refine_to_tolerance(unsafe.Pointer(mem), (*C.ManifoldManifold)(h.Ptr()), C.double(tol))
	return handle.NewManifold(unsafe.Pointer(p))
}

func MinkowskiSum(a, b *handle.Manifold) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_minkowski_sum(unsafe.Pointer(mem), (*C.ManifoldManifold)(a.Ptr()), (*C.ManifoldManifold)(b.Ptr()))
	return handle.NewManifold(unsafe.Pointer(p))
}

func MinkowskiDifference(a, b *handle.Manifold) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_minkowski_difference(unsafe.Pointer(mem), (*C.ManifoldManifold)(a.Ptr()), (*C.ManifoldManifold)(b.Ptr()))
	return handle.NewManifold(unsafe.Pointer(p))
}

func Status(h *handle.Manifold) int {
	return int(C.manifold_status((*C.ManifoldManifold)(h.Ptr())))
}

func Cylinder(height, radiusLow, radiusHigh float64, circularSegments int, center bool) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	c := C.int(0)
	if center {
		c = 1
	}
	p := C.manifold_cylinder(unsafe.Pointer(mem),
		C.double(height), C.double(radiusLow), C.double(radiusHigh),
		C.int(circularSegments), c)
	return handle.NewManifold(unsafe.Pointer(p))
}

func Cube(x, y, z float64, center bool) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	c := C.int(0)
	if center {
		c = 1
	}
	p := C.manifold_cube(unsafe.Pointer(mem), C.double(x), C.double(y), C.double(z), c)
	return handle.NewManifold(unsafe.Pointer(p))
}

func Transform(h *handle.Manifold,
	x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4 float64,
) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_transform(
		unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()),
		C.double(x1), C.double(y1), C.double(z1),
		C.double(x2), C.double(y2), C.double(z2),
		C.double(x3), C.double(y3), C.double(z3),
		C.double(x4), C.double(y4), C.double(z4),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// DeleteManifold is defined in bridge.go (same cppref package).

// Translate is the C++-side reference implementation: a direct call to
// manifold_translate. Used as the oracle that the Go-side Translate is
// validated against.
func Translate(h *handle.Manifold, v geom.Vec3) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_translate(
		unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()),
		C.double(v.X), C.double(v.Y), C.double(v.Z),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// Scale is the C++-side reference implementation for Manifold::Scale.
func Scale(h *handle.Manifold, v geom.Vec3) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_scale(
		unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()),
		C.double(v.X), C.double(v.Y), C.double(v.Z),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// Rotate is the C++-side reference implementation for Manifold::Rotate.
// Angles are in degrees.
func Rotate(h *handle.Manifold, xDeg, yDeg, zDeg float64) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_rotate(
		unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()),
		C.double(xDeg), C.double(yDeg), C.double(zDeg),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// Revolve is the C++-side reference for static Manifold::Revolve.
func Revolve(polys [][]geom.Vec2, circularSegments int, revolveDegrees float64) *handle.Manifold {
	// Build ManifoldPolygons via manifold_simple_polygon + manifold_polygons.
	var simples []*C.ManifoldSimplePolygon
	for _, poly := range polys {
		spMem := C.manifold_alloc_simple_polygon()
		var pp unsafe.Pointer
		if len(poly) > 0 {
			pp = unsafe.Pointer(&poly[0])
		}
		sp := C.manifold_simple_polygon(unsafe.Pointer(spMem),
			(*C.ManifoldVec2)(pp), C.size_t(len(poly)))
		simples = append(simples, sp)
	}
	defer func() {
		for _, sp := range simples {
			C.manifold_delete_simple_polygon(sp)
		}
	}()
	polyMem := C.manifold_alloc_polygons()
	var spPtr **C.ManifoldSimplePolygon
	if len(simples) > 0 {
		spPtr = (**C.ManifoldSimplePolygon)(unsafe.Pointer(&simples[0]))
	}
	polysC := C.manifold_polygons(unsafe.Pointer(polyMem), spPtr, C.size_t(len(simples)))
	defer C.manifold_delete_polygons(polysC)

	mem := C.manifold_alloc_manifold()
	p := C.manifold_revolve(unsafe.Pointer(mem), polysC,
		C.int(circularSegments), C.double(revolveDegrees))
	return handle.NewManifold(unsafe.Pointer(p))
}

// CalculateCurvature is the C++-side reference for
// Manifold::CalculateCurvature.
func CalculateCurvature(h *handle.Manifold, gaussianIdx, meanIdx int) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_calculate_curvature(unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()),
		C.int(gaussianIdx), C.int(meanIdx))
	return handle.NewManifold(unsafe.Pointer(p))
}

// CalculateNormals is the C++-side reference for Manifold::CalculateNormals.
func CalculateNormals(h *handle.Manifold, normalIdx int, minSharpAngle float64) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_calculate_normals(unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()),
		C.int(normalIdx), C.double(minSharpAngle))
	return handle.NewManifold(unsafe.Pointer(p))
}

// Simplify is the C++-side reference for Manifold::Simplify.
func Simplify(h *handle.Manifold, tol float64) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_simplify(unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()), C.double(tol))
	return handle.NewManifold(unsafe.Pointer(p))
}

// ReserveIDs is the C++-side reference for static Manifold::ReserveIDs.
func ReserveIDs(n uint32) uint32 {
	return uint32(C.manifold_reserve_ids(C.uint32_t(n)))
}

// HullPts is the C++-side reference for static Manifold::Hull(pts).
func HullPts(pts []geom.Vec3) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	var pp unsafe.Pointer
	if len(pts) > 0 {
		pp = unsafe.Pointer(&pts[0])
	}
	p := C.manifold_hull_pts(unsafe.Pointer(mem),
		(*C.ManifoldVec3)(pp), C.size_t(len(pts)))
	return handle.NewManifold(unsafe.Pointer(p))
}

// MinGap is the C++-side reference implementation for Manifold::MinGap.
func MinGap(a, b *handle.Manifold, searchLength float64) float64 {
	return float64(C.manifold_min_gap(
		(*C.ManifoldManifold)(a.Ptr()),
		(*C.ManifoldManifold)(b.Ptr()),
		C.double(searchLength)))
}

// Mirror is the C++-side reference implementation for Manifold::Mirror.
func Mirror(h *handle.Manifold, normal geom.Vec3) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_mirror(
		unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()),
		C.double(normal.X), C.double(normal.Y), C.double(normal.Z),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// ReadOBJ is the C++-side reference for Manifold::ReadOBJ. obj is the
// content of the OBJ file (not a path); a NUL terminator is appended
// internally for the C API.
func ReadOBJ(obj string) *handle.Manifold {
	cstr := C.CString(obj)
	defer C.free(unsafe.Pointer(cstr))
	mem := C.manifold_alloc_manifold()
	p := C.manifold_read_obj(unsafe.Pointer(mem), cstr)
	return handle.NewManifold(unsafe.Pointer(p))
}

// Split is the C++-side reference for Manifold::Split, returning the
// intersection and difference halves.
func Split(a, b *handle.Manifold) (first, second *handle.Manifold) {
	memFirst := C.manifold_alloc_manifold()
	memSecond := C.manifold_alloc_manifold()
	pair := C.manifold_split(unsafe.Pointer(memFirst), unsafe.Pointer(memSecond),
		(*C.ManifoldManifold)(a.Ptr()), (*C.ManifoldManifold)(b.Ptr()))
	return handle.NewManifold(unsafe.Pointer(pair.first)),
		handle.NewManifold(unsafe.Pointer(pair.second))
}

// SplitByPlane is the C++-side reference for Manifold::SplitByPlane.
func SplitByPlane(h *handle.Manifold, nx, ny, nz, offset float64) (first, second *handle.Manifold) {
	memFirst := C.manifold_alloc_manifold()
	memSecond := C.manifold_alloc_manifold()
	pair := C.manifold_split_by_plane(unsafe.Pointer(memFirst), unsafe.Pointer(memSecond),
		(*C.ManifoldManifold)(h.Ptr()),
		C.double(nx), C.double(ny), C.double(nz), C.double(offset))
	return handle.NewManifold(unsafe.Pointer(pair.first)),
		handle.NewManifold(unsafe.Pointer(pair.second))
}

// BoundingBox returns the axis-aligned bounding box of the Manifold as
// (min, max) corners. Used by differential tests for semantic geometry
// comparison.
func BoundingBox(h *handle.Manifold) (min, max geom.Vec3) {
	boxMem := C.manifold_alloc_box()
	box := C.manifold_bounding_box(unsafe.Pointer(boxMem), (*C.ManifoldManifold)(h.Ptr()))
	defer C.manifold_delete_box(box)

	mn := C.manifold_box_min(box)
	mx := C.manifold_box_max(box)
	return geom.Vec3{X: float64(mn.x), Y: float64(mn.y), Z: float64(mn.z)},
		geom.Vec3{X: float64(mx.x), Y: float64(mx.y), Z: float64(mx.z)}
}
