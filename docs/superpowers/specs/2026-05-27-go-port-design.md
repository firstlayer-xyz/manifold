# Manifold Go Port — Design

**Date:** 2026-05-27
**Status:** Active

## Goal

Port the entire Manifold library to pure Go. The end state is a Go module with no C/C++ source, no cgo, and no link against `libmanifold`. cgo is used only as temporary scaffolding during the port.

## Strategy: outside-in, function-by-function

1. Start at the public C++ API surface (`include/manifold/manifold.h`, `cross_section.h`).
2. Port one public function at a time, in Go.
3. Inside each newly-ported Go function, any call to inner C++ machinery goes through a cgo bridge.
4. When a bridge function isn't already exposed by `bindings/c/manifoldc`, write a small `.cpp` shim that exposes the inner C++ symbol with a C-callable signature.
5. As deeper layers get ported, bridges to those layers shrink; new bridges appear one layer further in. The cgo footprint moves inward and eventually disappears.

## Repo layout

```
manifold/
└── go/                              (top-level, self-contained for future extraction)
    ├── go.mod                       module github.com/firstlayer-xyz/manifold/go
    ├── manifold.go                  public Go API (grows over time)
    ├── manifold_test.go
    ├── bridge/                      cgo bridges to C++ internals
    │   ├── bridge.go                #cgo directives, opaque handle types
    │   ├── bridge.cpp               custom shims for symbols manifoldc doesn't expose
    │   └── bridge.h
    ├── reference/                   test-only oracle: cgo → libmanifoldc
    │   └── reference.go
    └── internal/diff/               differential comparison primitives
        └── compare.go
```

- All `import "C"` is confined to `bridge/` and `reference/`. The public `manifold` package source stays cgo-free, so "is this function still calling C?" reduces to "does it import `bridge`?"
- `reference/` is deleted at the end of the port. `bridge/` shrinks to zero as inner layers are ported.

## Build flow

- Prerequisite: C++ build artifacts present at `../build/bindings/c/libmanifoldc.{dylib,so}` and headers at `../bindings/c/include/`.
- cgo flags in `bridge/bridge.go` and `reference/reference.go` use `${SRCDIR}`-relative paths.
- For runtime loading on macOS: `-Wl,-rpath,@loader_path/../../../build/bindings/c` (or absolute path in dev). Linux: `-Wl,-rpath,...` similarly.
- No CMake integration in Stage 0. Developer responsibility: build C++ first, then `cd go && go test ./...`.

## Testing strategy: differential during port, translated tests after

- **During port:** every ported function gets a differential test. The test calls the Go implementation AND the `reference/` cgo path (which calls libmanifoldc's `manifold_*` function directly), then compares results with the appropriate strategy.
- **After port:** when a function is fully Go-native and trusted, replace its differential test with translated unit tests adapted from `test/*.cpp`. The `reference/` cgo path can be deleted from that function's test once the port is fully trusted.

### Comparison strategy (because doubles)

Bit-identical equality is not a goal. C++ and Go diverge in the LSBs from math library differences, summation order, FMA usage. The diff package provides:

- `ScalarAlmostEqual(a, b float64, absTol, relTol float64) bool` — both absolute and relative tolerance.
- `BoundingBoxAlmostEqual` — corner-wise scalar compare.
- `ManifoldAlmostEqual` — semantic compare: volume, surface area, bounding box, genus. NOT vertex-position or triangulation structural compare (coplanar regions triangulate differently between implementations).
- Per-test tolerances. Booleans get looser tolerances than `Translate`.

## Stage plan

**Stage 0 — Scaffold only, no port.**
- Create `go/` directory, `go.mod`.
- Create `bridge/bridge.go` with cgo flags linking `libmanifoldc`.
- One test calling `manifold_alloc_manifold` + `manifold_delete_manifold` to validate the build/link path. No semantics ported.

**Stage 1 — First port: `NumVert()`.**
- Construct a Manifold via the `reference/` path (cgo → `manifold_tetrahedron`).
- Implement `NumVert()` in Go (will need a bridge to read the impl's vertex count, since this is now an internal access).
- Differential test: assert Go's `NumVert` matches `reference/`'s `manifold_num_vert` on the same Manifold.

**Stage 2+ — Subsequent ports.**
- Next functions chosen layer-by-layer. Driven by what's small and what unblocks subsequent work. Likely candidates after `NumVert`: other accessors (`NumTri`, `NumEdge`, `IsEmpty`), then `BoundingBox`, then static factories (`Tetrahedron`, `Cube`).

## Out of scope

- TypeScript/JavaScript port. WASM port. Python port. This is Go only.
- Code generation. We hand-write the first bridges; if a pattern emerges, consider codegen later.
- Performance parity. Correctness first; performance work happens once functions are pure Go.
