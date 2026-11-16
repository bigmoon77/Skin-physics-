# Skin Physics

Skin Physics is a GPU-oriented C++ library for real-time Soft Body and Cloth simulation on Direct3D 11.

The repository is implemented in stages. Phase 1 (common asset model and importers) is complete. Phase 2 (offline tetrahedralization, Physics Mesh construction, skin-weight transfer and cache) is in progress.

## Current implementation

- C++17, Visual Studio 2022 solution and project files.
- Common asset model for meshes, vertices, materials, named vertex groups, nodes, skins and animation clips.
- OBJ importer with polygon triangulation, negative indices, smoothing groups and basic MTL material factors.
- glTF 2.0 / GLB importer with triangle, triangle-strip and triangle-fan primitives, PBR material factors, node hierarchy, up to eight skin influences, inverse bind matrices, and STEP / LINEAR / CUBICSPLINE TRS animation tracks.
- Common model validation and diagnostics.
- Physics Mesh builder: tetrahedron orientation correction, rest matrices / shape gradients, volume and element quality, boundary extraction, vertex-to-tet CSR, and lumped mass.
- ASCII Gmsh MSH 2.2 reader and optional external fTetWild process adapter (not linked into runtime).
- Physics skin-weight transfer: closest-triangle barycentric interpolation on the boundary and PCG-based positive graph-Laplacian harmonic extension for interior vertices.
- Physics Asset Cache: explicit little-endian layout, exact source/generator identity matching, payload checksum, temporary-file write and replacement, corruption/stale-cache diagnostics.
- Importer-only dependency: cgltf 1.15 (MIT), pinned in third_party/cgltf.

OBJ has no standard unit metadata. Callers must set has_obj_unit_scale = true and provide obj_unit_scale in meters per OBJ unit.

## Build

Open SkinPhysics.sln in Visual Studio 2022 and build the x64 configuration.

The solution contains:
- SkinPhysics: static library.
- SkinPhysicsTests: console test executable.

The Windows CI workflow builds the Visual Studio solution and runs the test executable. It runs on pushes to the implementation branch and pull requests targeting main.

## Basic usage

```cpp
#include "skin_physics/asset/importer.hpp"

skin_physics::asset::import_options options;
options.has_obj_unit_scale = true;
options.obj_unit_scale = 0.01f; // OBJ coordinates are centimeters.

const skin_physics::asset::import_result result =
    skin_physics::asset::import_model("character.obj", options);
if (!result.success)
{
    for (const skin_physics::asset::diagnostic& item : result.diagnostics)
    {
        // Route item.message / item.code to the application's diagnostic UI.
    }
}
```

For glTF and GLB, the importer reads glTF meter-based coordinates and does not require an OBJ unit scale.

## Offline tetrahedralization

The optional fTetWild adapter launches an externally installed FloatTetwild executable on Windows. It writes a temporary OBJ, requests ASCII Gmsh MSH 2.2 output, then validates the raw tetrahedral volume mesh. The executable path is supplied through `ftetwild_settings::executable_path`; fTetWild is not bundled and is not a runtime dependency. See `third_party/OPTIONAL_TOOLS.md` for its pinned reference commit and MPL-2.0 license.

The Physics Mesh builder rejects degenerate / non-manifold volume meshes, repairs negative tetrahedron orientation, computes rest shape gradients and lumped masses, and builds boundary faces and vertex-to-tetrahedron CSR adjacency.

## Skin weights and cache

For a skinned Render Mesh, call `transfer_skin_weights` after the Physics Mesh has been built. The boundary vertices are mapped to the closest valid Render Mesh triangle and receive barycentrically interpolated bone weights. Interior vertices are solved with a Jacobi-preconditioned conjugate-gradient method over the positive edge-weighted graph Laplacian. The result is checked for connectivity, convergence and normalized non-negative weights; a failed transfer does not replace the existing Physics Mesh weights.

`save_physics_asset_cache` stores the generated rest data and optional Physics Mesh skin weights. `load_physics_asset_cache` only accepts an exact cache identity match, including source asset hash, generator identity/options and layout versions. On mismatch or corruption it returns `needs_rebuild = true`; it never silently treats a stale cache as current.

The current cache payload covers Physics Mesh rest data, boundary faces, vertex-to-tetrahedron CSR, masses and transferred skin weights. Render Vertex Binding, Cloth constraints, Collision Proxy and paint data are added in later phases; the cache schema must be advanced when those payload sections are introduced.

## Importer limits in this milestone

- Morph targets and morph-weight animation are rejected.
- Draco-compressed geometry is rejected because no Draco decoder is bundled.
- Meshopt-compressed buffer views are rejected because no meshoptimizer decoder is bundled.
- Only triangle, triangle-strip and triangle-fan primitives are imported.
- Texture image bytes / URIs are preserved as asset references; image transcoding and GPU texture creation belong to later render-resource stages.
- OBJ assumes the caller's coordinates are already right-handed Y-up and only applies the explicit unit scale.

These limits are diagnosed instead of silently producing incomplete geometry.
