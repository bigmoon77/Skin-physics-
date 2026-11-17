# Skin Physics

Skin Physics is a GPU-oriented C++ library for real-time Soft Body and Cloth simulation on Direct3D 11.

The repository is being implemented in stages. Phase 1 importers and the Phase 2 Physics Mesh, skin-weight transfer and cache foundations are implemented; resolution-driven tetrahedralization settings/tests remain. Phase 3 currently provides a CPU reference for Render Vertex Binding/feedback, and Phase 4 provides fixed-step time control. D3D11 runtime dispatch and later solvers remain in progress.

## Current implementation

- C++17, Visual Studio solution and project files.
- Common asset model for meshes, vertices, materials, named vertex groups, nodes, skins and animation clips.
- OBJ importer with polygon triangulation, negative indices, smoothing groups and basic MTL material factors.
- glTF 2.0 / GLB importer with triangle, triangle-strip and triangle-fan primitives, PBR material factors, node hierarchy, up to eight skin influences, inverse bind matrices, and STEP / LINEAR / CUBICSPLINE TRS animation tracks.
- Common model validation and diagnostics.
- Physics Mesh builder: tetrahedron orientation correction, rest matrices / shape gradients, volume and element quality, boundary extraction, vertex-to-tet CSR, and lumped mass.
- ASCII Gmsh MSH 2.2 reader and optional external fTetWild process adapter (not linked into runtime).
- Closest-tetrahedron / boundary-triangle Render Vertex Binding and CPU feedback with local-basis Rest offsets; UV, color, material and index data are preserved.
- Fixed-step simulation clock with Play / Pause / Stop / Reset / single-step, backlog preservation and callback-failure rollback.
- CPU reference Cloth solver with XPBD Stretch, Shear and signed Bend constraints; Bend uses an analytical chain-rule gradient validated against central finite differences.
- Importer-only dependency: cgltf 1.15 (MIT), pinned in third_party/cgltf.

OBJ has no standard unit metadata. Callers must set has_obj_unit_scale = true and provide obj_unit_scale in meters per OBJ unit.

## Build

Open SkinPhysics.sln in Visual Studio 2022 and build the x64 configuration.

The solution contains:
- SkinPhysics: static library.
- SkinPhysicsTests: console test executable.

The CI workflow builds the Visual Studio solution and runs the test executable on Windows. It runs on pushes to the implementation branch and pull requests targeting main.

## Basic usage

\`\`\`cpp
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
\`\`\`

For glTF and GLB, the importer reads the glTF meter-based coordinates and does not require an OBJ unit scale.

## Offline tetrahedralization

The optional fTetWild adapter launches an externally installed FloatTetwild executable on Windows. It writes a temporary OBJ, requests ASCII Gmsh MSH 2.2 output, then validates the raw tetrahedral volume mesh. The executable path is supplied through `ftetwild_settings::executable_path`; fTetWild is not bundled and is not a runtime dependency. See `third_party/OPTIONAL_TOOLS.md` for its pinned reference commit and MPL-2.0 license.

The Physics Mesh builder rejects degenerate / non-manifold volume meshes, repairs negative tetrahedron orientation, computes rest shape gradients and lumped masses, and builds boundary faces and vertex-to-tetrahedron CSR adjacency.

## Importer limits in this milestone

- Morph targets and morph-weight animation are rejected.
- Draco-compressed geometry is rejected because no Draco decoder is bundled.
- Meshopt-compressed buffer views are rejected because no meshoptimizer decoder is bundled.
- Only triangle, triangle-strip and triangle-fan primitives are imported.
- Texture image bytes / URIs are preserved as asset references; image transcoding and GPU texture creation belong to later render-resource stages.
- OBJ assumes the caller's coordinates are already right-handed Y-up and only applies the explicit unit scale.

These limits are diagnosed instead of silently producing incomplete geometry.


## Golden test corpus and phase verification

Persistent inputs and reviewed expected results are stored in `tests/fixtures/`. The phase/test mapping is maintained in `tests/fixtures/manifest.json`, while the verification rules and phase exit gates are described in `docs/testing_strategy.md`.

Phases 1 and the mesh/skin/cache portion of Phase 2 load persistent fixtures in the automated test executable. Phase 2 target-resolution tests remain pending. Phase 3 runs CPU reference tests for interior/exterior embeddings, Rest Pose reconstruction, invalid-distance rejection and attribute preservation; the D3D11 GPU feedback path remains pending. Phase 4 runs fixed-step, Pause/Step/Reset, overload-backlog and callback-failure tests; scenario JSON and GPU telemetry remain pending. Phase 5 runs CPU XPBD Cloth topology, Rest geometry, Stretch/Shear/Bend compliance and Bend-gradient reference tests; D3D11 kernels remain pending. Phases 6–11 still have fixture data only. The manifest records partial versus automated coverage explicitly.

Run `SkinPhysicsTests.exe` from Visual Studio or the command line. Each implemented test prints a `[PASS]` entry with its phase/test ID. Fixture paths are resolved relative to the test source, independent of the process working directory.
