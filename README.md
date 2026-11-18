# Skin Physics

Skin Physics is a GPU-oriented C++ library for real-time Soft Body and Cloth simulation on Direct3D 11.

The repository is being implemented in stages. Asset import, Physics Mesh construction, CPU reference solvers and fixture-based GPU verification kernels are present. A complete production simulation runtime that connects every stage is still in progress; phase status and known gaps are tracked in `tests/fixtures/manifest.json`.

The GPU Runtime / Library integration design is being revised against `docs/requirements.md` 1.1. The design revision is on branch `feature/integrated-gpu-runtime-design`; it does not claim that the production asynchronous Runtime has been implemented. The original `feature/phase1-importer` implementation remains the source baseline.

## Current implementation

- C++17, Visual Studio solution and project files.
- Common asset model for meshes, vertices, materials, named vertex groups, nodes, skins and animation clips.
- OBJ importer with polygon triangulation, negative indices, smoothing groups and basic MTL material factors.
- glTF 2.0 / GLB importer with triangle, triangle-strip and triangle-fan primitives, PBR material factors, node hierarchy, up to eight skin influences, inverse bind matrices, and STEP / LINEAR / CUBICSPLINE TRS animation tracks.
- Common model validation and diagnostics.
- Physics Mesh builder: tetrahedron orientation correction, rest matrices / shape gradients, volume and element quality, boundary extraction, vertex-to-tet CSR, and lumped mass.
- ASCII Gmsh MSH 2.2 reader and optional external fTetWild process adapter (not linked into runtime). The adapter supports mutually exclusive target-edge-length and target-tetrahedron-count modes, records actual element/edge/quality statistics and rejects output above configured count/quality limits.
- Closest-tetrahedron / boundary-triangle Render Vertex Binding and CPU feedback with local-basis Rest offsets; UV, color, material and index data are preserved.
- Fixed-step simulation clock with Play / Pause / Stop / Reset / single-step, backlog preservation and callback-failure rollback.
- CPU reference Cloth solver with XPBD Stretch, Shear and signed Bend constraints; Bend uses an analytical chain-rule gradient validated against central finite differences.
- CPU single-tetrahedron Corotational FEM evaluation with proper polar rotation / Jacobi-SVD fallback, deformation gradient, first Piola stress, internal forces and finite inversion penalty diagnostics.
- CPU collision reference routines for vertex-triangle and edge-edge closest features, weighted VT position correction, spatial-cell broadphase with pair filters / capacity overflow handling, point-triangle Conservative Advancement CCD and atomic initial-penetration repair.
- CPU animation target sampling with translation velocity, quaternion shortest-path Slerp and teleport velocity reset.
- CPU attribute Paint / selected-region Smooth / Average / Erase, with versioned checksum-validated binary weight serialization.
- Hardware-independent performance sample collector with CPU distributions, optional externally supplied GPU pass timings, scenario-hash matching and correctness-invariant gates.
- Scenario JSON and Run Result validation/persistence, semantic round-trip, unknown extension preservation and comparability by scenario hash / asset identity.
- Transactional simulation-stage callback pipeline: callbacks run in design order against a working copy; failed or non-finite output does not commit.
- WARP-tested HLSL XPBD Stretch / Shear / Bend constraints, positive-Jacobian Corotational FEM element evaluation, VT/EE narrowphase, host-built BVH traversal, point/static-triangle CCD, animation targets and attribute painting.
- D3D11 Timestamp Query ring with non-blocking collection, DXGI local-memory usage/budget query, and an ordered Compute Pass Graph with SRV/UAV alias validation and explicit CS resource unbinding.
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

Persistent inputs and reviewed expected results are in \`tests/fixtures/\`. The phase/test mapping is in \`tests/fixtures/manifest.json\`, and the verification methodology is in \`docs/testing_strategy.md\`.

Windows CI runs the CPU test suite and D3D11 WARP readback tests. Current WARP coverage includes Render Feedback, all three Cloth XPBD constraint types, positive-J Corotational FEM, VT/EE contacts, host-built BVH traversal and overflow, point/static-triangle CCD, animation/teleport, Paint/Smooth/Average/Erase, Timestamp Query/DXGI telemetry, and a two-pass Compute Pass Graph. Scenario JSON round-trip, Run Result validation/comparison and transactional pipeline behavior are also tested.

WARP is a software D3D11 device. These tests establish shader compilation, dispatch, buffer layout and numerical agreement with references; they are not physical-GPU performance measurements. Phase 2 fTetWild execution, full solver runtime connection, GPU BVH construction, global FEM integration, complete cloth iterations, moving-primitive CCD, editor workflows and physical-GPU performance profiles remain open.


## GPU Runtime integration design

The current integration proposal is documented in:

- [Basic design](docs/basic_design.md) 1.3
- [Detailed design](docs/detailed_design.md) 1.4
- [Design closure / v1 decisions](docs/design_closure.md)
- [Requirements](docs/requirements.md) 1.1
- [GPU Runtime / Library integration](docs/runtime_integration_design.md) 1.1
- [GPU Runtime UML](docs/uml/phase12_gpu_runtime.md)
- [Numerical acceptance profile](docs/numerical_acceptance_profile_v1.json)
- [Runtime test gates](docs/testing_strategy.md), R0-R6

The v1 Runtime preserves the Skin Physics model: multiple Mesh Components per World, mixed Soft Body / Cloth, Corotational FEM for Soft Body, XPBD for Cloth and Contact, bidirectional VT / EE collision, and self-collision. The implementation target remains the existing C++17 Visual Studio static library and D3D11. The Host owns the D3D11 Device / Immediate Context; Runtime owns GPU physics resources and asynchronous tick commit state.

This proposal intentionally does not import another project's single-body / 200k-particle / 40-FPS / fixed-1/120-second requirements. Skin Physics keeps its configurable Fixed Delta Time and workload-dependent 3-40 FPS evaluation range.

**Status: design closure candidate / independent critical review required.** The v1 support scope, initial numerical / collision parameters, Capacity Growth, async multi-World scheduling, and Render Candidate / Lease lifetime are consolidated in docs/design_closure.md. This is documentation only: GPU Runtime integration, global force / contact scheduling, full multi-iteration solver execution, moving VT / EE CCD and physical-GPU performance measurements remain incomplete. WARP validates correctness only; it is not physical-GPU performance evidence.
