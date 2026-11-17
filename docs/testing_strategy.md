# Skin Physics Test Strategy

## Test architecture

Every implementation phase must contain the same test layers:

1. **Persistent input fixture**: version-controlled mesh, animation, material, collision or scenario data.
2. **Reviewed expected result**: exact topology/state and numerical reference values, including explicit tolerances.
3. **Executable assertions**: the test loads the input fixture and compares the implementation output against the expected result.
4. **Failure cases**: invalid, degenerate, capacity-limited or discontinuous input must produce the specified diagnostic and must not partially commit state.

The authoritative index is `tests/fixtures/manifest.json`. Input files are immutable during tests. Expected files are manually reviewed and are never regenerated automatically when a test fails.

### Comparison rules

- Vertex IDs, triangle/tetrahedron connectivity, pair IDs, counts, schema versions and state transitions use exact comparison.
- Floating-point geometry and solver values use absolute or relative tolerances declared by the corresponding expected-result file.
- Normalized weights must satisfy non-negativity and a weight sum of 1 within the declared tolerance.
- Performance profiles compare workloads, counts, memory and per-pass times. Absolute FPS is deliberately not a golden value because it depends on hardware, drivers and runtime configuration.
- Failure tests verify both the diagnostic and the absence of partial state commits.

## Phase matrix

| Phase | Fixture families | Executable verification | Current status / limitations |
|---|---|---|---|
| 01 Importer | OBJ quad + MTL, glTF skin/animation, missing unit | Persistent import fixtures, units, skinning and topology | Automated |
| 02 Physics Mesh | Single/two/center-star tets, resolution settings | Rest volume, quality, boundary, mass, CSR, harmonic weights, cache and resolution assertions | Partial: external fTetWild execution and production calibration remain environment-dependent |
| 03 Render Binding | Interior tet, exterior boundary offset, invalid point | CPU binding/feedback and D3D11 WARP render-feedback readback | Partial: cloth surface binding and full resource lifecycle remain |
| 04 Test Environment | Fixed-step, Scenario JSON, Run Result | Overload/backlog, semantic JSON round-trip, unknown fields, Run Result validation, stage order and atomic rollback | Partial: scenario editor and automatic application of scenario data to concrete solvers remain |
| 05 Cloth | Two-triangle square patch | CPU XPBD plus WARP Stretch/Shear/Bend kernels compared to CPU reference | Partial: full GPU multi-iteration scheduling, graph coloring/Gather and coupled animation/contact solve remain |
| 06 Soft Body FEM | Unit tet rotation, compression and inversion | CPU FEM plus WARP positive-J per-element deformation, polar rotation, stress, forces and energy | Partial: GPU nodal force accumulation/integration and inverted-element GPU SVD fallback remain |
| 07 Collision | VT, EE, pair set, bounded output | CPU reference plus WARP VT/EE narrowphase and host-built BVH traversal/overflow | Partial: GPU BVH build/refit, broadphase-to-contact pipeline, friction/restitution and persistent contacts remain |
| 08 CCD / Initial Repair | Swept point-triangle, over-budget repair | CPU repair plus WARP point-vs-static-triangle Conservative Advancement TOI/barycentrics | Partial: moving triangles, EE/TT CCD and global TOI scheduler remain |
| 09 Animation | Linear translation, quaternion and teleport | CPU sampling plus WARP target/velocity, Slerp and teleport policy | Partial: general clip sampling, hierarchy binding and substepped solver coupling remain |
| 10 Attribute Paint | Three-vertex line and four operations | CPU plus WARP Paint/Smooth/Average/Erase readback | Partial: editor brush UI, neighbor Laplacian and persistence integration remain |
| 11 Quality / Performance | 2k/8k/32k workload profiles | CPU invariants, WARP timestamp query, DXGI memory-query capability and Compute Pass Graph hazards | Partial: physical-GPU performance runs and regression baselines remain |

## GPU validation environment

Windows CI creates a D3D11 WARP software device, compiles Shader Model 5 kernels, dispatches them and reads structured buffers back. WARP verifies correctness; **WARP timing is not physical-GPU performance data**.

The Timestamp Query profiler uses a ring of frame slots and \`D3D11_ASYNC_GETDATA_DONOTFLUSH\`. Pending slots are not overwritten. Local memory usage/budget comes from \`IDXGIAdapter3::QueryVideoMemoryInfo\`; unsupported adapters return an explicit unavailable reason.

The Compute Pass Graph preserves insertion order, validates slots and SRV/UAV aliasing, and unbinds CS SRV/UAV/constant-buffer state at pass boundaries. It does not infer dependencies or wire solver stages automatically. Callers define pass order and own intermediate resources.

## Phase exit gate

A phase is complete only when its manifest cases have executable tests and all required cases pass in Windows CI. Merely adding fixture files does not complete a phase. Before moving to the next phase, record:

- fixture IDs and test IDs executed;
- expected versus measured values and tolerances;
- negative/error cases and their diagnostics;
- CI commit and run result;
- known limitations or untested branches.

When solver code is introduced, add at least one analytical or independently computed reference case per numerical algorithm. For example, the rest tetrahedron has (V=1/6), density 6 gives total mass 1, and the center-star harmonic fixture has analytically checked interior weights 0.55 / 0.15 / 0.15 / 0.15. These references prevent a solver from passing solely because it produces finite output.

## Running tests

Build `SkinPhysics.sln` in Visual Studio 2022, x64. Run `bin/x64/Debug/SkinPhysicsTests.exe` or `bin/x64/Release/SkinPhysicsTests.exe`. The test helper resolves fixture paths relative to `tests/test_support.hpp`, so the current working directory is irrelevant. Implemented cases print one `[PASS]` line per case.
