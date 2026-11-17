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

| Phase | Fixture families | Reference results | Acceptance mapping | Current test status |
|---|---|---|---|---|
| 01 Importer | OBJ quad + MTL, glTF skin/animation, OBJ missing unit | Converted coordinates, triangle count, material factors, joint weights, animation keys | Importer, units, skinning and topology checks | Automated |
| 02 Physics Mesh | Single tet, two tets sharing a face, center-star tet mesh | Rest volume, quality, dihedral, boundary faces, lumped mass, CSR, harmonic weights, cache behavior | T13, T26; T25 resolution fixture prepared | Partial: mesh/skin/cache automated; resolution scenario awaits builder API |
| 03 Render Binding | Interior tet point, exterior boundary-offset point, invalid distant point | Barycentric coordinates, Rest offset, reconstructed Rest position, attribute preservation | T12, T27 | Partial: CPU binding/feedback golden tests; D3D11 GPU path pending |
| 04 Test Environment | Fixed-step timeline, scenario JSON, Run Result | Step count, accumulator, simulation time, semantic round trip, explicit unavailable metrics | T24, T28 | Partial: fixed-step/overload tests; JSON and GPU telemetry pending |
| 05 Cloth | Two-triangle square patch | Rest lengths, areas, dihedral, finite-difference gradient reference, compliance monotonicity | T05, T06, T18 | Partial: CPU XPBD Stretch/Shear/Bend and finite-difference verification; GPU dispatch pending |
| 06 Soft Body FEM | Unit right tetrahedron under rigid rotation, compression and inversion | Deformation gradient, determinant, volume, strain/energy and inversion diagnostic | T01–T04, T19 | Fixtures prepared |
| 07 Collision | VT, EE, broadphase pair set, capacity overflow | Closest points, barycentrics, penetration, exhaustive candidate superset, retry/no-drop behavior | T07, T08, T10, T11, T17, T22 | Fixtures prepared |
| 08 CCD / Initial Repair | Swept point-triangle and over-budget initial penetration | Analytical TOI, one impact impulse, remaining time, blocked state | T09, T16, T23 | Fixtures prepared |
| 09 Animation Tracking | Linear translation, quaternion rotation and teleport | Target position/velocity and discontinuity policy | T20, T21 | Fixtures prepared |
| 10 Attribute Paint | Three-vertex weight line and Paint/Smooth/Average/Erase operations | Per-operation scalar arrays, range and save/load | Paint/remap requirements | Fixtures prepared |
| 11 Quality / Performance | 2k/8k/32k tet and cloth workload profiles | Correctness invariants, candidates, contacts, timings, memory | T14, T15 | Fixtures prepared |

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
