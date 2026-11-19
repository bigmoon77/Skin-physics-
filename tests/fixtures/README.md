# Golden test data

This directory contains version-controlled input fixtures and expected reference results.

## Rules

1. Input fixtures are immutable test inputs. A test must never regenerate or edit them during execution.
2. Expected files are reviewed reference data. Never update them automatically because a test failed.
3. Floating-point comparisons use the tolerance declared by the expected file or test case. Topology, IDs, counts, state transitions and JSON schema fields use exact comparisons.
4. Every implementation phase must add at least one positive case, one boundary/degenerate case and one negative/error case where that class of failure is meaningful.
5. Each implemented phase must expose a named test group in `SkinPhysicsTests`. A fixture existing on disk does not count as a passing test.
6. Golden changes require a reason, a numerical derivation or independent reference, and a test review in the same commit.
7. Performance fixtures define workload size and correctness invariants; they do not define machine-independent FPS thresholds.

## Verified test coverage

- **Phase 01 Importer:** persistent OBJ and glTF fixtures, unit conversion, skinning, material and animation import.
- **Phase 02 Physics Mesh / Skin Weight / Cache:** persistent MSH fixtures, topology, mass, harmonic weights, cache and resolution settings. Actual fTetWild execution against a pinned binary remains environment-dependent.
- **Phase 03 Render Binding:** CPU interior/exterior embedding and attribute preservation plus D3D11 WARP Render Feedback readback.
- **Phase 04 Test Environment:** fixed-step overload behavior, Scenario JSON round-trip, unknown extension preservation, Run Result validation/comparison and transactional stage rollback.
- **Phase 05 Cloth:** CPU XPBD tests and WARP Stretch, Shear and Bend kernel readback against the CPU reference.
- **Phase 06 Soft Body FEM:** CPU rigid-rotation/compression/inversion tests and WARP per-tetrahedron FEM evaluation for positive-J cases. The inverted-element GPU SVD fallback remains pending.
- **Phase 07 Collision:** CPU references plus WARP VT/EE narrowphase, host-built BVH traversal, pair filtering and bounded-output overflow checks.
- **Phase 08 CCD:** CPU initial-penetration repair plus WARP point-versus-static-triangle Conservative Advancement TOI/barycentric readback.
- **Phase 09 Animation:** CPU target sampling plus WARP translation/velocity, quaternion Slerp and teleport policy.
- **Phase 10 Attribute Paint:** CPU operations and WARP Paint/Smooth/Average/Erase comparisons.
- **Phase 11 Performance / GPU orchestration:** WARP Timestamp Query collection, DXGI local-memory query capability handling and two-pass Compute Pass Graph execution with SRV/UAV hazard validation.
- **Phase 12 Correction Motion CCD (CPU reference + initial GPU pipeline):** fixture-driven VT / EE correction path hit and crossing, positive / negative latched side, same-side transient proximity, endpoint contact discovery, certified miss, ambiguous start-side rejection, non-finite input rejection and interval-budget exhaustion. CPU checks cover per-primitive outward-rounded Correction Swept AABBs, the existing Spatial Hash Broadphase adapter, shared-vertex and explicit one-ring topology filtering, stable-ID canonical Candidate Union, atomic Position-Writer Epoch processing, independent Candidate / Epoch generations, Final Discrete VT / EE Active Contact Set construction, Vertex→Contact CSR generation, stale-refresh rejection and capacity rollback. WARP shader/test coverage has been added for the Correction Swept AABB pass and the Count → Exclusive Scan / Capacity Check → Fill → Bitonic Sort → Dedup Candidate pipeline, compared against CPU Canonical Pair output including topology filters and no-partial-capacity behavior; both GPU paths remain unverified until a Windows Runner executes. Production GPU interval CCD, GPU Contact Patch / CSR integration, actual solver Epoch scheduling, endpoint-clamped EE persistent-normal support, and Runtime Commit Gate / pseudo-time integration remain pending.

## GPU test interpretation

Windows CI uses **D3D11 WARP**, a software device. It verifies shader compilation, dispatch, buffer layout and readback against reference data. It does not establish physical-GPU throughput, driver portability or performance at the 2k/8k/32k workload profiles.

The complete runtime solver connection remains partial. GPU BVH construction, FEM force accumulation/integration, cloth multi-iteration scheduling, moving-primitive CCD, editor integration and physical-GPU profiling remain open. The per-phase mapping and exact limitations are maintained in \`manifest.json\`.

The mapping from phase/test ID to input and expected result is in `manifest.json`.