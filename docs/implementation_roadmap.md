# GPU Runtime Implementation Roadmap

## Working method

Implementation follows the frozen requirements / basic design / detailed design / Runtime Integration Design. The Runtime Integration Design is the implementation contract for ownership, ABI, dispatch ordering, capacity, time and commit semantics.

Work proceeds in waterfall order:
1. Freeze the phase boundary, inputs, ABI and exit criteria before implementation.
2. Implement each phase as a complete vertical slice, including its public boundary and data ownership.
3. Add test sources and deterministic fixtures as part of the implementation, but avoid repeated review-driven micro-refactoring.
4. Integrate all phases in dependency order.
5. After the full feature set is present, run the complete build / CPU / WARP / physical-GPU validation matrix and perform a consolidated bug-fix pass.
6. Do not merge the implementation PR until the final validation gates and documented limitations are resolved.

A phase is not called complete merely because headers, shaders or test sources exist. Completion requires the implementation, lifecycle integration, executable tests and the phase exit criteria.

## Phase order

### Phase 0 — Design baseline and ABI freeze
**Status: complete**

- Preserve the merged design baseline on `main`.
- Use `docs/runtime_integration_design.md` as the Runtime-specific contract and `docs/detailed_design.md` as the solver / numerical contract.
- Keep the implementation on `feature/gpu-runtime-implementation`.
- Record open implementation dependencies in this roadmap; change normative design only when a real contradiction is found.

### Phase 1 — R0 Runtime boundary and asynchronous scheduler
**Status: in progress — asynchronous Tick submission / Query observation and a first gated Soft Body path are implemented; immutable Segment Snapshots and Render leases remain pending**

- Public C++17 API, descriptors, status codes and generation-checked World / Model / Collider handles.
- Host-owned D3D11 Device / Immediate Context validation and COM lifetime.
- Single registered Submission Thread enforcement.
- World lifecycle, fixed-step admission, accumulator / Step Request / Tick Reservation accounting.
- Runtime / World execution budgets and revisioned recovery.
- GPU status buffer, Event Query ring, non-blocking poll and contiguous CPU-observed COMMITTED prefix.
- Per-World immutable Dispatch Plan, predecessor gate, terminal status and success-only Commit Control.
- Per-Soft-Body-Mesh persistent position / velocity State Banks, per-Tick Scratch, gated GPU submission and reset-to-rest-state lifecycle.
- Rollback / retry keeps the exact same CPU Reservation; immutable per-Tick Input Snapshot / Packet Version is still pending.
- Render Candidate / Host Render Lease publication remains pending; Query / in-flight lease admission and safe destroy / drain / shutdown guards are present.

**Exit criteria**
- Public headers match the Runtime Design field widths and semantics.
- All APIs validate handle generation, state and submission-thread rules.
- Pause / Stop / Step / Reset and time accounting preserve requested / reserved / committed / observed time independently.
- Query ring never blocks the frame path, overwrites an in-flight slot or advances the observed prefix across a non-COMMITTED tick.
- A failed / blocked / stale-predecessor Tick cannot change the active State Bank or committed generation.
- Lifecycle and retry tests cover all documented BUSY / BLOCKED / REJECTED / FAILED boundaries.

### Phase 2 — R1 Soft Body GPU integration
**Status: in progress — first Runtime-connected Soft Body compute path added; numerical and feature-completeness gates remain open**

- Implemented initial runtime upload of per-model Soft Body Tet / CSR / inverse-mass resources and two persistent position / velocity State Banks plus Tick Scratch.
- Implemented gated Tet deformation-gradient / scaled-polar stress evaluation, deterministic Vertex-to-Tet CSR force gather, and damped force integration.
- Implemented World-preflight substep dispatch, odd-substep Scratch publication into the inactive bank, GPU failure validation, and success-only Tick Commit.
- Added WARP Runtime coverage for two in-flight Ticks, three Substeps per Tick, alternating State Banks, Query observation, and Reset to the rest pose; source is registered but not yet executed by Windows CI.
- Remaining: proper-rotation SVD fallback, inverted / near-singular parity, per-vertex attribute and density remapping, CFL-derived limits, Force / Impulse segmentation, animation / kinematic coupling, Collision / Contact coupling, and CPU oracle parity for the production Runtime path.

### Phase 3 — R2 Cloth GPU iteration
**Status: pending**

- Stretch coloring and Shear / Bend / Animation Tracking Gather-Jacobi paths.
- Same-snapshot reads and projected Lambda Delta semantics.
- Vertex-to-Constraint CSR, per-iteration Position-Writer Epoch creation and correction-candidate refresh.
- Lambda / Substep / Segment lifetime and actual-dt Compliance behavior.
- CPU reference parity and topology / coloring validation.

### Phase 4 — R3 Mixed Collision Runtime
**Status: pending**

- GPU Spatial Hash or designed hierarchy build/refit, cell records and complete Candidate generation.
- All enabled Mesh Pairs, self-collision topology filtering and canonical pair deduplication.
- VT / EE Narrowphase, deterministic Contact Patch reduction and bidirectional correction.
- Active Contact Set and Vertex-to-Contact CSR generation / generation binding.
- Non-responsive Contact guard and World-wide Coupled Projected Jacobi impulse solve.
- Capacity growth, atomic publication and exact Reservation rollback.

### Phase 5 — R4 CCD and initial repair
**Status: pending**

- Moving VT / EE conservative advancement and minimum-TOI reduction.
- Initial penetration repair and per-vertex cumulative repair budget.
- Unique TOI boundary grouping, zero-time / end-time handling and residual Segment scheduling.
- Correction Motion CCD on every Position-Writer Epoch and Contact Position iteration.
- Interval-certified fail-closed handling and strict separation of physical TOI time from correction pseudo-time.
- No Commit on unresolved geometry, crossing, incomplete candidate coverage or capacity failure.

### Phase 6 — R5 Animation and Render integration
**Status: pending**

- Immutable Animation / Kinematic Trajectory Packets and GPU-local time cursor.
- Clip / blend / derivative evaluation, teleport policy and versioned input snapshots.
- GPU Skinning and animation target / velocity generation.
- Render Feedback Candidate generation, validation and atomic publication with Physics State.
- Render Candidate Ring, CPU-observed committed prefix and Host Render Lease lifecycle.

### Phase 7 — R6 target-GPU qualification
**Status: pending**

- Physical GPU / driver validation, D3D11 hazard and device-loss behavior.
- 2k / 8k / 32k workload series and declared quality / timing / memory measurements.
- Resource budget growth / rollback, multi-World scheduling and 1 / 2 / 4 in-flight window comparisons.
- Cross-vendor qualification where hardware is available.

### Phase 8 — Consolidated bug-fix and release gate
**Status: pending**

- Run all existing and new CPU tests, WARP tests, Windows x64 Debug / Release builds and physical-GPU scenarios.
- Classify every failure as source defect, test defect, environment / runner defect or unsupported hardware path.
- Fix defects in dependency order; rerun the complete affected regression set and then the full suite.
- Reconcile the test manifest, testing strategy, README and PR description with observed results.
- Keep PR #3 Draft until all required gates are green or an explicitly documented environmental limitation is accepted.

## Current known validation constraint

Windows CI has repeatedly reported a failed job with no reported job steps and no retrievable log artifact. Such attempts do not establish a compiler or test failure. Until a Windows runner executes the workflow, C++ compilation, HLSL compilation, WARP execution and physical-GPU behavior remain unverified.

## Scope control

- Do not simplify solver quality, remove candidates, reduce requested time or turn unresolved geometry into No-Hit to make a phase appear complete.
- Do not silently change the normative design to fit an implementation shortcut.
- Keep the implementation branch isolated; do not merge PR #3 early.
- Once the phase implementations are present, defer non-critical refactoring and cosmetic review until the consolidated bug-fix phase.
