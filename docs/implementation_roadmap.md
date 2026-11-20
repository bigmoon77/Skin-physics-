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
**Status: in progress — asynchronous Tick submission / Query observation, Soft Body Render Candidate / Host Lease, and immutable per-Tick CPU Input Snapshot ownership are implemented; Segment Snapshots remain pending**

- Public C++17 API, descriptors, status codes and generation-checked World / Model / Collider handles.
- Host-owned D3D11 Device / Immediate Context validation and COM lifetime.
- Single registered Submission Thread enforcement.
- World lifecycle, fixed-step admission, accumulator / Step Request / Tick Reservation accounting.
- Runtime / World execution budgets and revisioned recovery. Budget Retry requires the oldest uncommitted, CPU-observed Work-Budget Block, zero in-flight Submissions, monotonic Admission Limits and exact Substep / Segment lower bounds. Idle Budget changes are rejected while Whole Tick or Step demand remains.
- GPU status buffer, Event Query ring, non-blocking poll and contiguous CPU-observed COMMITTED prefix.
- Per-World immutable Dispatch Plan, predecessor gate, terminal status and success-only Commit Control.
- Multiple in-flight non-Commit outcomes restore Reservations in ascending Sequence order. The earliest FAILED / REJECTED result is preserved; a predecessor-blocked Reservation is re-preflighted only after its predecessor Commit is CPU-observed.
- CPU Preflight and GPU Dynamic TOI Budget Status must be returned by poll(world) for that World before a Budget Retry. Polling another World cannot authorize the update. A completed older Commit advances committed time while retaining a newer Head's own Budget Block.
- GPU terminal BLOCKED is classified by Status Origin: Dynamic TOI maps to WORK_BUDGET_EXCEEDED / retryable (or WORK_BUDGET_UNSATISFIABLE above the immutable Hard Limit), while an origin-free terminal BLOCKED maps to BLOCKED_BY_PREDECESSOR. The production Dynamic TOI Segment producer and Boundary Table scheduler remain open.
- advance_report distinguishes Host Render Lease Backpressure from Query / Capacity Admission stalls.
- Per-Soft-Body-Mesh persistent position / velocity State Banks, per-Tick Scratch, gated GPU submission and reset-to-rest-state lifecycle.
- Every CPU Tick Reservation owns a monotonic-versioned immutable Input Snapshot covering World settings, Force / Impulse inputs, Model Root / Animation / Material state, transferred Physics Attribute weights, and Collider descriptors / pose samples. The same shared snapshot survives CPU Preflight Block / Budget Retry, GPU Submission, in-flight Query ownership and non-COMMITTED Reservation restoration.
- Soft Body Render Candidate Ring uses maximum in-flight + 1 Slots, GPU-side Binding / Normal / Tangent feedback, full-width Slot ID / Generation Status publication, CPU-observed-only Host Lease, and submission blocking while leased. Cloth / Collision Render Candidates remain with their solver phases.

**Exit criteria**
- Public headers match the Runtime Design field widths and semantics.
- All APIs validate handle generation, state and submission-thread rules.
- Pause / Stop / Step / Reset and time accounting preserve requested / reserved / committed / observed time independently.
- Query ring never blocks the frame path, overwrites an in-flight slot or advances the observed prefix across a non-COMMITTED tick.
- A failed / blocked / stale-predecessor Tick cannot change the active State Bank or committed generation.
- Lifecycle and retry tests cover all documented BUSY / BLOCKED / REJECTED / FAILED boundaries.

### Phase 2 — R1 Soft Body GPU integration
**Status: in progress — gated Soft Body GPU path, paint transfer and CFL preflight are connected; numerical execution gates remain open**

- Implemented per-model Soft Body Tet / CSR / inverse-mass resources, two persistent position / velocity State Banks, and Tick Scratch.
- Implemented gated Tet deformation-gradient / scaled-polar stress evaluation with a proper-rotation 3×3 Jacobi SVD fallback. Finite inverted Tet intermediates now follow the CPU Reference diagnostic path, publish finite per-Tet output for parity checks, and set terminal failure before any downstream State / Render Commit.
- Render Mesh Paint channels transfer to boundary Physics Vertices and use double-precision Dirichlet harmonic extension for interior vertices; ambiguous coincident surface samples fail Model registration.
- Tet Shape Recovery / Volume use four-vertex mean Paint values; Damping uses each Physics Vertex's Paint value. Density remaps inverse masses while preserving Rest Density calibration.
- World Preflight aggregates the FEM CFL limit with World / Contact limits. Budget overflow blocks the same Reservation; increasing the admission budget retries the same derived Substep plan.
- Immutable Input Snapshot-driven CPU Segment planning inserts every Force start / end and Impulse event time into the World Substep boundaries, preserves distinct event times, merges exactly equal boundaries, assigns active Force / start-boundary Impulse event indices to each positive-duration Segment, and retains the deterministic plan across same-Reservation Budget Retry.
- Soft Body GPU integration dispatch consumes the exact per-Segment delta time and creates one immutable Integration Parameters constant buffer per Model Mesh / Segment. The In-Flight Tick retains these constant buffers until its Event Query is CPU-observed, preventing later Segment / Tick parameter updates from overwriting queued GPU work. The existing multi-in-flight WARP Soft Body Runtime regression configures three World Substeps and exercises the per-Segment dispatch path before Render Candidate publication.
- GPU Force / Impulse weight distribution and application remain pending.
- Odd-substep Scratch publication, GPU failure validation, success-only Tick Commit, and Reset to rest pose are wired into the Runtime.
- Added WARP source coverage for two in-flight Ticks, three Substeps, alternating State Banks, Query observation, CFL budget recovery, Reset, and Render Candidate / Host Lease lifecycle. Render SRV regressions read Position / Normal / Tangent streams back, validate model-local output after translated / rotated / slightly non-unit Root Transform input, check frame normalization / orthogonality, and reject premature publication / mismatched Lease release.
- Added production Runtime FEM WARP parity coverage for positive polar deformation, near-singular positive SVD fallback and finite inverted deformation. Inverted elements publish finite diagnostic FEM output matching CPU Reference while terminal failure prevents Tick Commit. The parity test uses the current 80-byte Commit Control ABI and compares deformation, Proper Rotation, stress, inversion penalty forces and energy. GPU finite gating checks deformation, rotation, both stress tensors, every nodal force and both energy terms. The WARP source is registered; execution remains unverified pending a runner that reports build steps.
- `advance_report` distinguishes Host Render Lease backpressure from GPU Status Query Ring capacity admission and clears stale block reporting after successful submission.
- The CPU-observed terminal path restores every non-COMMITTED Reservation in ascending Sequence order. Later predecessor-blocked Status cannot downgrade the earliest FAILED / REJECTED result. A WARP regression creates a degenerate Render-only face to fail Candidate validation while a dependent Tick is already in flight, then checks both Reservations remain ordered until Reset.
- These sources remain unexecuted because the latest Windows CI run failed before reporting any build steps.
- Added strict Tet rest-matrix / shape-gradient, unique-vertex and exact Vertex-to-Tet CSR validation before GPU upload; malformed asset / CSR input is rejected.
- Runtime reports BLOCKED_BY_INPUT and defers Submission when Force / Impulse Events, Animation Playback / Blend, Root Transform Samples, Cloth or Collider work is registered but the corresponding Segment / Trajectory / Solver path is not connected. WARP coverage verifies Force / Impulse and Root Sample inputs preserve the CPU Reservation without GPU Submission and Reset explicitly clears them. This prevents silently committing a Tick while ignoring accepted simulation input.
- Remaining: CPU-oracle parity and physical/WARP numerical qualification; animation tracking stability limit and immutable trajectories; GPU Force / Impulse weight distribution / application; Cloth / Collision / Contact coupling; and Physics Attribute Paint cache persistence / remapping on topology rebuild.

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
**Status: pending — foundational Soft Body Render Candidate / Host Lease path was brought forward into R0; animation-dependent rendering remains pending**

- Immutable Animation / Kinematic Trajectory Packets and GPU-local time cursor.
- Clip / blend / derivative evaluation, teleport policy and versioned input snapshots.
- GPU Skinning and animation target / velocity generation.
- Extend the existing gated Soft Body Render Feedback Candidate to animation-skinned targets, full Render / Contact validation, and all supported solver modes.
- Expand Render Candidate publication / Host Lease integration to Cloth / Collision resources and validate renderer hazards on the target GPU.

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
