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
**Status: in progress — Soft Body GPU FEM / Force / cross-Mesh targets and Runtime Playback / Blend → CPU Hierarchy → per-Segment GPU Skinning / Kelvin–Voigt Tracking are connected; immutable GPU Packet execution and target-GPU qualification remain open**

- Implemented per-model Soft Body Tet / CSR / inverse-mass resources, two persistent position / velocity State Banks, and Tick Scratch.
- Implemented gated Tet deformation-gradient / scaled-polar stress evaluation with a proper-rotation 3×3 Jacobi SVD fallback. Finite inverted Tet intermediates now follow the CPU Reference diagnostic path, publish finite per-Tet output for parity checks, and set terminal failure before any downstream State / Render Commit.
- Render Mesh Paint channels transfer to boundary Physics Vertices and use double-precision Dirichlet harmonic extension for interior vertices; ambiguous coincident surface samples fail Model registration.
- Tet Shape Recovery / Volume use four-vertex mean Paint values; Damping uses each Physics Vertex's Paint value. Density remaps inverse masses while preserving Rest Density calibration.
- World Preflight aggregates FEM CFL, Animation Tracking frequency, World and Contact limits. `mesh_physics_parameters` exposes finite non-negative Kelvin–Voigt `tracking_stiffness` (N/m) / `tracking_damping` (N*s/m), both defaulting to zero. For active Playback / Blend input, Preflight computes the maximum per-vertex frequency from density-remapped Rest Lumped Mass and Animation Tracking Paint / multiplier, then applies `dt_tracking_limit = 0.2 / omega_max`. Zero stiffness / Paint and Fixed vertices contribute no spring-frequency limit. Retry preserves the same Reservation and derived Substep / Segment plan.
- Immutable Input Snapshot-driven CPU Segment planning inserts every Force start / end and Impulse event time into the World Substep boundaries, preserves distinct event times, merges exactly equal boundaries, assigns active Force / start-boundary Impulse event indices to each positive-duration Segment, and retains the deterministic plan across same-Reservation Budget Retry.
- Soft Body GPU integration dispatch consumes the exact per-Segment delta time and creates one immutable Integration Parameters constant buffer per Model Mesh / Segment. The In-Flight Tick retains these constant buffers until its Event Query is CPU-observed, preventing later Segment / Tick parameter updates from overwriting queued GPU work. The existing multi-in-flight WARP Soft Body Runtime regression configures three World Substeps and exercises the per-Segment dispatch path before Render Candidate publication.
- Added a C++17 CPU double-precision reference and persistent golden for the normative Rest Lumped Mass Wendland C2 distribution. The production GPU path supports exact-Mesh, Model-wide and World-wide Soft Body Force / Impulse targets. Every selected Mesh computes Segment-start mass-weighted Wendland C2 values; its group partial sums are written into a shared per-Tick scratch buffer, one target-scope Finalize produces the common denominator, and every selected Mesh applies its share before Segment integration. Event-ID order, per-Mesh FEM Force composition, one-shot Impulse-before-damping integration and success-only Commit are preserved. Scratch buffers and views remain owned until the Tick Query is CPU-observed. Cloth targets remain deferred.
- Odd-substep Scratch publication, GPU failure validation, success-only Tick Commit, and Reset to rest pose are wired into the Runtime.
- Added WARP source coverage for two in-flight Ticks, three Substeps, alternating State Banks, Query observation, CFL budget recovery, Reset, and Render Candidate / Host Lease lifecycle. Render SRV regressions read Position / Normal / Tangent streams back, validate model-local output after translated / rotated / slightly non-unit Root Transform input, check frame normalization / orthogonality, and reject premature publication / mismatched Lease release.
- Added production Runtime FEM WARP parity coverage for positive polar deformation, near-singular positive SVD fallback and finite inverted deformation. Inverted elements publish finite diagnostic FEM output matching CPU Reference while terminal failure prevents Tick Commit. The parity test uses the current 80-byte Commit Control ABI and compares deformation, Proper Rotation, stress, inversion penalty forces and energy. GPU finite gating checks deformation, rotation, both stress tensors, every nodal force and both energy terms. This regression passed in hosted Windows WARP run #1415.
- `advance_report` distinguishes Host Render Lease backpressure from GPU Status Query Ring capacity admission and clears stale block reporting after successful submission.
- The CPU-observed terminal path restores every non-COMMITTED Reservation in ascending Sequence order. Later predecessor-blocked Status cannot downgrade the earliest FAILED / REJECTED result. A WARP regression creates a degenerate Render-only face to fail Candidate validation while a dependent Tick is already in flight, then checks both Reservations remain ordered until Reset.
- Hosted Windows CI run #1415 executed the Release x64 solution build and complete CPU / WARP test suite successfully. A separate run #1414 failed in the DXGI Timestamp Query collection regression; run #1415 passed the full suite on the immediately following commit.
- Added strict Tet rest-matrix / shape-gradient, unique-vertex and exact Vertex-to-Tet CSR validation before GPU upload; malformed asset / CSR input is rejected.
- Added `R1_WARP_SOFT_BODY_ANIMATION_TRACKING_STABILITY_PREFLIGHT`: high Tracking stiffness blocks CPU Preflight before GPU submission, invalid stiffness / damping coefficients are rejected, zero Animation Tracking Paint removes the Tracking frequency term, and high Tracking coefficients without a trajectory do not alter the FEM-only Substep count. Unsupported trajectory input remains explicitly `BLOCKED_BY_INPUT`. This regression passed Windows WARP runs #1441 / #1442 and the full current-code suite in #1457.
- Added a CPU Animation Channel Span evaluator and Clip Span Scheduler to `physics/animation_track.hpp/.cpp`. STEP, LINEAR and CUBICSPLINE channels become immutable Span-local coefficients; LINEAR Quaternion uses shortest-arc Slerp; CUBICSPLINE Quaternion normalization projects derivatives into the tangent space. Translation / Scale / Rotation derivatives are returned per World Simulation second. Paused Tracks produce zero velocity, reverse Playback preserves signed derivatives, and malformed / cross-Key Spans fail closed. CPU multi-Track / Loop / Blend / Hierarchy / Skin Matrix evaluation and immutable Packet construction are implemented and separately covered by CPU / WARP ABI tests.
- Added synchronized multi-Track Clip Span construction and World-Time scheduling. Every Node / Path Track contributes Key boundaries to one shared Clip partition; channels preserve Asset Track order and share exact Clip-Time start / delta. Reverse schedules emit boundaries in descending Clip-Time order while retaining positive, increasing World-Time intervals. Invalid Node references, duplicate Node / Path channels, malformed Tracks, unrepresentable World-Time increments and partial Clip publication fail closed.
- Added loop-aware CPU Playback Span scheduling. Positive Playback splits at Clip duration and resumes at zero; negative Playback splits at zero and resumes at Clip duration. Non-looping Playback holds its terminal Pose for the remaining Logical Tick, Speed=0 emits one zero-Clip-Time Span, an exact Tick-end wrap emits no empty Span, and a configurable Span-count limit rejects pathological wrap rates atomically. The scheduler also supports already-unwrapped per-Clip intervals through the underlying Clip Span API; Ping-Pong and automatic clip-time unwrapping remain pending.
- The Asset Animation Clip Scheduler merges Key times across all Tracks and creates a synchronized Clip Span array. Each Clip Span contains one coefficient record per original Asset Track, in stable order, with a shared Clip-Time start / delta. Forward and reverse schedules use deterministic boundary ordering; duplicate Node / Path channels, invalid Node indices, Clip-wrap-crossing intervals and malformed Track data fail atomically. One-key constant Tracks remain in every Clip Span but do not add interpolation boundaries. Loop-aware Playback and synchronized Source / Target / Hermite Weight scheduling are also connected. The CPU Node Hierarchy evaluator applies Rest TRS / static Matrix plus synchronized channels, builds parent-before-child Global Matrices and analytic derivatives, and emits inverse-Mesh-relative Skin Matrices / derivatives. The CPU Blend Pose evaluator samples Source / Target on one shared World-Time span, blends Translation / Scale with the `w_dot * (target - source)` derivative term, hemisphere-aligns and normalizes Quaternion Blend with analytic tangent-space derivative, then rebuilds the hierarchy and Skin Matrices from the blended local Pose.
- Added an immutable CPU Animation Trajectory Packet Builder. It emits fixed-layout Span / Track / Channel / Blend records, a dense Asset Node static-matrix buffer, per-Skin / Joint Inverse Bind matrices, and immutable Skin → ordered Joint mapping records. Every Skinned Mesh Node gets one Skin Record; Mesh Nodes sharing the same Asset Skin reuse the same ordered Skin Joint range. Each Skin Joint entry points to its Track-local topological Channel offset, the Asset Joint Node Index, and its Skin-specific absolute IBM Buffer index. A shared Joint Node may have different IBM matrices in different Skins. Duplicate Joint Nodes within one Skin, invalid Joint / Skin / IBM data, malformed Track ranges, non-contiguous Spans, and invalid certified Bounds fail atomically. The mutable Segment Cursor remains separate from the immutable Packet and retains the Packet Input Version.
- The immutable Animation Trajectory Packet is now the Runtime GPU data source. Each active fully skinned Soft Body Mesh / Integration Segment builds and uploads its immutable Span / Track / Channel / Blend / Skin / Skin Joint / IBM / Static Matrix tables plus an independently versioned Segment Cursor; all Packet resources are retained by the In-flight Tick until its Event Query is CPU-observed. GPU Channel Evaluation executes each Track, GPU Blend evaluates the synchronized Hermite Weight and analytic derivative, depth-ordered GPU Hierarchy Evaluation builds Global Matrices / derivatives, GPU Skin Matrix Evaluation applies the sampled Model Root Transform / analytic Root Matrix derivative and Skin-specific IBM, and production GPU LBS generates Physics Vertex Target Position / Velocity. Ordered Root Transform Samples in the immutable Tick Snapshot add exact Integration Segment boundaries; translation and shortest-arc Quaternion interpolation produce analytic World-Time Root Matrix derivatives, while a teleport sample resets the derivative at its boundary. The Segment-owned status word carries stage-specific diagnostic bits (Channel=1, Blend=2, Hierarchy=4, Skin Matrix=8, LBS=16) into the Integrate failure code. The production Integrate pass consumes the GPU targets and applies Paint-weighted Kelvin–Voigt stiffness / damping. The mutable Cursor is uploaded and retained, while shader-side Cursor advancement / multi-Span packet reuse is still pending.
- Added `shaders/runtime/animation_channel_evaluate.cs.hlsl`: one GPU Thread evaluates one immutable 224-byte Channel Record into a 112-byte Segment-local TRS / analytic-derivative record. Translation / Scale polynomial derivatives are converted to World-Time units; Rotation supports STEP, shortest-arc LINEAR Slerp and normalized CUBICSPLINE with tangent-space derivative projection. Rest TRS is constant with zero Velocity. Node / Parent indices, masks, interpolation flags and quaternion normalization fail closed through a Segment Status bit. The WARP Golden Regression `P09_WARP_ANIMATION_CHANNEL_EVALUATION` passed #1784. This is the GPU Local Channel stage only; Blend, Parent Global Matrix / derivative, inverse Mesh Node, Skin-specific IBM composition, Runtime Packet / Cursor execution and Commit Gate wiring remain pending.
- Added `shaders/runtime/animation_blend_local_transform.cs.hlsl`: the GPU Blend stage consumes Source / Target Local TRS / Derivative records and the immutable Hermite Blend Record. It evaluates the shared World-Time Span Weight, adds `w_dot * (target - source)` to Translation / Scale Velocity, hemisphere-aligns Source / Target Quaternion values and derivatives, then normalizes the blended Quaternion and projects its derivative into the tangent space. Node / Parent / Role mismatches and invalid weights / quaternions fail closed. WARP Golden `P09_WARP_ANIMATION_BLEND_LOCAL_TRANSFORMS` passed #1802.
- Added `shaders/runtime/animation_node_hierarchy_evaluate.cs.hlsl`: Local TRS / Derivative records are converted to affine Local Matrices / analytic derivatives; static Matrix Nodes read the dense Asset Node Matrix Buffer. Parent-before-child Depth Dispatch composes Global Matrices and Product-Rule derivatives. Self-parent / non-topological Parent, cycles, non-finite matrices, non-positive Scale and zero Quaternion fail closed. WARP Golden `P09_WARP_ANIMATION_NODE_HIERARCHY` passed #1808. Model Root composition, Mesh Node inverse and Skin-specific IBM composition remain the next GPU stages.
- Added `shaders/runtime/animation_skin_matrix_evaluate.cs.hlsl`: the GPU resolves Track-relative Mesh / Joint Channel offsets through immutable Skin / Skin Joint mappings, fetches each Skin-specific absolute IBM record, and computes `inverse(mesh_global) * joint_global * IBM`. Its analytic derivative includes `-inverse(mesh) * mesh_dot * inverse(mesh)` and the Joint Global derivative product term. Output is packed in Skin-local Joint order. Singular / non-affine matrices, invalid ranges and mismatched Asset Node identities fail closed. WARP Golden `P09_WARP_ANIMATION_SKIN_MATRICES` passed in full-suite run #1827. Model Root Transform and Runtime Packet / Cursor dispatch remain pending.
- Runtime continues to report `BLOCKED_BY_INPUT` for Root Transform Samples without an active Animation Track, kinematic Collider trajectory work, Cloth and unsupported physical paths. Fully skinned Soft Body Mesh Playback / Blend now executes the immutable Packet on GPU through Channel / Blend / Hierarchy / Skin Matrix / LBS stages before Kelvin–Voigt Tracking integration. When Playback / Blend is active, ordered Model Root Transform Samples split exact Integration Segment boundaries and provide translation / Quaternion interpolation plus analytic Root Matrix velocity; teleport samples reset the Root derivative at the boundary. Non-skinned animated Soft Body Meshes fail closed. Runtime WARP coverage verifies one-joint Translation Playback, two-Clip Hermite Blend, sampled Root Motion, committed Tick sequences, Physics Vertex movement, and Render Candidate readback / lease release. Packet Track Parent Channel indices are explicitly Track-local; the two-Track builder regression verifies that the second Track does not leak absolute Packet offsets into its local hierarchy. Model / World cross-Mesh Force denominators remain covered by the 1/2 and 1/4 response-ratio regression.
- Dedicated WARP regressions validate Force distribution Weight / Reduction / Finalize / Apply against the CPU double-precision oracle; Runtime Exact Mesh / Model / World scope response ratios 1 / 1/2 / 1/4 passed #1418 / #1419. Animation Tracking stability / Paint-mask Preflight passed #1441 / #1442 and full-suite #1457. CPU Channel Span, multi-Track / Loop / Reverse / Blend / Hierarchy / Skin Matrix derivative and immutable Packet / Skin Joint ABI regressions passed #1468–#1666. CPU Packet Builder Track-local Parent Channel offsets and GPU Packet upload fixture validation passed #1895 / #1896. The Runtime GPU Packet pipeline, including two-Track Playback / Blend, GPU Channel / Blend / Hierarchy / Skin Matrix / LBS, stage-specific failure diagnostics, Kelvin–Voigt Tracking, Tick Commit and Render Candidate readback / release, passed #1895. Sampled Root Transform Segment splitting and GPU Root Matrix derivative are under current Windows Release x64 / WARP validation. Physical target-GPU qualification, shader-side Cursor advancement / multi-Span reuse, Root-only motion, Collider trajectory integration, and remaining Cloth / Collision / Contact coupling remain pending.

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

The Runtime Playback / GPU Skinning / Tracking implementation and end-to-end WARP regression passed Release x64 build and the complete CPU / WARP test suite in #1761 / #1762 on `e08d0c6`. The latest source head `2745d24` adds explicit World Reset to the animated Skin test cleanup and is under repeat verification in #1763 / #1764. Physical-GPU driver qualification, performance profiling and cross-vendor behavior remain unverified and are still required by Phase 7 / Phase 8.

## Scope control

- Do not simplify solver quality, remove candidates, reduce requested time or turn unresolved geometry into No-Hit to make a phase appear complete.
- Do not silently change the normative design to fit an implementation shortcut.
- Keep the implementation branch isolated; do not merge PR #3 early.
- Once the phase implementations are present, defer non-critical refactoring and cosmetic review until the consolidated bug-fix phase.
