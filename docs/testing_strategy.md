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


## Numerical acceptance profile

The proposed normalized GPU float32 / CPU double comparison and physical-invariant thresholds are recorded in `docs/numerical_acceptance_profile_v1.json`.

- Compare trajectories only when CPU and GPU use the same solver equations, algorithm, operation order, iteration order, substep boundaries and input snapshots.
- Compare different solver algorithms using analytical fixtures, physical invariants and convergence/energy behavior. Do not demand frame-by-frame trajectory equality between different algorithms.
- The profile thresholds are initial engineering candidates. Validate them against existing Golden Fixtures, analytical cases and WARP before treating them as normative pass thresholds.
- A threshold change requires documented evidence, a profile version increment, independent review, and rerunning all relevant numerical / physical tests.
- WARP is a correctness environment. WARP timing is not physical-GPU performance evidence.

## Phase 12 — GPU Runtime transaction and integration

The Runtime design is specified in `docs/runtime_integration_design.md`; the design-closure decisions and review checklist are in `docs/design_closure.md`; its UML is `docs/uml/phase12_gpu_runtime.md`. These gates connect the existing CPU references and WARP kernels into a production asynchronous GPU Runtime. The gates are planned; they are not claimed complete by the presence of design documents.

| Gate | Fixture / test family | Required executable verification | Exit condition |
|---|---|---|---|
| R0 Runtime boundary / scheduler | API lifecycle, fixed-step, async status ring | Host-owned Device / Context; single submission thread; Handle generation; advance/poll separation; Pause / Stop / Step / Reset; Step Queue priority over wall-clock Accumulator while Paused / Stopped; Stop cancellation accounting; Reset timeline clear; Fixed Delta変更時のseconds remainder保持; Tick Reservation ledger; GPU Commit / CPU Observation time ownership; failure replay without time duplication; per-World in-flight limit 1/2/4; aggregate Query Ring; non-blocking Query polling; common Substep schedule; Model / Collider / World / Runtime lifecycle | No double-accumulator, lost reserved time, duplicate event, stale handle / lease, query overwrite or premature Resource release |
| R1 Soft Body integration | Existing P06 analytical / WARP tests plus multi-tet fixture | GPU Tet stress / force contribution, Vertex→Tet CSR force Gather, mass integration, FEM CFL / Tracking limit aggregation into World common Substeps, inversion diagnostics, CPU double agreement | Multi-tet force integration and common Substep schedule pass with no float Atomic Add |
| R2 Cloth integration | Existing P05 cloth fixtures | Cloth velocity / position prediction, gravity / sustained Force / one-shot Impulse, Mass-weighted Wendland C2 distribution, exponential damping, pre-contact velocity reconstruction after non-contact constraints, Multi-iteration Stretch Coloring, Shear / Bend Gather Jacobi, World common Integration Segment schedule, dt_segment Compliance, Segment-start Lambda reset, Segment-local lifetime and constraint-frequency Temporal Quality Limit | Cloth prediction, force conservation, one-shot event application, timestep quality estimate and each constraint family pass analytic / FD tolerances |
| R3 Mixed Collision Runtime | Existing P07 broadphase / VT / EE / overflow fixtures plus mixed Model World | GPU Spatial Hash build / pair generation / dedup; deterministic Morton LBVH Node Build / Stackless Query; Large Primitive Bucket; all default Mesh Pair combinations; topological 1-ring self-collision filter; bidirectional correction; deterministic up-to-4-point Contact Patch reduction / split; zero-effective-mass guard; World-wide Coupled Projected Jacobi Contact Impulse with cross-Patch shared-vertex coupling; common Substep timing; capacity growth / rollback | GPU candidate set contains exhaustive reference set; non-responsive contacts never divide by zero; impulse residual / Coulomb cone / energy invariants meet Profile; manifold residual penetration / normal coverage / support hull metrics meet declared tolerance; no candidate silently dropped |
| R4 CCD / Initial Repair | Existing P08 point-triangle / initial repair plus moving VT / EE fixtures | Moving VT and EE CA; simultaneous TOI; initial contact; positive-time TOI; exact GPU Animation Trajectory Packet evaluation at arbitrary tau; conservative Span deviation / speed bounds; impact once; remaining-time World-wide reintegration; degenerate VT / EE zero-distance normal fallback; zero-effective-mass response guard; ContactGeometryUnresolved / ContactImpulseUnresolved reject; unresolved rollback | False no-hit = 0; trajectory bound violation = 0; unresolved / arbitrary-normal candidates never commit |
| R5 Animation / Render Interop | Existing P03 / P09 fixtures plus multi-key / discontinuity fixtures | CPU-generated immutable Animation Span Packet / Bounds; GPU arbitrary-tau Bone Hierarchy / Skin Matrix / Target Derivative evaluation; STEP / LINEAR / CUBICSPLINE; two-track Blend Weight and w_dot; BlendVelocity discontinuity; key / loop / root boundaries inserted into World Segments; missing-packet BLOCKED_BY_INPUT; Render Candidate validation before Commit; atomic Physics State / Render Slot generation switch; CPU-observed Committed-only Render Binding; Lease Generation validation; immediate/deferred Command List execution before Release; lifecycle / Device Loss | GPU trajectory position / velocity agree with independent CPU double reference; conservative speed / deviation bounds never underbound; UV / Color / Material / Index preserved; no solver scratch exposed; stale / double lease release rejected; no slot overwritten while leased / in-flight / unobserved; no normal-frame Physics Vertex readback |
| R6 Physical GPU performance / memory | Existing P11 profiles plus mixed collision profiles | GTX 1660 SUPER physical GPU pass times, CPU submission, frame time, 2k/8k/32k scaling, candidate/contact/CCD counts, rollback, library allocation and DXGI Budget | Workload results reported with all correctness gates; 3–40 FPS remains an evaluation range, not a universal guarantee |

### Runtime transaction regression cases

Add persistent fixture and executable tests for the following state paths before implementing the production scheduler.

1. **GPU accepted, then committed:** Dispatch submission succeeds; GPU commit control writes COMMITTED; Event Query completes; status Map becomes available; CPU contiguous completed prefix advances exactly once.
2. **GPU accepted, then rejected:** Dispatch submission succeeds; GPU status REJECTED due to NonFinite / Physical Gate; CPU must not advance completed time and Renderer must retain the prior committed state.
3. **Capacity block and replay:** Small capacity fixture forces overflow. Fill / Consumer / Commit do not run past capacity. Required count is returned asynchronously. Capacity grows on the submission thread. The same logical tick input and tick-start snapshot are replayed.
4. **CCD unresolved:** Iteration limit is reached. The candidate is not treated as No Hit; the tick is rejected / blocked and time is preserved.
5. **Out-of-order observation:** A later tick's query completes before an earlier tick. CPU completed time advances only when the earlier committed prefix is complete.
6. **Query ring pressure:** Fill the ring with delayed queries. Slots are not overwritten and normal polling does not Flush / synchronously wait.
7. **Render binding lease and generation:** Host obtains all Mesh Bindings for one World lease, binds SRVs, draws, unbinds every relevant Graphics / Compute stage, then calls release_render_bindings. AddRef preserves COM object lifetime only and does not extend the slot-content lease. Verify no slot is overwritten before release, Render Ring exhaustion blocks new submission without dropping time, and World / Device / Slot generation mismatch is diagnosed.
8. **Device loss:** Device Removed blocks new submissions. Recovery rebuilds from validated Asset / Initial State and resets the timeline; no false continuation from lost GPU state is reported.
9. **Mixed model indexing:** Multiple Model Instances with Soft Body / Cloth / None meshes use unique World Global IDs; collision filters and Render Bindings resolve to the intended mesh ranges.
10. **No hidden CPU fallback:** Device loss, WARP absence, capacity failure or GPU solver failure never switches the World to CPU physics.
11. **In-flight predecessor failure:** Submit Tick N and N+1. Force N to reject. Verify N+1 is blocked before any physics buffer access/write, then replay N from the same snapshot and resubmit N+1 with the new committed state generation.
12. **Contact Patch support manifold:** Use planar face contact, edge contact, corner contact, and a narrow folded cloth fixture. Verify deterministic up-to-4 support point reduction, tie-breaking by Contact Key, correct handling of point/line patches, and no increase beyond the declared penetration / support-region tolerance relative to the unreduced reference.
13. **Render Candidate / Commit atomicity:** Force Render Candidate geometry validation to fail after Physics solve succeeds. Verify neither Physics State Generation nor Committed Render Slot Generation changes and the prior render binding remains usable.
14. **Render Ring exhaustion:** Hold the CPU-observed Render Lease while forcing all Candidate Slots to be occupied. Verify the next Tick is not submitted, the Accumulator retains elapsed time, and a new submission proceeds only after Graphics SRV Unbind plus release_render_bindings.
15. **Multi-World Query Ring:** Create multiple independent Worlds with different in-flight limits. Verify Runtime Query Ring capacity covers the aggregate limits, World Tick Sequences and State Generations remain isolated, and shared Immediate Context submissions remain serialized.
16. **Pose history gap:** Omit a required Root / Bone / Kinematic sample interval. Verify the affected Tick becomes BLOCKED_BY_INPUT, no stale-pose extrapolation or false velocity is generated, and time remains pending until the interval is supplied.

17. **Tick Reservation time conservation:** Submit Tick N and N+1; force N to Capacity Block, CCD retry, and predecessor rejection in separate runs. Verify each Tick keeps the same time interval / sequence through retry, requested time is not added twice, committed time advances once, and CPU Observation does not mutate Accumulator. Accept single_step requests and verify each accepted Step adds exactly one fixed dt to requested time without changing wall-clock Accumulator; with pre-existing whole ticks and fractional remainder, verify Step Requests run first and leave Accumulator unchanged. Verify Stop cancels only unreserved Step Requests and preserves wall-clock Accumulator; Reset clears the Accumulator / Step Queue / timeline. Verify cancelled_requested_time is reported while Reserved / in-flight Step Requests are not canceled or double-committed.
18. **World common Substep:** Combine a stiff Soft Body, Cloth, and contact pair with different local FEM limits. Verify all solver input samples and Contact Candidate positions use identical Substep boundary times and dt; exceeding maximum_substeps blocks the entire Logical Tick.
19. **Cloth external input:** Compare total per-vertex Force / Impulse sum against Descriptor magnitude across multiple mesh resolutions. Verify uniform gravity yields acceleration independent of lumped vertex mass, Force start/end segmentation, one-shot Impulse event ID, fixed-vertex reaction, zero-target diagnostic, and no Force/Impulse double integration. Add TOI and Force-event split fixtures proving XPBD Lambda resets at each World Segment boundary, alpha_tilde uses dt_segment, and Lambda accumulates only inside that Segment.
20. **Lifecycle backpressure:** Call destroy_model / destroy_collider with in-flight Tick / Tick Reservation or Host Lease and verify BUSY without invalidating handles/resources. Verify unreserved Step Requests do not pin Model / Collider snapshots and execute against the latest World structure after the pre-submit gates clear. Call reset_world / destroy_world with each disallowed state, including whole-tick and fractional Accumulator remainder, and verify the documented BUSY / explicit Reset behavior. Execute Command List before Release, test stale/double lease generation rejection, then drain and shutdown after all Worlds, Accumulator demand and Step Requests are cleared.
21. **Degenerate Contact:** Build coincident VT and parallel / crossing EE fixtures with valid prior normal, Rest reference, velocity reference, and no valid fallback. Verify the fallback priority is deterministic and the final unresolved case rejects the Logical Tick without arbitrary-axis correction.
22. **LBVH stackless traversal:** Include N=0 / N=1 / N>=2, equal Morton codes, zero-extent axes, minimum / maximum quantization bounds, large-large overlap, regular-large cross-pairs, and degenerate Morton distributions. Verify exact Pair Set, Node Count / escape Sentinel, and no unbounded / stack traversal path.

The existing `fixed_step_controller` and `simulation_step_pipeline` remain synchronous CPU utilities for reference / test environment use. Production GPU Runtime tests must exercise the asynchronous scheduler and GPU commit status separately.

## Runtime phase exit evidence

For every R0-R6 gate, record the exact commit, CI workflow run, test IDs, GPU adapter/driver, WARP-versus-Hardware mode, expected/observed values, tolerance profile ID/hash, capacity / retry counters and known untested paths. A passing WARP run establishes shader behavior and resource ABI correctness only. A physical-GPU run is required for measured execution time and DXGI memory behavior.


## Design closure regression coverage

The v1 defaults and API contracts consolidated in `docs/design_closure.md` must have persistent fixtures before their production kernels are implemented. At minimum, add:
- broadphase Large Primitive vs regular-cell cross-pair and exact candidate-set fixtures;
- Contact Patch point / line / plane / folded-surface support-point fixtures;
- CA tolerance boundary and maximum-iteration Unresolved fixtures;
- Capacity Growth power-of-two / 1.5x / uint32 overflow / DXGI budget rejection fixtures;
- Fixed Delta Time change with fractional remainder preservation;
- multiple Worlds sharing one Runtime Query Ring;
- common Substep timing across mixed Soft Body / Cloth / Contact;
- Mass-weighted Force / Impulse distribution and event replay;
- deterministic Morton LBVH equal-key / zero-extent / stackless traversal;
- Contact Patch residual penetration / normal coverage / support hull metrics and degenerate normal Unresolved;
- Tick Reservation time conservation across retry and predecessor failure;
- stale Lease Generation, Destroy BUSY, Drain / Shutdown lifecycle;
- Render Candidate validation failure, CPU-observed commit, lease release, and Ring Backpressure fixtures.

Expected results are reviewed and immutable. A failing test must never regenerate its expected result automatically.


### State Bank invariants

- Exactly two persistent State Banks exist per World. Tick source bank is immutable until Commit; destination bank receives all writes.
- Reject / Block never changes active bank or State Generation.
- Segment / TOI snapshot is reusable scratch and never treated as a third persistent bank.
- Submit N and N+1 in command order. N+1 reads the bank selected by N only after its predecessor gate; force N to reject and assert that N+1 performs no Physics State access / writes.


### Integration Segment / Damping / Kinematic Velocity regression

23. **Segment prediction boundary:** Build a World Substep split by Force start/end, one-shot Impulse, and TOI. Verify Soft Body / Cloth / Contact / Animation Tracking all use identical Segment start/end timestamps and dt. Verify force evaluation is refreshed at every Segment start, state continues from the preceding Segment end, no zero-duration Segment is submitted, and Event ID prevents duplicate Impulse during Retry / TOI retiming.
24. **Linear damping integration:** For Dynamic Vertex, compare GPU output against analytical constant-force linear-damping cases: (a) zero damping, (b) positive damping, (c) small z=damping_rate*dt_segment using the series branch, (d) Force plus Segment-start Impulse, and (e) no-force velocity decay. Verify velocity and position formulas separately, finite behavior at damping_rate=0, and that Force / Impulse magnitude is independent of mesh resolution under the normalized mass-weighted distribution.
25. **Kinematic velocity ownership:** Drive Fixed / Kinematic Soft Body and Cloth vertices with STEP, LINEAR, CUBICSPLINE, Clip Blend, teleport, and reset trajectories. Compare Animation Target Velocity against the analytical derivative of the final Skinning Target Trajectory, including playback speed and blend-weight derivative. Verify the same velocity is used by Tracking, Contact Relative Velocity, friction, restitution and CCD. Verify inverse_mass=0 prevents impulse velocity mutation while the prescribed Kinematic Velocity remains intact.
26. **Impact continuation:** Resolve a TOI impact inside a World Substep, then simulate the remaining Segment time. Verify post-impact Dynamic Velocity is the next Segment input, Kinematic Velocity remains prescribed, XPBD Lambda resets at the new Segment boundary, and no Logical-Tick-wide velocity reconstruction erases the impact response.

27. **CCD discrete-path consistency:** Use a constant-velocity / constant-force damped segment with an analytically known semi-implicit path. Verify CA path_velocity and x_path(tau) match the TOI prefix state exactly, dt=tau is used for non-contact XPBD Compliance without recomputing the Dynamic Velocity Prediction, Kinematic geometry is evaluated from the exact Animation / Rigid Transform Trajectory at tau while endpoint chord deviation only bounds Swept AABB expansion and the CA speed bound is conservative, and the impact-updated state starts the residual Segment. Add a case where re-predicting with dt=tau would yield a different velocity and assert that the implementation follows the original discrete path.
28. **Kinematic CCD path approximation:** Use LINEAR and CUBICSPLINE animation targets, hierarchical skinning, Clip Blend and a rotating Kinematic Collider. Verify the evaluator returns conservative full-interval chord deviation and trajectory speed bounds (Bezier Control Hull for cubic translation, interval / derivative bound for hierarchy and skinning, Sagitta / angular-speed bound for fixed-axis Slerp). Verify the deviation bound inflates Swept AABB, CA evaluates exact trajectory distance and uses the speed bound, subdivision is deterministic up to depth 8, and an unprovable or excessive bound returns CCDUnresolved without treating the sweep as No-Hit. Verify contact velocity uses the curve derivative or v_linear + omega cross radius, never chord velocity.

29. **Contact projection / restitution separation:** Use a falling Dynamic body against a fixed plane with restitution 0 and 1, including a deliberately large initial overlap. Save v_pre_contact after non-contact constraints and before Contact Position XPBD. Verify positional depenetration alone does not create normal / tangential kinetic energy, restitution uses v_pre_contact, the Contact correction is never included in velocity reconstruction, and TOI Impact velocity remains unchanged until the residual Segment begins.
30. **Animation trajectory derivative:** For STEP / LINEAR / CUBICSPLINE, non-unit playback speed, time-varying Clip Blend weight, quaternion rotation and hierarchical skinning, compare GPU Target Velocity against an independently differentiated analytical trajectory. Verify STEP boundary discontinuity policy, no stale previous-frame velocity, and BLOCKED_BY_INPUT when derivative samples are unavailable.
31. **Kinematic Collider rigid velocity / sweep:** Rotate a Kinematic Triangle Mesh around a fixed origin and translate it simultaneously. Compare contact-point velocity against v_linear + omega cross radius. Verify the Slerp arc Sagitta bound and angular-speed bound enclose the true path, expanded Swept AABB includes the arc, CA evaluates the exact rotated triangle at each tau, and fast rotating thin geometry cannot be reported No-Hit because endpoint chord misses the arc.
32. **GPU Memory Budget sentinel / accounting:** Test maximum_gpu_resource_bytes values 0 and positive, dxgi_current_usage smaller / equal / larger than library_owned_current_bytes, host_reserved_gpu_bytes, DXGI budget exhaustion, and per-World memory_budget_bytes inheritance / override. Verify 0 means no configured cap, not zero allocation; Resource Group allocation is rejected before partial switch when either Runtime or World headroom is insufficient.
33. **Cloth Temporal Quality Limit:** Compare Stretch / Shear / Bend frequency estimates against analytical single-constraint oscillators and resolution-scaled cloth fixtures. Verify dt_limit_cloth_quality is treated as a quality heuristic rather than a stability proof, is aggregated with FEM / Tracking / World limits, and exceeding maximum_substeps blocks the whole Tick.
34. **World absolute Force / Impulse event time:** Submit Force / Impulse Events across Render Frame boundaries, fixed-step backlog, Pause / Step, Retry and Logical Tick boundary. Verify all start / end / event times are absolute World Simulation Time seconds, half-open event intervals apply once, and no event is shifted by Render Frame or Clip Local Time.


35. **GPU Animation Trajectory Packet / arbitrary TOI time:** Build STEP / LINEAR / CUBICSPLINE channels for translation, scale and quaternion rotation, including non-unit and negative Playback Speed, Loop wrap, hierarchy, mesh-node inverse and up to eight skin influences. Evaluate deterministic tau values inside each Animation Span on GPU and compare Bone / Skin Matrix / final Physics Vertex Position and analytic Velocity against an independent CPU-double evaluator. Verify Span Key / Loop boundaries are split, every reservation pins the same Packet Version, Retry reuses it, and missing coefficients / invalid quaternion normalization returns BLOCKED_BY_INPUT or Invalid Animation Input. Verify maximum trajectory Speed / Acceleration / Chord Deviation bounds enclose an independent interval reference and adversarial dense samples; any underbound rejects the fixture.
36. **Non-responsive Contact effective inverse mass:** Cover Static-Static, Kinematic-Kinematic, fixed-only surface vertices, and a mixed patch where some contacts have positive Dynamic inverse mass. Verify W_ii <= epsilon_inverse_mass skips Contact Position XPBD and Velocity Impulse, emits NON_RESPONSIVE_CONTACT exactly once per canonical contact, keeps all state finite, and allows responsive contacts in the same World to solve normally. Include W_ii just below / above the threshold.
37. **Coupled Contact Impulse:** Create planar multi-point support, two adjacent VT contacts sharing a Dynamic Vertex, EE / VT contacts across separate Patches sharing vertices, and a folded Cloth contact island. Compare the GPU Projected Jacobi result with CPU-double execution of the same update order and an independent small Delassus block oracle. Verify cross-contact coupling changes the result from independent one-shot Impulses, accumulated Normal Impulse stays non-negative, Tangent Impulse stays in the Coulomb disk including XPBD support impulse lambda_n / dt_contact, projected residual <= 1e-4, and restitution is applied once. Force a case that cannot converge within 16 iterations and verify CONTACT_IMPULSE_UNRESOLVED prevents Logical Tick Commit.
38. **Contact Temporal Quality Limit:** Verify contact_max_substep_delta_time defaults to 1/120 s, 0 explicitly disables only this Contact-specific cap, and the effective World dt is the minimum of Contact / Cloth / FEM / Tracking / world limits. Verify maximum_substeps overflow blocks the entire Logical Tick and does not independently subcycle Contact.
39. **BlendVelocity / Clip Blend trajectory:** Blend two clips with moving translation, rotating quaternion and time-varying Hermite Weight. Compare final Bone / Skinning Position and Velocity including w_dot terms. At explicit Discontinuity verify PreserveVelocity, ResetVelocity and BlendVelocity use the documented Dynamic Velocity formula and reset target history only at the event boundary. Verify Additive and three-or-more Track Blend are rejected as Unsupported in v1.
