# Phase 12 — GPU Runtime Transaction / Render Integration

## 1. Component boundary

```mermaid
flowchart LR
    Host[Host Application]
    Renderer[D3D11 Renderer]
    API[SkinPhysics Public API]
    Manager[Simulation Manager]
    Scheduler[GPU Tick Scheduler]
    Importer[Asset Importer / Builder]
    World[Simulation World]
    FEM[Soft Body FEM Solver]
    Cloth[Cloth XPBD Solver]
    Collision[Collision / CCD]
    Feedback[Render Feedback]
    GPU[D3D11 Device / Immediate Context]

    Host --> API
    Renderer --> GPU
    API --> Manager
    Importer --> World
    Manager --> Scheduler
    Scheduler --> World
    World --> FEM
    World --> Cloth
    World --> Collision
    FEM --> GPU
    Cloth --> GPU
    Collision --> GPU
    Feedback --> GPU
    World --> Feedback
    API --> Renderer
    API --> GPU
```

Host owns the D3D11 Device / Immediate Context and Renderer. SkinPhysics retains COM references and owns Simulation World state, physics resources, and the asynchronous scheduler. The same Immediate Context is used by Host Renderer and SkinPhysics on one submission thread.

## 2. Logical tick normal path

```mermaid
sequenceDiagram
    participant H as Host
    participant API as SkinPhysics API
    participant S as Simulation Manager
    participant G as GPU Tick Scheduler
    participant D as D3D11 Immediate Context
    participant Q as Query / Status Ring
    participant R as Renderer

    H->>API: advance(frame_delta_time)
    API->>S: accumulate active elapsed
    S->>G: request logical tick(s)
    G->>G: reserve per-tick Render Candidate Slot
    G->>D: snapshot committed state
    G->>D: Skinning / FEM / Cloth / Broadphase / CCD / Contact
    G->>D: prepare Render Candidate (Position / Normal / Tangent)
    G->>D: validate Render Candidate + Physics State
    G->>D: GPU Commit Control (Physics State + Render Slot atomically)
    G->>Q: copy tick / state / render-slot status + Event Query
    API-->>H: submitted / pending / observed status
    Note over Q: Later frame, non-blocking Poll
    H->>API: advance(frame_delta_time) / poll()
    API->>Q: GetData(DONOTFLUSH)
    Q-->>API: Event Query complete
    API->>Q: Map status (DO_NOT_WAIT)
    Q-->>API: COMMITTED + tick / state generation / render slot
    API-->>H: contiguous CPU-observed committed prefix
    H->>API: get_render_binding(world, model, mesh)
    API-->>H: Render Slot lease + read-only SRVs
    H->>R: bind Render Binding
    R->>D: Draw
    H->>R: Unbind VS / HS / DS / GS / PS / CS SRVs
    H->>API: release_render_bindings(world)
```

GPU command submission and CPU observation are separate events. Event Query completion proves command completion; the GPU Status Buffer determines whether a Logical Tick committed. Render Candidate is produced before Commit, but its slot becomes visible only when the same GPU Commit Control atomically selects the committed Physics State and Render Slot. The Host receives only the latest CPU-observed committed prefix. A World has one active Render Lease, and its ring contains maximum_in_flight_ticks + 1 slots; a slot cannot be reused until it is no longer in flight / unobserved and its Host lease has been released.

The Runtime permits up to two submitted-but-not-observed ticks. Each immutable Dispatch Plan contains the expected predecessor sequence and source State Generation. Tick N+1 checks the GPU Commit result of N before any Physics State read/write. If N fails, N+1 is blocked before touching Physics State and must be resubmitted after N is successfully replayed.

## 3. Capacity / CCD failure and retry

```mermaid
sequenceDiagram
    participant G as GPU Tick Scheduler
    participant D as D3D11 GPU
    participant Q as Query / Status Ring
    participant C as CPU Runtime Control

    G->>D: preserve tick-start snapshot
    G->>D: Count / Scan / Capacity Check
    D-->>G: overflow or unresolved CCD flag
    G->>D: set BLOCKED / REJECTED; suppress downstream writes
    G->>Q: Copy small error status + Event Query
    Note over Q: GPU completion remains asynchronous
    C->>Q: Poll Event Query
    Q-->>C: completed
    C->>Q: Map status DO_NOT_WAIT
    Q-->>C: required capacity / error counters
    C->>C: grow validated GPU resource on submission thread
    C->>G: retry same logical tick / same initial snapshot
    G->>D: rerun with unchanged tick input
    D-->>G: Commit only after all gates pass
```

No candidate is silently dropped. The failed working state is not published to Renderer. If the error cannot be repaired within configured limits, the World remains blocked or failed and simulation time is not advanced.

## 4. Submission and completion state

```mermaid
stateDiagram-v2
    [*] --> Ready
    Ready --> PendingInput: advance / tick requested
    PendingInput --> Submitted: required input ready + slot available
    Submitted --> GpuExecuting
    GpuExecuting --> GpuCommitted: GPU commit control passes
    GpuExecuting --> BlockedByCapacity: capacity overflow
    GpuExecuting --> BlockedByCCD: CCD unresolved / iteration limit
    GpuExecuting --> Rejected: nonfinite / physical gate failure
    BlockedByCapacity --> Submitted: capacity grown + same tick replay
    BlockedByCCD --> Submitted: substep / CCD budget adjusted + replay
    GpuCommitted --> CpuObserved: Event Query + COMMITTED status + contiguous prefix
    CpuObserved --> Ready
    Rejected --> Failed
    Failed --> [*]
```

The CPU-observed completed time advances only from the contiguous COMMITTED tick prefix. Render Binding exposes committed data only.

## 5. In-flight predecessor dependency

```mermaid
sequenceDiagram
    participant CPU as Runtime CPU Scheduler
    participant GPU as GPU Tick Control
    participant N as Tick N Passes
    participant N1 as Tick N+1 Passes

    CPU->>GPU: Immutable Plan N (expected predecessor = N-1)
    GPU->>N: tick_begin_control validates predecessor + state generation
    N->>GPU: Commit Control
    alt Tick N commits
        GPU->>GPU: committed_sequence = N; advance state_generation
        CPU->>GPU: Immutable Plan N+1 (expected predecessor = N)
        GPU->>N1: gate matches; execute from committed generation N
    else Tick N rejects or blocks
        GPU->>GPU: retain previous committed sequence / generation
        CPU->>GPU: previously submitted Plan N+1 reaches tick_begin_control
        GPU-->>N1: BLOCKED_BY_PREDECESSOR; no Physics State read/write
        CPU->>GPU: replay Tick N from same snapshot
        N->>GPU: successful Commit N
        CPU->>GPU: resubmit Tick N+1 with new predecessor generation
    end
```

Every budgeted Physics Pass verifies the immutable Tick Plan / scheduler gate before touching Physics State. Commit failure must leave the committed State Set and its generation unchanged. Later submitted ticks are replayed only after their predecessor commits.

## 5. Persistent Physics State Bank transaction

```mermaid
sequenceDiagram
    participant CPU as Runtime Scheduler
    participant GPU as D3D11 Command Stream
    participant A as Active State Bank
    participant B as Inactive State Bank
    participant S as Substep Snapshot
    CPU->>GPU: Submit Tick N (Plan source=A, destination=B)
    GPU->>A: Read-only source state
    GPU->>B: FEM / Cloth / Contact updates
    GPU->>S: Save current substep segment before TOI retime
    alt All physics and render gates pass
        GPU->>GPU: Commit Control
        GPU->>GPU: active bank A -> B, generation++
    else Reject / Block
        GPU->>GPU: Keep active bank A and generation unchanged
        GPU->>B: Discard / overwrite destination on retry
    end
    CPU->>GPU: Retry Tick N with same source A / immutable input
    GPU->>B: Rebuild destination from A
```

The active bank is the complete Logical Tick rollback source. The inactive bank is disposable destination state. The substep snapshot is reusable TOI / segment scratch, not a third persistent bank.

## 6. Key implementation invariants

1. A Logical Tick is the smallest GPU Commit transaction.
2. All GPU passes are issued on one ordered D3D11 Immediate Context.
3. GPU-generated variable work counts use GPU-generated DispatchIndirect arguments or fixed upper-bound Dispatch with a GPU-side lane guard; no synchronous Count Readback is used to size a dispatch.
4. Any Buffer capacity error blocks Fill / Consumer / Commit and records required capacity.
5. Rollback replays the exact same Logical Tick input and initial state.
6. Soft Body Tet Force uses Tet Contribution + Vertex CSR Gather; no floating-point Atomic Add.
7. Cloth and Contact XPBD apply the same projected Lambda Delta to both Lambda and Position.
8. Render Feedback reads only the committed physics state and is published after the Tick succeeds.
9. Full Physics Vertex / Contact / Lambda readback is prohibited in the normal frame path.
10. WARP validates shader execution and readback correctness; physical GPU runs validate performance and memory.


## 6. Render Slot backpressure

```mermaid
stateDiagram-v2
    [*] --> Free
    Free --> CandidateReserved: tick plan reserves slot
    CandidateReserved --> CandidateReady: feedback + validation pass
    CandidateReserved --> Free: tick rejected
    CandidateReady --> GpuCommitted: commit control selects state + slot
    GpuCommitted --> CpuObserved: Event Query + COMMITTED status + contiguous prefix
    CpuObserved --> HostLeased: get_render_binding
    HostLeased --> CpuObserved: unbind all stages + release_render_bindings
    CpuObserved --> Free: superseded by a newer observed commit
```

A Host-leased slot, an unobserved committed slot, or an in-flight candidate is never overwritten. When all slots are occupied, the next Logical Tick remains pending in the Runtime Accumulator. No time or contact candidate is dropped.
