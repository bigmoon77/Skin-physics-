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
    G->>D: snapshot committed state
    G->>D: Skinning / FEM / Cloth / Broadphase / CCD / Contact
    G->>D: Render Feedback Publish
    G->>D: GPU Tick Commit Control
    G->>Q: Copy small status + End Event Query
    API-->>H: submitted / pending / observed status
    H->>R: bind committed Render Binding
    R->>D: Draw
    Note over Q: Later frame, non-blocking Poll
    H->>API: poll()
    API->>Q: GetData(DONOTFLUSH)
    Q-->>API: Event Query complete
    API->>Q: Map status (DO_NOT_WAIT)
    Q-->>API: COMMITTED status + tick sequence
    API-->>H: contiguous completed tick prefix
```

GPU command submission and CPU observation are separate events. Event Query completion proves command completion; the GPU Status Buffer determines whether a Logical Tick committed.

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

## 5. Key implementation invariants

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
