# Phase 2: Physics Mesh, Skin Weight and Cache UML

## Class diagram

```mermaid
classDiagram
    class FtetwildRunner {
        +Run(surfaceMesh, settings) TetrahedralizerResult
    }
    class GmshReader {
        +ReadAscii22(path) RawTetMeshResult
    }
    class PhysicsMeshBuilder {
        +Build(rawMesh, settings) PhysicsMeshBuildResult
    }
    class PhysicsMesh {
        +Vec3[] rest_positions
        +Tetrahedron[] tetrahedra
        +BoundaryTriangle[] boundary_triangles
        +uint32[] vertex_tetrahedron_offsets
        +uint32[] vertex_tetrahedron_ids
        +float[] mass
        +float[] inverse_mass
        +PhysicsSkinWeight[] skin_weights
    }
    class SkinWeightTransfer {
        +Transfer(renderMesh, skin, physicsMesh, settings) Result
    }
    class SurfaceBVH {
        +Build(renderMesh)
        +FindClosestTriangle(point) BarycentricResult
    }
    class HarmonicWeightSolver {
        +Solve(graphLaplacian, boundaryWeights, settings) JointWeights
    }
    class PhysicsAssetCache {
        +Save(path, physicsMesh, identity) WriteResult
        +Load(path, expectedIdentity) ReadResult
    }
    class CacheIdentity {
        +uint32 schema_version
        +string source_asset_id
        +string source_asset_hash
        +string generator_id
        +string generator_version
        +string generator_options
        +uint32 coordinate_system_version
        +uint32 material_layout_version
    }

    FtetwildRunner --> GmshReader
    GmshReader --> PhysicsMeshBuilder
    PhysicsMeshBuilder --> PhysicsMesh
    SkinWeightTransfer --> SurfaceBVH
    SkinWeightTransfer --> HarmonicWeightSolver
    SkinWeightTransfer --> PhysicsMesh
    PhysicsAssetCache --> PhysicsMesh
    PhysicsAssetCache ..> CacheIdentity
```

## Physics Mesh generation sequence

```mermaid
sequenceDiagram
    participant AssetBuild
    participant Runner as FtetwildRunner
    participant Tool as FloatTetwild_bin.exe
    participant Reader as GmshReader
    participant Builder as PhysicsMeshBuilder

    AssetBuild->>Runner: Run(surfaceMesh, settings)
    Runner->>Runner: Write temporary OBJ
    Runner->>Tool: Launch fTetWild, request MSH 2.2 ASCII
    Tool-->>Runner: volume.msh
    Runner->>Reader: Read(output.msh)
    Reader-->>Runner: RawTetMesh
    Runner-->>AssetBuild: RawTetMesh
    AssetBuild->>Builder: Build(rawMesh, qualitySettings)
    Builder->>Builder: Orient tetrahedra and compute rest data
    Builder->>Builder: Extract boundary / build CSR / lump mass
    Builder-->>AssetBuild: Validated PhysicsMesh
```

## Skin weight transfer sequence

```mermaid
sequenceDiagram
    participant AssetBuild
    participant Transfer as SkinWeightTransfer
    participant BVH as SurfaceBVH
    participant Solver as HarmonicWeightSolver
    participant Physics as PhysicsMesh

    AssetBuild->>Transfer: Transfer(renderMesh, skin, physicsMesh, settings)
    Transfer->>BVH: Build Render Mesh triangle BVH
    loop Each Physics boundary vertex
        Transfer->>BVH: Find closest triangle
        BVH-->>Transfer: Triangle index + barycentric weights
        Transfer->>Transfer: Interpolate up to 8 bone weights
    end
    Transfer->>Transfer: Build unique positive weighted edge graph
    Transfer->>Transfer: Check every component reaches boundary
    loop Each active joint
        Transfer->>Solver: Solve L_II w_I = -L_IB w_B
        Solver-->>Transfer: Weights + relative residual
    end
    Transfer->>Transfer: Clamp, keep top 8, normalize and validate
    Transfer->>Physics: Commit weights only after full success
    Transfer-->>AssetBuild: Diagnostics + convergence metrics
```

## Cache sequence

```mermaid
sequenceDiagram
    participant AssetBuild
    participant Cache as PhysicsAssetCache
    participant Disk

    AssetBuild->>Cache: Load(path, expectedIdentity)
    Cache->>Disk: Read header, identity and payload
    Cache->>Cache: Compare complete identity and checksum
    alt Exact identity and valid payload
        Cache-->>AssetBuild: Valid PhysicsMesh cache
    else Missing, stale or corrupt
        Cache-->>AssetBuild: needs_rebuild + diagnostic
        AssetBuild->>Cache: Save(path, rebuiltPhysicsMesh, identity)
        Cache->>Disk: Write temporary file, flush, replace
        Cache-->>AssetBuild: WriteResult
    end
```

## Ownership and execution boundary

- fTetWild is an external offline executable; it is not linked into SkinPhysics and is not invoked by the runtime solver.
- PhysicsMeshBuilder owns orientation correction, element quality validation, rest matrix / shape gradient computation, boundary extraction, CSR adjacency, and lumped mass.
- SkinWeightTransfer is an offline Asset Build operation. It uses a BVH over Render Mesh triangles for boundary lookup and a positive graph Laplacian for interior weights.
- Skin weight transfer builds a temporary result and only commits it to PhysicsMesh after every component, solve residual, influence count and normalization check passes.
- PhysicsAssetCache stores deterministic data in an explicit little-endian format. The FNV-1a payload checksum detects accidental corruption; it is not a cryptographic authenticity mechanism. Source asset identity must therefore include a separately computed source hash.
- Cache identity mismatch, corrupt data, unsupported file versions and invalid payloads require an explicit rebuild. No invalid cache silently falls back to a partial PhysicsMesh.
- The current cache schema contains Physics Mesh rest data, mass, adjacency, boundary faces and optional skin weights. Future payload sections require a schema version increase and round-trip tests.
