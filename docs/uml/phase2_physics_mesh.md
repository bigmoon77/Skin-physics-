# Phase 2: Physics Mesh Generation UML

## Class diagram

\`\`\`mermaid
classDiagram
    class FtetwildRunner {
        +Run(surfaceMesh, settings) TetrahedralizerResult
    }
    class FtetwildSettings {
        +path executable_path
        +float target_edge_length
        +float relative_envelope
        +uint32 timeout_milliseconds
    }
    class GmshReader {
        +ReadAscii22(path) RawTetMeshResult
    }
    class RawTetMesh {
        +Vec3[] positions
        +uint4[] tetrahedra
    }
    class PhysicsMeshBuilder {
        +Build(rawMesh, settings) PhysicsMeshBuildResult
    }
    class PhysicsMeshBuildSettings {
        +float density
        +float minimum_volume
        +float minimum_quality
        +float minimum_dihedral_degrees
        +uint32 maximum_tetrahedra
    }
    class PhysicsMesh {
        +Vec3[] rest_positions
        +Tetrahedron[] tetrahedra
        +BoundaryTriangle[] boundary_triangles
        +uint32[] vertex_tetrahedron_offsets
        +uint32[] vertex_tetrahedron_ids
        +float[] mass
        +float[] inverse_mass
    }
    class Tetrahedron {
        +uint4 vertices
        +float rest_volume
        +float quality
        +float minimum_dihedral_degrees
        +Mat3 rest_inverse
        +Vec3 rest_shape_gradients[4]
    }

    FtetwildRunner --> GmshReader
    FtetwildRunner ..> FtetwildSettings
    GmshReader --> RawTetMesh
    PhysicsMeshBuilder --> PhysicsMesh
    PhysicsMeshBuilder ..> PhysicsMeshBuildSettings
    PhysicsMesh *-- Tetrahedron
    PhysicsMesh *-- BoundaryTriangle
\`\`\`

## Generation sequence

\`\`\`mermaid
sequenceDiagram
    participant AssetBuild
    participant Runner as FtetwildRunner
    participant Tool as FloatTetwild_bin.exe
    participant Reader as GmshReader
    participant Builder as PhysicsMeshBuilder
    participant Cache as AssetCache

    AssetBuild->>Runner: Run(surfaceMesh, settings)
    Runner->>Runner: Write temporary OBJ
    Runner->>Tool: -i OBJ -o MSH --la length --no-binary
    Tool-->>Runner: MSH 2.2 ASCII
    Runner->>Reader: Read(output.msh)
    Reader-->>Runner: RawTetMesh
    Runner-->>AssetBuild: RawTetMesh
    AssetBuild->>Builder: Build(rawMesh, qualitySettings)
    Builder->>Builder: Orient tetrahedra and compute rest data
    Builder->>Builder: Extract boundary faces and build vertex-tet CSR
    Builder->>Builder: Lump mass
    Builder-->>AssetBuild: PhysicsMesh
    AssetBuild->>Cache: Serialize validated PhysicsMesh
\`\`\`

## Ownership and execution boundary

- fTetWild is an external offline executable; it is not linked into SkinPhysics and is not invoked by the runtime solver.
- The runner writes temporary OBJ and requests ASCII Gmsh MSH 2.2 output.
- GmshReader only parses node and tetrahedron elements. Surface triangles are derived from the validated tetrahedral volume boundary.
- PhysicsMeshBuilder owns orientation correction, element quality validation, rest matrix / shape gradient computation, boundary extraction, CSR adjacency, and lumped mass.
- Any invalid index, degenerate element, non-manifold face, or capacity overflow fails the build before a PhysicsMesh is accepted.
