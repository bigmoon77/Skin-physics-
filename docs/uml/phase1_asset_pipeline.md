# Phase 1: Asset / Importer UML

## Class diagram

\`\`\`mermaid
classDiagram
    class ImporterFacade {
        +ImportModel(path, options) ImportResult
    }
    class ImportOptions {
        +bool has_obj_unit_scale
        +float obj_unit_scale
    }
    class ObjImporter {
        +Import(path, options) ImportResult
    }
    class GltfImporter {
        +Import(path, options) ImportResult
    }
    class ModelValidator {
        +Validate(model) Diagnostic[]
    }
    class Model {
        +string source_path
        +Mesh[] meshes
        +Material[] materials
        +Node[] nodes
        +Skin[] skins
        +AnimationClip[] animations
    }
    class Mesh {
        +Vertex[] vertices
        +uint32[] indices
        +Submesh[] submeshes
        +PhysicsMode mode
    }
    class Vertex {
        +float3 position
        +float3 normal
        +float4 tangent
        +float2 texcoord_0
        +uint16 joints[8]
        +float weights[8]
        +float physics_weights[7]
    }
    class Node {
        +uint32 parent_index
        +uint32 mesh_index
        +uint32 skin_index
        +float4x4 local_matrix
    }
    class Skin {
        +uint32[] joints
        +float4x4[] inverse_bind_matrices
    }
    class AnimationClip {
        +float duration
        +AnimationTrack[] tracks
    }
    class AnimationTrack {
        +uint32 node_index
        +AnimationPath path
        +Interpolation mode
        +AnimationKey[] keys
    }
    class Material {
        +float4 base_color_factor
        +float metallic_factor
        +float roughness_factor
    }

    ImporterFacade --> ObjImporter
    ImporterFacade --> GltfImporter
    ImporterFacade --> ModelValidator
    ObjImporter --> Model
    GltfImporter --> Model
    Model *-- Mesh
    Mesh *-- Vertex
    Model *-- Node
    Model *-- Skin
    Model *-- AnimationClip
    AnimationClip *-- AnimationTrack
    Model *-- Material
    Node --> Mesh
    Node --> Skin
\`\`\`

## Import sequence

\`\`\`mermaid
sequenceDiagram
    participant Client
    participant Facade as ImporterFacade
    participant Loader as ObjImporter / GltfImporter
    participant Common as Common Model
    participant Validator as ModelValidator

    Client->>Facade: ImportModel(path, options)
    Facade->>Facade: Determine extension
    alt OBJ
        Facade->>Loader: Import OBJ with explicit unit scale
        Loader->>Loader: Parse vertices / UV / normals / faces / MTL
        Loader->>Common: Build vertices, submeshes, materials
    else glTF / GLB
        Facade->>Loader: Parse document and load buffers
        Loader->>Loader: Decode accessors / primitives / nodes
        Loader->>Loader: Decode skins and animation tracks
        Loader->>Common: Build common model
    end
    Common-->>Facade: Imported model
    Facade->>Validator: Validate(model)
    Validator-->>Facade: Diagnostics
    Facade-->>Client: ImportResult
\`\`\`

## Ownership boundaries

- ImporterFacade owns format selection and common validation.
- ObjImporter and GltfImporter only convert source data into the common asset model.
- cgltf is compiled once in cgltf_impl.c and used only by GltfImporter.
- Simulation, solvers and GPU resources do not include cgltf.h.
- Imported data owns its strings, arrays and embedded image bytes. No pointers into cgltf_data survive cgltf_free().
