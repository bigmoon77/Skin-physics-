# Skin Physics

Skin Physics is a GPU-oriented C++ library for real-time Soft Body and Cloth simulation on Direct3D 11.

The repository is being implemented in stages. The current implementation milestone is **Phase 1: common asset model and importers**.

## Current implementation

- C++17, Visual Studio solution and project files.
- Common asset model for meshes, vertices, materials, nodes, skins and animation clips.
- OBJ importer with polygon triangulation, negative indices, smoothing groups and basic MTL material factors.
- glTF 2.0 / GLB importer with triangle, triangle-strip and triangle-fan primitives, PBR material factors, node hierarchy, up to eight skin influences, inverse bind matrices, and STEP / LINEAR / CUBICSPLINE TRS animation tracks.
- Common model validation and diagnostics.
- Importer-only dependency: cgltf 1.15 (MIT), pinned in third_party/cgltf.

OBJ has no standard unit metadata. Callers must set has_obj_unit_scale = true and provide obj_unit_scale in meters per OBJ unit.

## Build

Open SkinPhysics.sln in Visual Studio 2022 and build the x64 configuration.

The solution contains:
- SkinPhysics: static library.
- SkinPhysicsTests: console test executable.

The CI workflow builds the Visual Studio solution and runs the test executable on Windows.

## Basic usage

\`\`\`cpp
#include "skin_physics/asset/importer.hpp"

skin_physics::asset::import_options options;
options.has_obj_unit_scale = true;
options.obj_unit_scale = 0.01f; // OBJ coordinates are centimeters.

const skin_physics::asset::import_result result =
    skin_physics::asset::import_model("character.obj", options);
if (!result.success)
{
    for (const skin_physics::asset::diagnostic& item : result.diagnostics)
    {
        // Route item.message / item.code to the application's diagnostic UI.
    }
}
\`\`\`

For glTF and GLB, the importer reads the glTF meter-based coordinates and does not require an OBJ unit scale.

## Importer limits in this milestone

- Morph targets and morph-weight animation are rejected.
- Draco-compressed geometry is rejected because no Draco decoder is bundled.
- Meshopt-compressed buffer views are rejected because no meshoptimizer decoder is bundled.
- Only triangle, triangle-strip and triangle-fan primitives are imported.
- Texture image bytes / URIs are preserved as asset references; image transcoding and GPU texture creation belong to later render-resource stages.
- OBJ assumes the caller's coordinates are already right-handed Y-up and only applies the explicit unit scale.

These limits are diagnosed instead of silently producing incomplete geometry.
