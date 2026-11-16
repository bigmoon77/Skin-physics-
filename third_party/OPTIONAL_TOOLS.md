# Optional Offline Tools

## fTetWild

- Project: https://github.com/wildmeshing/fTetWild
- Pinned reference commit: 8118f810478e0e65a7bf2d8cecdc5e203a876e97
- License: Mozilla Public License 2.0 (MPL-2.0)
- Purpose: offline tetrahedralization of Soft Body surface meshes.
- Distribution: Skin Physics does not bundle fTetWild source or executable and does not link it into the runtime library.
- Integration: FtetwildRunner launches an externally installed FloatTetwild executable, requests ASCII Gmsh MSH 2.2 output, and converts that file into the internal raw tetrahedral mesh.

Install/build fTetWild separately and provide its executable path in ftetwild_settings. fTetWild's build uses its own CMake project and numerical dependencies; CMake is not required to build Skin Physics itself.

Example command issued by the adapter:

    FloatTetwild_bin.exe -i surface.obj -o volume.msh --la 0.05 --epsr 0.001 --no-binary --is-quiet --log ftetwild.log

The actual ideal edge length and relative envelope are supplied through ftetwild_settings. The tool's output is validated by the Gmsh reader and PhysicsMeshBuilder before use.
