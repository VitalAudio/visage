# Visage agent guide

Visage is a GPU-accelerated, cross-platform C++ library for native UI and 2D graphics. This file is
the single source of truth for coding agents; `CLAUDE.md` imports it.

## Repository layout

- `visage_graphics/` canvas, shapes, paths, text, layers, effects and the renderer.
- `visage_graphics/gpu/` the graphics layer. `gpu.h` is the public interface, `gpu_backend.h` the
  backend interface, `gpu.cpp` the shared frame and resource logic, `gpu_shader.cpp` shader loading.
  Backends: `gpu_metal.mm` (macOS), `gpu_vulkan.cpp` (Linux), `gpu_d3d11.cpp` (Windows) and
  `gpu_gles.cpp` (WebGL2 on Emscripten). There is no bgfx.
- `visage_graphics/shaderc/` the in-tree shader compiler (glslang and SPIRV-Cross through
  FetchContent). It builds the `shader_compiler` executable, which emits SPIR-V, MSL and DXBC.
- `visage_graphics/shaders/` shader sources. Embedded shaders are always compiled at build time.
- `visage_windowing/` native windows and events per platform (`macos/`, `win32/`, `linux/` for X11,
  `emscripten/`).
- `visage_ui/`, `visage_widgets/`, `visage_app/`, `visage_utils/`, `visage_file_embed/` the
  remaining libraries.
- `examples/` example apps. `manual_tests/` an interactive app for checks automated tests can't do
  (see its README).

## Build

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

Useful options: `VISAGE_BUILD_TESTS`, `VISAGE_BUILD_EXAMPLES` (both on for a top-level build),
`VISAGE_ADDRESS_SANITIZER`, `VISAGE_ENABLE_GRAPHICS_DEBUG_LOGGING`, and `VISAGE_LIVE_SHADERS` (off by
default, links the shader compiler into apps for live shader editing). `VISAGE_SHADER_COMPILER`
points at a prebuilt `shader_compiler` instead of building one.

Linux needs the X11 and Vulkan development packages listed in `.github/workflows/linux-test.yaml`.
Set `VISAGE_VULKAN_VALIDATION=1` to enable Vulkan validation layers at runtime.

Emscripten:

```sh
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release
cmake --build build-web
```

Web examples land in `build-web/examples/builds/`; serve that directory over HTTP to run them.

## Test

Tests use Catch2 and run through ctest:

```sh
ctest --test-dir build --output-on-failure
```

Test targets are `VisageGraphicsTests`, `VisageUiTests`, `VisageWidgetsTests`, `VisageUtilsTests`
and `VisageIntegrationTests`; each can also be run directly with a Catch2 filter. Tests that need a
display call Catch2's `SKIP` when none is available. On a headless Linux machine run ctest
under Xvfb (`xvfb-run -a ctest ...`) with Mesa's lavapipe Vulkan driver installed. CI also builds
with clang, ASan (`ASAN_OPTIONS=detect_leaks=0`) and IPO on Linux.

Rendering changes should be checked on every backend they touch (Vulkan, Metal, D3D11, WebGL2), not
just the one on the machine at hand. Device-loss recovery can be exercised with
`gpu::simulateDeviceLoss()`.

## Formatting

- Run clang-format 18 (18.1.8, the version upstream CI uses) on every changed `.cpp` and `.h` file
  before committing. Newer versions format some files differently from CI. If it isn't installed,
  `uvx clang-format==18.1.8 -i <files>` runs the matching PyPI build.
- CI checks everything except `visage_graphics/third_party/` and `visage_windowing/macos/`.
- `.clang-format` uses `IncludeBlocks: Regroup`, so a blank line does not preserve include order.
  When one header must come before another (for example `<X11/Xlib.h>` before
  `<vulkan/vulkan_xlib.h>`), separate the blocks with a `#define` or other non-include line.
- Enable the repo's pre-commit format check once per clone with
  `git config core.hooksPath .githooks`. `.claude/settings.json` adds the same check before
  `git commit` for Claude Code.

## Commits and pull requests

- No code comments in PR diffs.
- Never credit an AI tool: no `Co-Authored-By`, `Claude-Session`, "Generated with" lines or similar
  in commits, PR descriptions or other artifacts.
- Commit messages follow the existing style: one plain past-tense summary line, such as "Fixed the
  Linux build by including X11 before the Vulkan Xlib header".
- PRs are merged with "Rebase and merge", so branches must be linear: no merge commits. Rebase onto
  `main` instead of merging it in.
- Keep PRs under 8k changed lines; split larger work. Stack PRs only when needed, and rebase the
  stacked branch once its base merges.
- CI (format, Linux, macOS, Windows, MinGW tests) runs on the upstream repository, VitalAudio/visage.
  On a fork without Actions, run the format check and tests locally before opening a PR.

## Known pitfalls

- HLSL `mul(vec, mat2)` fills matrices by row and GLSL by column. Write 2D rotations as dot products
  so every backend gets the same result.
- The first configure of a fresh Visual Studio build directory on Windows can fail inside the
  spirv_cross FetchContent sub-build. Re-running configure succeeds; Ninja is unaffected.
- GNOME on Wayland sends no frame callbacks while the monitor is off, so frame rates measured then
  look like ~10 fps. Use a headless compositor or Xvfb for measurements.
- The Metal backend treats any command buffer error as device loss, so a bad draw call shows up as a
  device rebuild every frame rather than a validation error.
