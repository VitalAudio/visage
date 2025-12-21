# Testing Guide

## Running Tests

### Standard Environment

```bash
cd build
ctest --output-on-failure
```

### Headless/CI Environment

For headless environments (Docker, CI servers, etc.) where GPU acceleration isn't available:

```bash
cd build
XDG_RUNTIME_DIR=/tmp xvfb-run -a -s "-screen 0 1024x768x24" ctest --output-on-failure
```

#### Required Dependencies

Install these packages for headless testing on Ubuntu/Debian:

```bash
apt-get update && apt-get install -y \
  xvfb \
  mesa-vulkan-drivers \
  vulkan-tools \
  libgl1-mesa-dri \
  libglx-mesa0 \
  mesa-utils
```

These provide:
- **Xvfb**: Virtual X11 display server
- **mesa-vulkan-drivers**: Software Vulkan rendering (lavapipe)
- **mesa-utils**: OpenGL software rendering (llvmpipe)

## Test Structure

The Visage test suite includes 220+ test cases across:

- **visage_utils** (86 tests) - Core utilities, string operations, events, threading
- **visage_ui** (66 tests) - Layout system, frames, scroll bars, undo/redo
- **visage_graphics** (44 tests) - Canvas, colors, gradients, paths, fonts
- **visage_widgets** (20 tests) - UI widgets (buttons, toggles)
- **visage_app** (4 tests) - Integration tests with screenshot validation

## Test Coverage Analysis

See [TEST_COVERAGE_ANALYSIS.md](TEST_COVERAGE_ANALYSIS.md) for detailed coverage analysis and improvement recommendations.

Current estimated coverage: ~45-50% overall
- Excellent coverage: visage_utils (85%), visage_ui (85%)
- Needs improvement: visage_graphics (35%), visage_widgets (10%), visage_app (10%)
- No coverage: visage_windowing (0%)
