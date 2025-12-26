# Faveworm - Agent Development Guide

This document provides information for AI agents and developers working on the Faveworm oscilloscope project.

## Project Overview

Faveworm is a real-time analog oscilloscope waveform visualization built with the Visage graphics framework. It features physics-based beam rendering, phosphor persistence, and a 360° SVF filter for XY mode visualization.

**Key Technologies:**
- C++17
- Visage UI/Graphics Framework
- Custom DSP components (SVF filter, RPM oscillator)
- Cross-platform: Desktop (macOS/Windows/Linux) + Web (Emscripten/WebGL)

## Project Structure

```
Faveworm/
├── faveworm.cpp          # Main application (oscilloscope, audio player, UI)
├── FilterMorpher.h       # 360° filter morphing + SimpleSVF implementation
├── FilterJoystick.h      # UI widgets (knobs, sliders, joystick, switches)
├── TestSignalGenerator.h # RPM oscillator test signal for XY mode
├── dsp/                  # DSP components
│   ├── dfl_StateVariableFilter.h/.cpp  # State variable filter
│   ├── dfl_RPMOscillator.h             # Recursive phase modulation oscillator
│   ├── dfl_FilterBase.h                # Filter base class
│   └── ...
├── README.md             # User documentation
└── AGENTS.md             # This file
```

## Building the Project

### Prerequisites

**Desktop Build:**
- CMake 3.17+
- C++17 compiler (Clang, GCC, or MSVC)
- Platform-specific dependencies are fetched automatically by CMake

**Web Build (Emscripten):**
- Emscripten SDK installed and activated
- CMake 3.17+

### Desktop Build

From the visage root directory:

```bash
# Create build directory (first time only)
mkdir -p build
cd build

# Configure and build
cmake ..
cmake --build . --target ExampleFaveworm -j8
```

**Output locations:**
- **macOS**: `build/examples/ExampleFaveworm.app/Contents/MacOS/ExampleFaveworm`
- **Linux**: `build/examples/ExampleFaveworm`
- **Windows**: `build/examples/ExampleFaveworm.exe`

### Web Build (Emscripten)

From the visage root directory:

```bash
# Create wasm build directory (first time only)
mkdir -p build_wasm
cd build_wasm

# Configure with Emscripten toolchain
emcmake cmake ..

# Build
cmake --build . --target ExampleFaveworm -j8
```

**Output location:**
- `build_wasm/examples/builds/Faveworm/index.html`
- `build_wasm/examples/builds/Faveworm/index.js`
- `build_wasm/examples/builds/Faveworm/index.wasm`

**Running locally:**
```bash
# Serve with Python (from build_wasm/examples/builds/Faveworm/)
cd build_wasm/examples/builds/Faveworm
python3 -m http.server 8000

# Then open: http://localhost:8000/index.html
```

### Quick Rebuild

After making changes, rebuild with:

```bash
# Desktop
cd build
cmake --build . --target ExampleFaveworm -j8

# Web
cd build_wasm
cmake --build . --target ExampleFaveworm -j8
```

## Development Guidelines

### Code Style

- Follow existing code style (clang-format configuration in repo root)
- Use C++17 features where appropriate
- Prefer `const` and `constexpr` where possible
- Use `std::clamp` for range limiting

### Platform Differences

The code uses preprocessor macros to handle platform differences:

```cpp
#if VISAGE_EMSCRIPTEN
  // Web-specific code (SDL audio, different defaults)
#else
  // Desktop code (PortAudio, native windowing)
#endif
```

**Key differences:**
- **Audio**: Web uses SDL2, desktop uses PortAudio
- **Default volume**: Web starts muted (0.0), desktop at 0.5
- **File loading**: Web doesn't support drag-and-drop in all browsers

### UI Widget System

All UI widgets are defined in `FilterJoystick.h`:
- `FilterKnob` - Rotary knob with multiple scaling modes (linear, logarithmic, bipolar, reverse logarithmic)
- `FilterSlider` - Vertical slider
- `FilterJoystick` - 2D joystick for filter morphing
- `PushButtonSwitch` - Toggle button with LED indicator
- `ModeSelector` - Cycling selector
- `NumericDisplay` - Digital readout

**Knob Scaling Modes:**
- `logarithmic` - For frequency controls (exponential scaling)
- `bipolar_logarithmic` - For symmetric controls around zero
- `reverse_logarithmic` - For controls where high values need more precision (e.g., resonance)
- `bidirectional` - Visual indicator fills from center

### DSP Components

**SimpleSVF** (in `FilterMorpher.h`):
- State variable filter with tanh saturation
- Outputs: LP, BP, HP, BR (notch), AP (allpass)
- Safe for resonance up to 1.0 (self-oscillation) due to tanh clipping

**FilterMorpher**:
- 360° morphing between filter modes
- Supports stereo split mode (different outputs to L/R)
- Joystick control for intuitive filter exploration

**RPMOscillator** (in `dsp/dfl_RPMOscillator.h`):
- Recursive phase modulation oscillator
- Used for test signal generation in XY mode
- Produces complex, evolving waveforms

### Git Workflow

**Atomic Commits:**
Commits should be atomic - each commit should represent a single logical change. Use `git add -p` (patch mode) to stage only relevant changes:

```bash
# Review and stage changes interactively
git add -p

# Or stage specific files
git add FilterJoystick.h faveworm.cpp

# Commit with descriptive message
git commit -m "Add reverse logarithmic scaling to resonance knob

- Extend resonance range to 1.0 for self-oscillation
- Implement x² curve for more precision at high values
- Safe due to tanh clipping in SimpleSVF"
```

**Good commit practices:**
- One logical change per commit
- Clear, descriptive commit messages
- Separate refactoring from feature additions
- Test builds before committing

## Common Development Tasks

### Adding a New UI Control

1. Define widget in `FilterJoystick.h` (or use existing widget class)
2. Add member variable to `ExampleEditor` class in `faveworm.cpp`
3. Initialize in `ExampleEditor` constructor
4. Add to control panel with `control_panel_.addScrolledChild(&widget_)`
5. Set callback to update audio/visual state
6. Position in `resized()` method
7. Update visibility in `updatePanelVisibility()` if mode-specific

### Modifying Filter Behavior

1. Update `SimpleSVF` in `FilterMorpher.h` for core filter changes
2. Update `FilterMorpher` for morphing/routing changes
3. Update `setFilterCutoff()`/`setFilterResonance()` in `AudioPlayer` class
4. Test with various resonance/cutoff settings
5. Verify stability at extreme settings (especially high resonance)

### Adding Keyboard Shortcuts

1. Add case to `handleKey()` method in `ExampleEditor`
2. Update help overlay in `HelpOverlay::draw()`
3. Document in README.md

## Testing

### Manual Testing Checklist

- [ ] Desktop build compiles without warnings
- [ ] Web build compiles without warnings
- [ ] UI controls respond correctly
- [ ] Audio playback works (desktop and web)
- [ ] Filter controls affect sound/visuals as expected
- [ ] No crashes with extreme parameter values
- [ ] Drag-and-drop file loading works (desktop)
- [ ] All display modes work correctly
- [ ] Keyboard shortcuts function properly

### Performance Considerations

- Oscilloscope rendering is GPU-accelerated via Visage
- Audio processing runs in separate thread
- Ring buffer for lock-free audio→visual data transfer
- Phosphor trails use frame history (4 frames)
- Beam rendering uses adaptive subdivision based on movement speed

## Troubleshooting

### Build Issues

**"No rule to make target Faveworm"**
- Use `ExampleFaveworm` (not `Faveworm`)

**Emscripten build fails**
- Ensure emscripten is activated: `source /path/to/emsdk/emsdk_env.sh`
- Use `emcmake cmake ..` for configuration

**Missing dependencies**
- CMake fetches dependencies automatically
- Ensure internet connection for first build

### Runtime Issues

**No audio output (web)**
- Click anywhere in the window first (browser autoplay policy)
- Check browser console for errors

**Choppy visuals**
- Check CPU usage (audio thread priority)
- Reduce phosphor trail length if needed

**Filter instability**
- Verify tanh saturation is active in SimpleSVF
- Check resonance clamping (should be 0.0-1.0)

## Resources

- [Visage Framework](https://visage.dev/)
- [Faveworm Original](https://music.metaservices.microsoft.com/faveworm/) by Laurent de Soras
- [SVF Filter Theory](https://cytomic.com/files/dsp/SvfLinearTrapOptimised2.pdf)
- User documentation: `README.md`

## Contact

For questions or issues, refer to the main Visage repository or project maintainer.
