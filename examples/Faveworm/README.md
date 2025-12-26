# Faveworm Oscilloscope

Analog oscilloscope waveform visualization inspired by [faveworm](https://music.metaservices.microsoft.com/faveworm/) by Laurent de Soras. Features physics-based beam rendering where brightness is inversely proportional to movement speed, phosphor persistence, and a 360° SVF filter for XY mode visualization.

## Display Modes

| Mode | Description |
|------|-------------|
| **TimeFree** | Horizontal sweep, free running |
| **TimeTrigger** | Horizontal sweep with trigger and waveform locking |
| **XY** | Lissajous mode (left=X, right=Y) with SVF filter |

## Keyboard Controls

### General
| Key | Action |
|-----|--------|
| `M` | Cycle display mode (TimeFree → TimeTrigger → XY) |
| `Space` | Play/pause audio |
| `B` | Toggle bloom effect |
| `P` | Toggle phosphor persistence |
| `Up/Down` | Adjust bloom intensity |

### Trigger Mode
| Key | Action |
|-----|--------|
| `L` | Toggle waveform locking (autocorrelation) |
| `E` | Toggle trigger edge (rising/falling) |
| `Shift+Up/Down` | Adjust trigger threshold |

### Filter (XY Mode)
| Key | Action |
|-----|--------|
| `F` | Toggle filter on/off |
| `S` | Toggle stereo split mode (mono → X/Y via different filter outputs) |
| `D` | Cycle split mode presets |
| `Left/Right` | Adjust filter cutoff frequency |

### Split Mode Presets

In stereo split mode, a mono source is routed through an SVF, with different filter outputs sent to X and Y axes. This creates phase relationships that produce rotating, evolving Lissajous patterns.

| Preset | X | Y | Best Settings | Visual Style |
|--------|---|---|---------------|--------------|
| LP/HP Retro | LP | HP | fc 40-80, Q 0.7 | Classic ellipses, analog feel |
| BP/AP Flowers | BP | AP | fc 150-250, Q 1.0 | Swirling geometric shapes |
| BR/AP Kaleidoscope | BR | AP | fc 300-800, Q 1.2 | Psychedelic mandalas |
| LP/BP Organic | LP | BP | fc 100-200, Q 0.8 | Soft loops and petals |
| AP/HP Liquid | AP | HP | fc 20-40, Q 0.5 | Liquid vector graphics |
| BP/BR Complement | BP | BR | fc 200-500, Q 0.9 | Complementary bands |
| In/Morph Live | Input | Filtered | varies | X stable, Y evolves (use joystick) |

### Test Signal Generator (XY Mode)
| Key | Action |
|-----|--------|
| `T` | Toggle test signal on/off |
| `W` | Cycle waveform (Sine → Triangle → Saw → Square → Noise) |
| `[` / `]` | Decrease/increase base frequency |
| `Shift+[` / `Shift+]` | Decrease/increase Y detune ratio |

The test signal generator uses two detuned RPM oscillators to create rotating Lissajous patterns without audio input. The slight detune causes the pattern to slowly rotate.

## Filter Joystick (XY Mode)

The circular joystick in the bottom-right corner controls the SVF filter morphing:

```
            BP (bandpass)
               ↑
               |
    LP ←———— AP ————→ HP
  (lowpass)  (center)  (highpass)
               |
               ↓
            BR (notch)
```

- **Angle**: Selects filter mode with smooth crossfades
- **Radius**: Controls filter amount (center = allpass, edge = full filter)
- **Mouse wheel**: Adjust cutoff frequency
- **Shift + wheel**: Adjust resonance

## Loading Audio

Drag and drop a WAV file onto the window to load and play it.

Supported formats:
- 16-bit PCM
- 24-bit PCM
- 32-bit PCM
- 32-bit float

## Recommended Settings for XY Visuals

| Style | Cutoff | Resonance | Mode | Description |
|-------|--------|-----------|------|-------------|
| Classic retro | 40-80 Hz | 0.7 | LP ↔ HP | Smooth, stable Lissajous |
| Geometric flowers | 150-250 Hz | 1.0 | BP | Rotating, ornate patterns |
| Psychedelic | 300-800 Hz | 1.2-1.8 | BR ↔ BP | Tight spirals, starburst |
| Liquid vector | 20-40 Hz | 0.5-0.7 | AP + slight HP | Soft, organic shapes |

## File Structure

```
Faveworm/
├── faveworm.cpp          # Main oscilloscope implementation
├── FilterMorpher.h       # 360° filter morphing + SimpleSVF
├── FilterJoystick.h      # Visual joystick UI control
├── TestSignalGenerator.h # RPM oscillator test signal for XY mode
└── dsp/                  # DSP components from superfreak
    ├── dfl_StateVariableFilter.h/.cpp
    ├── dfl_RPMOscillator.h
    ├── dfl_FilterBase.h
    ├── dfl_FastMath.h
    └── ...
```

## Building

```bash
cd /path/to/visage/build
cmake --build . --target ExampleFaveworm
```

On macOS, the app bundle will be at:
```
build/examples/ExampleFaveworm.app
```
