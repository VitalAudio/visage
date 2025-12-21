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
| `Left/Right` | Adjust filter cutoff frequency |

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
├── faveworm.cpp        # Main oscilloscope implementation
├── FilterMorpher.h     # 360° filter morphing + SimpleSVF
├── FilterJoystick.h    # Visual joystick UI control
└── dsp/                # Full SVF from superfreak (reference)
    ├── dfl_StateVariableFilter.h/.cpp
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
