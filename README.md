# Visage: UI library meets creative coding

**Visage** is a GPU-accelerated, cross-platform C++ library for native UI and 2D graphics. It merges the structure of a UI framework with the features of a creative graphics libraries.

## Features
- Event normalization : cross platform normalization of keyboard and mouse input events
- Automatic shape batching : automatically batches shapes together for the GPU for faster rendering
- Partial rendering : only redraws dirty areas
- Blend modes : blend layers with additive, subtractive or by drawing a custom stencil
- Shaders : write shaders once which are transpiled to Direct3d, Metal, and OpenGL
- Included effects : real-time large blur and bloom
- Pixel accurate : access to device pixel size so you can draw without having 1 pixel blurs

## Full Basic Example
```cpp
#include <visage_app/application_editor.h>

int main() {
  visage::WindowedEditor editor;

  editor.onDraw() = [&editor](visage::Canvas& canvas) {
    canvas.setColor(0xffff00ff);
    canvas.fill(0, 0, editor.width(), editor.height());
  };

  editor.show(800, 600); // Opens as 800 x 600 pixel window
  editor.runEventLoop(); // Runs window events. Returns when window is closed.
  return 0;
}
```

## Demos
- [Showcase](https://visage.dev/examples/Showcase/)
- [Blend Modes](https://visage.dev/examples/BlendModes/)
- [Bloom](https://visage.dev/examples/Bloom/)
- [Post Shader Effects](https://visage.dev/examples/PostEffects/)
- [Live Shader Editing](https://visage.dev/examples/LiveShaderEditing/)
- [Layout Flexing](https://visage.dev/examples/Layout/)

## Supported Platforms
- **Windows**: Direct3D11
- **MacOS**: Metal  
- **Linux**: OpenGL
- **Web**: WebGL
