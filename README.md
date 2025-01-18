# Visage: UI library meets creative coding

**Visage** is a GPU-accelerated, cross-platform C++ library for native UI and 2D graphics. It merges the structure of a UI framework with the features of a creative graphics libraries.

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

## Features

- **Event Normalization**  
&nbsp;&nbsp;&nbsp;Cross-platform support for keyboard and mouse input normalization.

- **Window Normalization**  
&nbsp;&nbsp;&nbsp;Cross-platform support for opening and handling windows.

- **Automatic Shape Batching**  
&nbsp;&nbsp;&nbsp;Automatically groups shapes for efficient GPU rendering.

- **Partial Rendering**  
&nbsp;&nbsp;&nbsp;Redraws only the dirty regions for optimal performance.

- **Blend Modes**  
&nbsp;&nbsp;&nbsp;Supports blending layers with additive, subtractive or by drawing a custom stencil

- **Shaders**  
&nbsp;&nbsp;&nbsp;Write shaders once and transpile them for Direct3d, Metal and OpenGL

- **Included Effects**  
&nbsp;&nbsp;&nbsp;Real-time effects such as large blur and bloom

- **Pixel Accuracy**  
&nbsp;&nbsp;&nbsp;Access to device pixel size ensures precise rendering without blurring.

## Supported Platforms
- **Windows**: Direct3D11
- **MacOS**: Metal  
- **Linux**: OpenGL
- **Web**: WebGL
