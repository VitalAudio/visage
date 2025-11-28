# Visage: UI library meets creative coding

**Visage** is a GPU-accelerated, cross-platform C++ library for native UI and 2D graphics. It merges the structure of a UI framework with the features of a creative graphics libraries.

## Full Basic Example
```cpp
#include <visage/app.h>

int main() {
  visage::ApplicationWindow app;

  app.onDraw() = [&app](visage::Canvas& canvas) {
    canvas.setColor(0xffff00ff);
    canvas.fill(0, 0, app.width(), app.height());
  };

  app.show(800, 600); // Opens as 800 x 600 pixel window
  app.runEventLoop(); // Runs window events. Returns when window is closed.
  return 0;
}
```

## Demos
- [Showcase](https://visage.dev/examples/Showcase/)
- [Blend Modes](https://visage.dev/examples/BlendModes/)
- [Bloom](https://visage.dev/examples/Bloom/)
- [Gradients](https://visage.dev/examples/Gradients/)
- [Paths](https://visage.dev/examples/Paths/)
- [Post Shader Effects](https://visage.dev/examples/PostEffects/)
- [Live Shader Editing](https://visage.dev/examples/LiveShaderEditing/)

## UI Features

- **Event Normalization**  
&nbsp;&nbsp;&nbsp;Cross-platform support for keyboard and mouse input normalization.

- **Window Normalization**  
&nbsp;&nbsp;&nbsp;Cross-platform support for opening and handling windows.

- **Text entry**  
&nbsp;&nbsp;&nbsp;Unicode text entry with multi line text editing

- **✨ Emojis ✨**  
&nbsp;&nbsp;&nbsp;If you're into that kind of thing 🤷

- **Partial Rendering**  
&nbsp;&nbsp;&nbsp;Redraws only the dirty regions for optimal performance.

## Graphics Features

- **Fluid motion**  
&nbsp;&nbsp;&nbsp;New frames are displayed at the monitor's refresh rate and animations are smooth

- **Automatic Shape Batching**  
&nbsp;&nbsp;&nbsp;Automatically groups shapes for efficient GPU rendering.

- **Blend Modes**  
&nbsp;&nbsp;&nbsp;Supports blending layers with additive, subtractive or by drawing a custom mask for the UI to pass through

- **Shaders**  
&nbsp;&nbsp;&nbsp;Write shaders once and transpile them for Direct3d, Metal and OpenGL

- **Included Effects**  
&nbsp;&nbsp;&nbsp;Real-time effects such as large blur and bloom

- **Pixel Accuracy**  
&nbsp;&nbsp;&nbsp;Access to device pixel size ensures precise rendering without blurring.

## Supported Platforms
- **Windows**: Direct3D11
- **MacOS**: Metal  
- **Linux**: Vulkan
- **Web/Emscripten**: WebGL
