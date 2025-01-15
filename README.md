# Visage: UI library meets creative coding

**Visage** is a GPU-accelerated, cross-platform C++ library for native UI and 2D graphics. It merges the structure of a UI framework with the features of a creative graphics libraries, supporting custom shaders for advanced effects.

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
- [Blend Modes](https://visage.dev/examples/BlendModes/)
- [Blur and Bloom](https://visage.dev/examples/BlurAndBloom/)
- [Post Shader Effects](https://visage.dev/examples/PostEffects/)
- [Live Shader Editing](https://visage.dev/examples/LiveShaderEditing/)
- [Layout Flexing](https://visage.dev/examples/Layout/)

## Supported Platforms
- **Windows**: Direct3D11
- **MacOS**: Metal  
- **Linux**: OpenGL
- **Web**: WebGL  
