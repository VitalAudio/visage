# JUCE Integration for Visage

This integration provides seamless embedding of Visage UI components within JUCE applications, solving common rendering issues encountered when using Visage in secondary windows.

## Problems Solved

The `JuceVisageBridge` addresses several critical issues:

- **Magenta flashes and transparency artifacts** - Proper OpenGL context initialization and clear color management
- **Flickering and dark bars** - Continuous repainting and correct viewport sizing
- **Non-draggable frameless windows** - Optional draggable title bar support
- **BGFX rendering conflicts** - Proper initialization order and context sharing

## Requirements

- JUCE Framework (v7.0+)
- Visage (this library)
- OpenGL support
- CMake 3.17+

## Building

Enable the JUCE integration when configuring Visage:

```bash
cmake -B build -DVISAGE_BUILD_JUCE_INTEGRATION=ON -DJUCE_DIR=/path/to/JUCE
cmake --build build
```

## Usage

### Basic Integration

```cpp
#include <visage/integrations/juce/juce_visage_bridge.h>

class MyAudioProcessorEditor : public juce::AudioProcessorEditor 
{
public:
    MyAudioProcessorEditor() 
    {
        // Create the bridge
        visageBridge = std::make_unique<JuceVisageBridge>();
        addAndMakeVisible(visageBridge.get());
        
        // Create your Visage UI
        myVisageFrame = std::make_unique<MyVisageFrame>();
        visageBridge->setRootFrame(myVisageFrame.get());
        
        setSize(800, 600);
    }
    
    void resized() override 
    {
        visageBridge->setBounds(getLocalBounds());
    }

private:
    std::unique_ptr<JuceVisageBridge> visageBridge;
    std::unique_ptr<MyVisageFrame> myVisageFrame;
};
```

### Secondary Window with Draggable Support

```cpp
class SecondaryWindow : public juce::DocumentWindow
{
public:
    SecondaryWindow() : juce::DocumentWindow("Visage Window", 
                                           juce::Colours::darkgrey, 
                                           juce::DocumentWindow::allButtons)
    {
        auto bridge = std::make_unique<JuceVisageBridge>();
        
        // Enable dragging for a 30-pixel title area (useful for frameless windows)
        bridge->setDraggableTitleHeight(30);
        
        auto visageFrame = std::make_unique<MyVisageFrame>();
        bridge->setRootFrame(visageFrame.get());
        
        setContentNonOwned(bridge.get(), true);
        setResizable(true, true);
        setVisible(true);
        
        // Store references
        this->bridge = std::move(bridge);
        this->frame = std::move(visageFrame);
    }

private:
    std::unique_ptr<JuceVisageBridge> bridge;
    std::unique_ptr<MyVisageFrame> frame;
};
```

### Frameless Popup Window

```cpp
class FramelessPopup : public juce::Component
{
public:
    FramelessPopup()
    {
        bridge = std::make_unique<JuceVisageBridge>();
        
        // Enable dragging for the entire top 40 pixels
        bridge->setDraggableTitleHeight(40);
        
        addAndMakeVisible(bridge.get());
        
        // Create a Visage frame with a custom title bar
        frame = std::make_unique<MyFramelessFrame>();
        bridge->setRootFrame(frame.get());
        
        setSize(400, 300);
        addToDesktop(juce::ComponentPeer::windowIsTemporary);
    }
    
    void resized() override
    {
        bridge->setBounds(getLocalBounds());
    }

private:
    std::unique_ptr<JuceVisageBridge> bridge;
    std::unique_ptr<MyFramelessFrame> frame;
};
```

## API Reference

### JuceVisageBridge

The main integration class that handles OpenGL rendering and event forwarding.

#### Methods

- `setRootFrame(visage::Frame* frame)` - Set the root Visage frame to render
- `getRootFrame()` - Get the current root frame
- `setDraggableTitleHeight(int height)` - Enable window dragging for frameless windows
- `getDraggableTitleHeight()` - Get current draggable area height

#### Features

- **Automatic OpenGL Context Management** - Handles initialization and cleanup
- **Event Forwarding** - Mouse events are properly converted and forwarded to Visage
- **Continuous Rendering** - 60fps rendering without manual timer management
- **Background Color Control** - Prevents magenta flashes with proper clear colors
- **Window Dragging** - Optional support for draggable title areas

## Troubleshooting

### Build Issues

1. **JUCE not found**: Ensure `JUCE_DIR` points to your JUCE installation
2. **OpenGL linking errors**: Install OpenGL development packages
3. **Missing Visage headers**: Build Visage first with amalgamated headers enabled

### Runtime Issues

1. **Black screen**: Ensure the root frame is set after the bridge is visible
2. **Magenta flashes**: This should be automatically resolved by the bridge
3. **Performance issues**: Check that continuous repainting is enabled (default)

### Platform-Specific Notes

- **macOS**: Requires Metal and OpenGL frameworks
- **Windows**: Needs OpenGL32 and DirectX libraries  
- **Linux**: Requires X11, OpenGL, and ALSA development packages

## Contributing

This integration is part of the main Visage repository. Please refer to the main project's contribution guidelines.

## License

Same as Visage (MIT License) - see LICENSE file in the root directory.
