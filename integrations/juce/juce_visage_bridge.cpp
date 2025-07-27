/* Copyright Vital Audio, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "juce_visage_bridge.h"

JuceVisageBridge::JuceVisageBridge() {
    // Set up the OpenGL context with this component as the renderer
    openGLContext.setRenderer(this);
    openGLContext.attachTo(*this);
    
    // Enable continuous repainting to ensure stable 60fps rendering
    // This eliminates flickering and provides smooth updates
    openGLContext.setContinuousRepainting(true);
    
    // Set as opaque to prevent underlying component painting issues
    setOpaque(true);
}

JuceVisageBridge::~JuceVisageBridge() {
    // Clean shutdown: stop timer and detach OpenGL context
    stopTimer();
    shutdownVisageWindow();
    openGLContext.detach();
}

void JuceVisageBridge::setRootFrame(visage::Frame* frame) {
    rootFrame = frame;
    
    // Update frame bounds to match current component size
    if (rootFrame && getWidth() > 0 && getHeight() > 0) {
        rootFrame->setBounds(0, 0, getWidth(), getHeight());
    }
    
    // Trigger a repaint to show the new frame
    repaint();
}

void JuceVisageBridge::paint(juce::Graphics& g) {
    // Fallback rendering before OpenGL is initialized
    // This prevents the initial magenta flash by providing a proper background
    if (!isInitialized) {
        g.fillAll(juce::Colour(0xff282828));  // Dark gray background to prevent magenta flash
        
        if (rootFrame) {
            // Show a subtle loading state without text to avoid visual artifacts
            g.setColour(juce::Colour(0xff3a3a3a));
            g.fillRect(getLocalBounds().reduced(2));
        }
    }
    // When OpenGL is initialized, this paint method won't be called as OpenGL takes over
}

void JuceVisageBridge::resized() {
    // Update the root frame bounds to match the new component size
    if (rootFrame) {
        rootFrame->setBounds(0, 0, getWidth(), getHeight());
    }
    
    // Handle window creation/destruction asynchronously for thread safety
    if (getWidth() > 0 && getHeight() > 0) {
        juce::MessageManager::callAsync([this] {
            createVisageWindowAsync();
        });
    }
}

void JuceVisageBridge::newOpenGLContextCreated() {
    // Initialize Visage graphics system with the JUCE OpenGL context
    // This ensures proper integration and prevents rendering conflicts
    
    // Get display handle for X11 (Linux) - on other platforms this will be null
    void* display = nullptr;
#if JUCE_LINUX
    if (auto* peer = getPeer()) {
        display = peer->getNativeHandle();
    }
#endif
    
    // Initialize Visage renderer with our OpenGL context
    // Note: The exact initialization may vary depending on Visage's internal API
    void* windowHandle = nullptr;
    if (auto* peer = getPeer()) {
        windowHandle = peer->getNativeHandle();
    }
    
    visage::Renderer::instance().checkInitialization(windowHandle, display);
    
    isInitialized = true;
    
    // Trigger initial layout update
    resized();
}

void JuceVisageBridge::createVisageWindowAsync() {
    // Ensure we're on the message thread for thread safety
    if (!juce::MessageManager::getInstance()->isThisTheMessageThread()) {
        juce::MessageManager::callAsync([this] { createVisageWindowAsync(); });
        return;
    }
    
    // Ensure we have valid context and size
    if (!contextReady || getWidth() <= 0 || getHeight() <= 0) {
        return;
    }
    
    // Clean up any existing setup
    shutdownVisageWindow();
    
    // Set up the canvas for rendering using peer's native handle
    if (auto* peer = getPeer()) {
        canvas.pairToWindow(peer->getNativeHandle(), getWidth(), getHeight());
    }
    
    isInitialized = true;
    
    // Start timer for 60fps rendering
    startTimerHz(60);
    
    // Trigger initial layout update
    if (rootFrame) {
        rootFrame->setBounds(0, 0, getWidth(), getHeight());
    }
}

void JuceVisageBridge::shutdownVisageWindow() {
    stopTimer();
    isInitialized = false;
}

void JuceVisageBridge::renderOpenGL() {
    if (!isInitialized || !rootFrame) {
        return;
    }
    
    // Clear the canvas with our background color
    canvas.setColor(0xff282828);  // Dark gray background
    canvas.fill(0, 0, getWidth(), getHeight());
    
    // Render the Visage frame hierarchy
    rootFrame->draw(canvas);
    
    // Submit the frame for rendering
    canvas.submit();
}

void JuceVisageBridge::timerCallback() {
    // Additional rendering update if needed
    // The OpenGL context should handle most rendering via renderOpenGL()
    if (isInitialized) {
        repaint();
    }
}

void JuceVisageBridge::openGLContextClosing() {
    // Clean shutdown of Visage graphics
    shutdownVisageWindow();
    contextReady = false;
}

void JuceVisageBridge::refreshContext() {
    // Force a refresh of the OpenGL context - useful for plugin hosts
    // that might change window states unexpectedly
    if (isInitialized && getWidth() > 0 && getHeight() > 0) {
        // Re-pair the canvas to ensure proper viewport using peer's native handle
        if (auto* peer = getPeer()) {
            canvas.pairToWindow(peer->getNativeHandle(), getWidth(), getHeight());
        }
        
        // Update frame bounds
        if (rootFrame) {
            rootFrame->setBounds(0, 0, getWidth(), getHeight());
        }
        
        // Trigger a repaint
        repaint();
    }
}

void JuceVisageBridge::mouseDown(const juce::MouseEvent& event) {
    // Handle dragging for frameless windows
    if (draggableTitleHeight > 0 && event.y < draggableTitleHeight) {
        // Start dragging the top-level component (window)
        auto* topLevel = getTopLevelComponent();
        if (topLevel) {
            componentDragger.startDraggingComponent(topLevel, event);
        }
        return;
    }
    
    // Forward mouse events to the Visage frame for UI interaction
    if (rootFrame) {
        visage::MouseEvent visageEvent;
        visageEvent.position = { static_cast<float>(event.x), static_cast<float>(event.y) };
        visageEvent.relative_position = visageEvent.position;
        visageEvent.window_position = visageEvent.position;
        visageEvent.is_down = true;
        visageEvent.frame = rootFrame;
        
        // Convert JUCE mouse button to Visage equivalent
        if (event.mods.isLeftButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonLeft;
            visageEvent.button_state = visage::kMouseButtonLeft;
        } else if (event.mods.isRightButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonRight;
            visageEvent.button_state = visage::kMouseButtonRight;
        } else if (event.mods.isMiddleButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonMiddle;
            visageEvent.button_state = visage::kMouseButtonMiddle;
        }
        
        rootFrame->processMouseDown(visageEvent);
    }
}

void JuceVisageBridge::mouseDrag(const juce::MouseEvent& event) {
    // Handle window dragging for frameless windows
    if (draggableTitleHeight > 0 && event.y < draggableTitleHeight) {
        auto* topLevel = getTopLevelComponent();
        if (topLevel) {
            componentDragger.dragComponent(topLevel, event, nullptr);
        }
        return;
    }
    
    // Forward drag events to Visage frame
    if (rootFrame) {
        visage::MouseEvent visageEvent;
        visageEvent.position = { static_cast<float>(event.x), static_cast<float>(event.y) };
        visageEvent.relative_position = visageEvent.position;
        visageEvent.window_position = visageEvent.position;
        visageEvent.is_down = true;
        visageEvent.frame = rootFrame;
        
        if (event.mods.isLeftButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonLeft;
            visageEvent.button_state = visage::kMouseButtonLeft;
        } else if (event.mods.isRightButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonRight;
            visageEvent.button_state = visage::kMouseButtonRight;
        } else if (event.mods.isMiddleButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonMiddle;
            visageEvent.button_state = visage::kMouseButtonMiddle;
        }
        
        rootFrame->processMouseDrag(visageEvent);
    }
}

void JuceVisageBridge::mouseUp(const juce::MouseEvent& event) {
    // Forward mouse up events to Visage frame
    if (rootFrame) {
        visage::MouseEvent visageEvent;
        visageEvent.position = { static_cast<float>(event.x), static_cast<float>(event.y) };
        visageEvent.relative_position = visageEvent.position;
        visageEvent.window_position = visageEvent.position;
        visageEvent.is_down = false;
        visageEvent.frame = rootFrame;
        
        if (event.mods.isLeftButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonLeft;
        } else if (event.mods.isRightButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonRight;
        } else if (event.mods.isMiddleButtonDown()) {
            visageEvent.button_id = visage::kMouseButtonMiddle;
        }
        
        rootFrame->processMouseUp(visageEvent);
    }
}

void JuceVisageBridge::mouseMove(const juce::MouseEvent& event) {
    // Forward mouse move events to Visage frame
    if (rootFrame) {
        visage::MouseEvent visageEvent;
        visageEvent.position = { static_cast<float>(event.x), static_cast<float>(event.y) };
        visageEvent.relative_position = visageEvent.position;
        visageEvent.window_position = visageEvent.position;
        visageEvent.is_down = false;
        visageEvent.frame = rootFrame;
        
        rootFrame->processMouseMove(visageEvent);
    }
}
