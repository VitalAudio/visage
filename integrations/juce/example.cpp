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

/**
 * Example showing how to use JuceVisageBridge to solve rendering issues
 * in secondary JUCE windows (fixes issue #47)
 */

#include "juce_visage_bridge.h"
#include <visage/ui.h>

// Example Visage frame for demonstration
class ExampleVisageFrame : public visage::Frame {
public:
    ExampleVisageFrame() : visage::Frame("ExampleFrame") {
        setBounds(0, 0, 800, 600);
    }
    
    void draw(visage::Canvas& canvas) override {
        // Draw a simple example interface
        canvas.setColor(0xff2a2a2a);  // Dark background
        canvas.fill(0, 0, width(), height());
        
        // Draw a header bar if this is a frameless window
        canvas.setColor(0xff3a3a3a);
        canvas.fill(0, 0, width(), 30);
        
        // Example UI elements
        canvas.setColor(0xff5555ff);
        canvas.fill(50, 50, 200, 100);
        
        canvas.setColor(0xffff5555);
        canvas.fill(300, 50, 200, 100);
        
        canvas.setColor(0xff55ff55);
        canvas.fill(50, 200, 450, 100);
    }
    
    void mouseDown(const visage::MouseEvent& e) override {
        // Handle mouse interactions
        if (e.isLeftButton()) {
            // Example: change color or state based on click position
        }
    }
};

//==============================================================================
// Example 1: Secondary DocumentWindow with Visage content
class SecondaryVisageWindow : public juce::DocumentWindow {
public:
    SecondaryVisageWindow() 
        : juce::DocumentWindow("Visage Secondary Window", 
                               juce::Colours::darkgrey, 
                               juce::DocumentWindow::allButtons) 
    {
        // Create the bridge - this solves the rendering issues
        bridge = std::make_unique<JuceVisageBridge>();
        
        // Create our Visage content
        visageFrame = std::make_unique<ExampleVisageFrame>();
        
        // Connect them together
        bridge->setRootFrame(visageFrame.get());
        
        // Set the bridge as window content
        setContentNonOwned(bridge.get(), true);
        
        // Configure window
        setResizable(true, true);
        setSize(800, 600);
        setVisible(true);
        
        // Center on screen
        centreWithSize(getWidth(), getHeight());
    }
    
    void closeButtonPressed() override {
        setVisible(false);
    }

private:
    std::unique_ptr<JuceVisageBridge> bridge;
    std::unique_ptr<ExampleVisageFrame> visageFrame;
};

//==============================================================================
// Example 2: Frameless popup with draggable support
class FramelessVisagePopup : public juce::Component {
public:
    FramelessVisagePopup() {
        // Create the bridge
        bridge = std::make_unique<JuceVisageBridge>();
        
        // Enable dragging for the top 30 pixels (title bar area)
        bridge->setDraggableTitleHeight(30);
        
        // Create Visage content
        visageFrame = std::make_unique<ExampleVisageFrame>();
        bridge->setRootFrame(visageFrame.get());
        
        // Add to this component
        addAndMakeVisible(bridge.get());
        
        // Set size and show as desktop component
        setSize(400, 300);
        addToDesktop(juce::ComponentPeer::windowIsTemporary | 
                    juce::ComponentPeer::windowHasDropShadow);
        
        // Position near mouse cursor
        auto mousePos = juce::Desktop::getMousePosition();
        setTopLeftPosition(mousePos.x - 200, mousePos.y - 150);
        
        setVisible(true);
    }
    
    void resized() override {
        bridge->setBounds(getLocalBounds());
    }
    
    // Handle escape key to close
    bool keyPressed(const juce::KeyPress& key) override {
        if (key.isKeyCode(juce::KeyPress::escapeKey)) {
            setVisible(false);
            return true;
        }
        return false;
    }

private:
    std::unique_ptr<JuceVisageBridge> bridge;
    std::unique_ptr<ExampleVisageFrame> visageFrame;
};

//==============================================================================
// Example 3: Plugin editor with embedded Visage UI
class VisageAudioProcessorEditor : public juce::AudioProcessorEditor {
public:
    VisageAudioProcessorEditor(juce::AudioProcessor& processor)
        : juce::AudioProcessorEditor(processor) 
    {
        // Create the bridge
        bridge = std::make_unique<JuceVisageBridge>();
        addAndMakeVisible(bridge.get());
        
        // Create Visage UI
        visageFrame = std::make_unique<ExampleVisageFrame>();
        bridge->setRootFrame(visageFrame.get());
        
        // Set editor size
        setSize(800, 600);
    }
    
    void resized() override {
        bridge->setBounds(getLocalBounds());
    }

private:
    std::unique_ptr<JuceVisageBridge> bridge;
    std::unique_ptr<ExampleVisageFrame> visageFrame;
};

//==============================================================================
// Usage example in main application
class ExampleApplication : public juce::JUCEApplication {
public:
    const juce::String getApplicationName() override { return "Visage JUCE Integration Example"; }
    const juce::String getApplicationVersion() override { return "1.0.0"; }
    
    void initialise(const juce::String&) override {
        // Create main window with button to test secondary windows
        mainWindow = std::make_unique<MainWindow>();
    }
    
    void shutdown() override {
        mainWindow.reset();
    }

private:
    class MainWindow : public juce::DocumentWindow {
    public:
        MainWindow() : juce::DocumentWindow("Main Window", juce::Colours::lightgrey, allButtons) {
            auto content = std::make_unique<juce::Component>();
            
            // Button to test secondary window
            auto secondaryButton = std::make_unique<juce::TextButton>("Open Secondary Visage Window");
            secondaryButton->onClick = [this] {
                // This demonstrates the fix for issue #47
                secondaryWindow = std::make_unique<SecondaryVisageWindow>();
            };
            content->addAndMakeVisible(secondaryButton.get());
            secondaryButton->setBounds(20, 20, 250, 30);
            
            // Button to test frameless popup
            auto popupButton = std::make_unique<juce::TextButton>("Open Frameless Popup");
            popupButton->onClick = [this] {
                popup = std::make_unique<FramelessVisagePopup>();
            };
            content->addAndMakeVisible(popupButton.get());
            popupButton->setBounds(20, 60, 250, 30);
            
            // Store button references
            this->secondaryButton = std::move(secondaryButton);
            this->popupButton = std::move(popupButton);
            
            setContentOwned(content.release(), true);
            setSize(300, 200);
            setVisible(true);
        }
        
        void closeButtonPressed() override {
            juce::JUCEApplication::getInstance()->systemRequestedQuit();
        }

    private:
        std::unique_ptr<juce::TextButton> secondaryButton;
        std::unique_ptr<juce::TextButton> popupButton;
        std::unique_ptr<SecondaryVisageWindow> secondaryWindow;
        std::unique_ptr<FramelessVisagePopup> popup;
    };
    
    std::unique_ptr<MainWindow> mainWindow;
};

// Application startup
START_JUCE_APPLICATION(ExampleApplication)
