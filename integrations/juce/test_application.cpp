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
 * Comprehensive test application for JuceVisageBridge
 * Tests all scenarios mentioned in GitHub issue #47
 */

#include "juce_visage_bridge.h"
#include <visage/ui.h>
#include <chrono>

// Test frame that simulates the problematic scenarios
class TestVisageFrame : public visage::Frame {
public:
    TestVisageFrame(const std::string& windowType) 
        : visage::Frame("TestFrame_" + windowType), windowType_(windowType) {
        setBounds(0, 0, 800, 600);
        startTime_ = std::chrono::steady_clock::now();
    }
    
    void draw(visage::Canvas& canvas) override {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime_).count();
        
        // Draw background - this should prevent magenta flashes
        canvas.setColor(0xff2a2a2a);  // Dark background
        canvas.fill(0, 0, width(), height());
        
        // Draw title bar area for draggable testing
        canvas.setColor(0xff3a3a3a);
        canvas.fill(0, 0, width(), 30);
        
        // Window type indicator
        canvas.setColor(0xffffffff);
        // Note: Text rendering may need adjustment based on actual Visage API
        
        // Test patterns to verify proper rendering
        float animPhase = (elapsed % 2000) / 2000.0f * 6.28318f; // 2 second cycle
        
        // Animated elements to test for flickering
        int offsetX = static_cast<int>(50 + 30 * std::sin(animPhase));
        int offsetY = 80;
        
        // Blue rectangle
        canvas.setColor(0xff5555ff);
        canvas.fill(offsetX, offsetY, 200, 100);
        
        // Red rectangle  
        canvas.setColor(0xffff5555);
        canvas.fill(offsetX + 250, offsetY, 200, 100);
        
        // Green rectangle
        canvas.setColor(0xff55ff55);
        canvas.fill(offsetX, offsetY + 150, 450, 100);
        
        // Status indicators
        canvas.setColor(0xffffff00);
        canvas.fill(10, height() - 40, 20, 20); // Yellow indicator for "alive"
        
        // Test for the "dark bar at bottom" issue
        // Draw something at the very bottom to see if it gets cut off
        canvas.setColor(0xffff00ff); // Magenta line to test bottom edge
        canvas.fill(0, height() - 5, width(), 5);
    }
    
    void mouseDown(const visage::MouseEvent& e) override {
        // Test mouse event forwarding
        clickCount_++;
        lastClickPos_ = e.position;
        
        // Simple interaction feedback
        if (e.isLeftButton()) {
            // Change background color briefly to show mouse events work
            backgroundHue_ = (backgroundHue_ + 0.1f);
            if (backgroundHue_ > 1.0f) backgroundHue_ = 0.0f;
        }
    }
    
    void mouseMove(const visage::MouseEvent& e) override {
        lastMousePos_ = e.position;
    }

private:
    std::string windowType_;
    std::chrono::steady_clock::time_point startTime_;
    int clickCount_ = 0;
    visage::Point lastClickPos_{0, 0};
    visage::Point lastMousePos_{0, 0};
    float backgroundHue_ = 0.0f;
};

//==============================================================================
// Test Case 1: Settings Panel (juce::DocumentWindow) - Issue #47 Case 1
class SettingsWindow : public juce::DocumentWindow {
public:
    SettingsWindow() 
        : juce::DocumentWindow("Settings Panel Test (580x650)", 
                               juce::Colours::darkgrey, 
                               juce::DocumentWindow::allButtons) 
    {
        // Create the bridge - this should solve the rendering issues
        bridge = std::make_unique<JuceVisageBridge>();
        
        // Create test Visage content
        visageFrame = std::make_unique<TestVisageFrame>("SettingsPanel");
        
        // Connect them together
        bridge->setRootFrame(visageFrame.get());
        
        // Set the bridge as window content
        setContentNonOwned(bridge.get(), true);
        
        // Configure window - exact dimensions from the issue
        setSize(580, 650);
        setResizable(true, true);
        centreWithSize(getWidth(), getHeight());
        setVisible(true);
    }
    
    void closeButtonPressed() override {
        setVisible(false);
    }

private:
    std::unique_ptr<JuceVisageBridge> bridge;
    std::unique_ptr<TestVisageFrame> visageFrame;
};

//==============================================================================
// Test Case 2: Frameless Waveform Editor - Issue #47 Case 2
class WaveformEditorWindow : public juce::Component {
public:
    WaveformEditorWindow() {
        // Create the bridge
        bridge = std::make_unique<JuceVisageBridge>();
        
        // Enable dragging for the top 30 pixels (title bar area)
        // This addresses the "non-draggable" issue
        bridge->setDraggableTitleHeight(30);
        
        // Create test Visage content
        visageFrame = std::make_unique<TestVisageFrame>("WaveformEditor");
        bridge->setRootFrame(visageFrame.get());
        
        // Add to this component
        addAndMakeVisible(bridge.get());
        
        // Set size and show as desktop component - exact dimensions from issue
        setSize(950, 920);
        addToDesktop(juce::ComponentPeer::windowIsTemporary | 
                    juce::ComponentPeer::windowHasDropShadow);
        
        // Position offset from the settings window
        setTopLeftPosition(100, 100);
        setVisible(true);
    }
    
    void resized() override {
        bridge->setBounds(getLocalBounds());
    }
    
    void paint(juce::Graphics& g) override {
        // Ensure we don't show any JUCE background
        // The bridge should handle all rendering
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
    std::unique_ptr<TestVisageFrame> visageFrame;
};

//==============================================================================
// Test Case 3: Plugin-style window (main plugin window 600x730 from issue)
class PluginMainWindow : public juce::AudioProcessorEditor {
public:
    PluginMainWindow() : juce::AudioProcessorEditor(dummyProcessor) {
        // Create the bridge
        bridge = std::make_unique<JuceVisageBridge>();
        addAndMakeVisible(bridge.get());
        
        // Create test Visage content
        visageFrame = std::make_unique<TestVisageFrame>("PluginMain");
        bridge->setRootFrame(visageFrame.get());
        
        // Set size from issue description - "tall and narrow"
        setSize(600, 730);
    }
    
    void resized() override {
        bridge->setBounds(getLocalBounds());
    }

private:
    // Dummy processor for testing
    struct DummyProcessor : public juce::AudioProcessor {
        const juce::String getName() const override { return "Test"; }
        void prepareToPlay(double, int) override {}
        void releaseResources() override {}
        void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override {}
        double getTailLengthSeconds() const override { return 0; }
        bool acceptsMidi() const override { return false; }
        bool producesMidi() const override { return false; }
        juce::AudioProcessorEditor* createEditor() override { return nullptr; }
        bool hasEditor() const override { return false; }
        int getNumPrograms() override { return 1; }
        int getCurrentProgram() override { return 0; }
        void setCurrentProgram(int) override {}
        const juce::String getProgramName(int) override { return "Default"; }
        void changeProgramName(int, const juce::String&) override {}
        void getStateInformation(juce::MemoryBlock&) override {}
        void setStateInformation(const void*, int) override {}
    } dummyProcessor;
    
    std::unique_ptr<JuceVisageBridge> bridge;
    std::unique_ptr<TestVisageFrame> visageFrame;
};

//==============================================================================
// Test Application with comprehensive testing UI
class VisageBridgeTestApplication : public juce::JUCEApplication {
public:
    const juce::String getApplicationName() override { 
        return "Visage JUCE Integration Test (Issue #47)"; 
    }
    const juce::String getApplicationVersion() override { return "1.0.0"; }
    
    void initialise(const juce::String&) override {
        mainWindow = std::make_unique<MainTestWindow>();
    }
    
    void shutdown() override {
        mainWindow.reset();
    }

private:
    class MainTestWindow : public juce::DocumentWindow {
    public:
        MainTestWindow() : juce::DocumentWindow("Visage Bridge Test Suite", 
                                               juce::Colours::lightgrey, 
                                               allButtons) {
            auto content = std::make_unique<juce::Component>();
            content->setSize(400, 300);
            
            // Instructions
            auto instructions = std::make_unique<juce::Label>("instructions", 
                "Test Suite for GitHub Issue #47\n\n"
                "Click buttons to test different window scenarios.\n"
                "Watch for:\n"
                "• NO magenta flashes on window creation\n"
                "• NO flickering or dark bars\n"
                "• Draggable title areas work correctly\n"
                "• Smooth 60fps animation\n"
                "• Mouse events respond properly");
            instructions->setBounds(20, 20, 360, 120);
            instructions->setJustificationType(juce::Justification::topLeft);
            content->addAndMakeVisible(instructions.get());
            
            // Test buttons
            auto settingsButton = std::make_unique<juce::TextButton>("Test Settings Panel (580x650)");
            settingsButton->onClick = [this] {
                // Test Case 1: Settings panel with DocumentWindow
                settingsWindow = std::make_unique<SettingsWindow>();
            };
            settingsButton->setBounds(20, 150, 360, 30);
            content->addAndMakeVisible(settingsButton.get());
            
            auto waveformButton = std::make_unique<juce::TextButton>("Test Waveform Editor (950x920 Frameless)");
            waveformButton->onClick = [this] {
                // Test Case 2: Frameless draggable window
                waveformWindow = std::make_unique<WaveformEditorWindow>();
            };
            waveformButton->setBounds(20, 190, 360, 30);
            content->addAndMakeVisible(waveformButton.get());
            
            auto pluginButton = std::make_unique<juce::TextButton>("Test Plugin Main Window (600x730)");
            pluginButton->onClick = [this] {
                // Test Case 3: Plugin-style main window
                if (pluginWindow == nullptr) {
                    pluginWindow = std::make_unique<PluginMainWindow>();
                    auto pluginWindowPtr = std::make_unique<juce::DocumentWindow>("Plugin Main Window", 
                                                                                 juce::Colours::black, 
                                                                                 juce::DocumentWindow::closeButton);
                    pluginWindowPtr->setContentNonOwned(pluginWindow.get(), true);
                    pluginWindowPtr->setSize(600, 730);
                    pluginWindowPtr->setTopLeftPosition(650, 50);
                    pluginWindowPtr->setVisible(true);
                    pluginHost = std::move(pluginWindowPtr);
                }
            };
            pluginButton->setBounds(20, 230, 360, 30);
            content->addAndMakeVisible(pluginButton.get());
            
            // Store references
            this->instructions = std::move(instructions);
            this->settingsButton = std::move(settingsButton);
            this->waveformButton = std::move(waveformButton);
            this->pluginButton = std::move(pluginButton);
            
            setContentOwned(content.release(), true);
            setSize(400, 300);
            setVisible(true);
        }
        
        void closeButtonPressed() override {
            juce::JUCEApplication::getInstance()->systemRequestedQuit();
        }

    private:
        std::unique_ptr<juce::Label> instructions;
        std::unique_ptr<juce::TextButton> settingsButton;
        std::unique_ptr<juce::TextButton> waveformButton;
        std::unique_ptr<juce::TextButton> pluginButton;
        
        std::unique_ptr<SettingsWindow> settingsWindow;
        std::unique_ptr<WaveformEditorWindow> waveformWindow;
        std::unique_ptr<PluginMainWindow> pluginWindow;
        std::unique_ptr<juce::DocumentWindow> pluginHost;
    };
    
    std::unique_ptr<MainTestWindow> mainWindow;
};

// Application startup
START_JUCE_APPLICATION(VisageBridgeTestApplication)
