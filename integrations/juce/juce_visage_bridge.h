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

#ifndef JUCE_VISAGE_BRIDGE_H
#define JUCE_VISAGE_BRIDGE_H

#include <juce_core/juce_core.h>
#include <juce_graphics/juce_graphics.h>
#include <juce_gui_basics/juce_gui_basics.h>
#include <juce_opengl/juce_opengl.h>

#include <visage/graphics.h>
#include <visage/ui.h>

/**
 * JuceVisageBridge provides seamless integration between JUCE and Visage UI.
 * 
 * This class solves rendering issues commonly encountered when hosting Visage UI
 * in secondary JUCE windows, including:
 * - Magenta flashes and transparency artifacts
 * - Flickering and dark bars
 * - Non-draggable frameless windows
 * 
 * Usage:
 * 1. Create a JuceVisageBridge instance
 * 2. Set it as content of your JUCE window/component
 * 3. Set the root Visage frame using setRootFrame()
 * 4. For frameless windows, optionally set draggable title height
 */
class JuceVisageBridge : public juce::Component, public juce::OpenGLRenderer {
public:
    JuceVisageBridge();
    ~JuceVisageBridge();

    /**
     * Set the root Visage frame to be rendered.
     * @param frame The Visage frame to render (can be nullptr to clear)
     */
    void setRootFrame(visage::Frame* frame);
    
    /**
     * Get the current root frame.
     * @return The current root frame, or nullptr if none set
     */
    visage::Frame* getRootFrame() const { return rootFrame; }

    /**
     * Set height for draggable title area (useful for frameless windows).
     * @param height Height in pixels of the draggable area (0 disables dragging)
     */
    void setDraggableTitleHeight(int height) { draggableTitleHeight = height; }

    /**
     * Get the current draggable title height.
     * @return Height in pixels of the draggable area
     */
    int getDraggableTitleHeight() const { return draggableTitleHeight; }

    // Component overrides
    void paint(juce::Graphics& g) override;
    void resized() override;

private:
    // OpenGLRenderer overrides
    void newOpenGLContextCreated() override;
    void renderOpenGL() override;
    void openGLContextClosing() override;

    // Mouse handling for draggable support
    void mouseDown(const juce::MouseEvent& event) override;
    void mouseDrag(const juce::MouseEvent& event) override;

    juce::OpenGLContext openGLContext;
    visage::Frame* rootFrame = nullptr;
    visage::Canvas canvas;
    int draggableTitleHeight = 0;
    juce::ComponentDragger componentDragger;
    bool isInitialized = false;
};

#endif // JUCE_VISAGE_BRIDGE_H
