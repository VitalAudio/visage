# Visage JUCE Bridge Test Plan
## Testing GitHub Issue #47 Fix

This document outlines the comprehensive testing procedure for the JuceVisageBridge solution that addresses rendering issues in secondary JUCE windows.

## Test Environment Setup

### Prerequisites
1. JUCE Framework (v7.0+ recommended, tested with 8.0.7 mentioned in issue)
2. Visage library (latest from repository)
3. OpenGL support enabled
4. CMake build system
5. Test platform: macOS 15.5+ (primary platform from issue), Windows 10+, Linux

### Build Instructions
```bash
# Enable JUCE integration during build
cmake -B build -DVISAGE_BUILD_JUCE_INTEGRATION=ON -DJUCE_DIR=/path/to/JUCE
cmake --build build

# If you have JUCE installed, run the test application
cd integrations/juce
# Compile test_application.cpp with JUCE and Visage
```

## Test Cases

### Test Case 1: Settings Panel (juce::DocumentWindow)
**Scenario**: Titled window (580x650) hosting Visage UI
**Reference**: Issue #47 "Issue 1: Titled juce::DocumentWindow"

#### Expected Behavior (PASS criteria):
- ✅ **No Magenta Flash**: Window opens with solid background, no purple/magenta flash
- ✅ **Clean Title Bar**: Native OS title bar appears correctly and stays visible
- ✅ **No Overpainting**: Visage content doesn't paint over the title bar
- ✅ **No Bottom Bar**: No dark/flickering bar at the bottom of the window
- ✅ **Draggable**: Window can be dragged by its native title bar
- ✅ **Resizable**: Window resizes correctly without artifacts
- ✅ **Opaque Rendering**: Background is solid, not transparent

#### What to Watch For (FAIL indicators):
- ❌ Purple/magenta flash during window creation
- ❌ Title bar appears briefly then disappears
- ❌ Flickering elements
- ❌ Dark bar at bottom (roughly title bar height)
- ❌ Window becomes undraggable
- ❌ Transparent or see-through background

### Test Case 2: Frameless Waveform Editor (juce::Component)
**Scenario**: Frameless window (950x920) with custom draggable area
**Reference**: Issue #47 "Issue 2: Frameless Waveform Editor Window"

#### Expected Behavior (PASS criteria):
- ✅ **No Magenta Flash**: Clean opening without purple flash
- ✅ **Draggable Title Area**: Top 30px area allows window dragging
- ✅ **Custom Title Bar**: Visage-drawn title area is functional
- ✅ **No Bottom Bar**: No persistent dark bar at bottom
- ✅ **Full Coverage**: Visage UI covers entire window area
- ✅ **Mouse Events**: Click/drag events work throughout the window
- ✅ **Escape to Close**: ESC key closes the window

#### What to Watch For (FAIL indicators):
- ❌ Magenta background with transparent Visage UI on top
- ❌ Cannot drag the window at all
- ❌ Dragging only works in wrong areas
- ❌ Persistent dark bar at bottom
- ❌ Mouse events not responding

### Test Case 3: Plugin Main Window (600x730)
**Scenario**: Plugin-style main window (tall and narrow)
**Reference**: Issue description mentions "main plugin window (600x730, tall and narrow)"

#### Expected Behavior (PASS criteria):
- ✅ **No Title Bar Issues**: No missing draggable title in AU/VST context
- ✅ **Correct Dimensions**: Proper 600x730 rendering without bars
- ✅ **Plugin Host Compatibility**: Works within plugin host window
- ✅ **No Flickering**: Smooth rendering without flash
- ✅ **Proper Aspect Ratio**: Tall/narrow layout maintained

#### What to Watch For (FAIL indicators):
- ❌ Title bar height missing from bottom
- ❌ Canvas drawn incorrectly
- ❌ Aspect ratio problems

## Performance & Visual Tests

### Animation Smoothness Test
**Purpose**: Verify 60fps rendering without stuttering
- ✅ **Smooth Animation**: Colored rectangles move smoothly
- ✅ **No Frame Drops**: Consistent frame rate
- ✅ **No Tearing**: Clean movement without visual artifacts

### Mouse Interaction Test
**Purpose**: Verify proper event forwarding to Visage
- ✅ **Click Detection**: Left clicks change visual feedback
- ✅ **Mouse Movement**: Cursor tracking works
- ✅ **Hover Effects**: Mouse-over areas respond
- ✅ **Drag Operations**: Both window dragging and UI dragging work

### Initialization Order Test
**Purpose**: Verify proper OpenGL context setup
- ✅ **Context Ready**: No rendering before OpenGL context
- ✅ **Thread Safety**: All operations on correct threads
- ✅ **Resource Cleanup**: Clean shutdown without crashes

## Detailed Testing Procedure

### Step 1: Run Test Application
1. Launch the test application
2. **First Check**: Main window should appear without any magenta flash
3. Read the instructions in the main window

### Step 2: Test Settings Panel
1. Click "Test Settings Panel (580x650)"
2. **Immediate Check**: Watch for magenta flash during window creation
3. **Visual Check**: Verify title bar is present and not painted over
4. **Interaction Check**: Try dragging the window by title bar
5. **Resize Check**: Resize window and check for artifacts
6. **Animation Check**: Watch the moving colored rectangles for smoothness
7. **Bottom Check**: Look for any dark bar at the bottom edge
8. **Close**: Close window and verify clean shutdown

### Step 3: Test Waveform Editor
1. Click "Test Waveform Editor (950x920 Frameless)"
2. **Immediate Check**: No magenta flash on creation
3. **Draggable Check**: Try dragging in the top 30-pixel area
4. **Coverage Check**: Verify Visage UI fills entire window
5. **Border Check**: No dark bars anywhere, especially bottom
6. **Mouse Check**: Click on colored rectangles for interaction
7. **Escape Check**: Press ESC to close

### Step 4: Test Plugin Window
1. Click "Test Plugin Main Window (600x730)"
2. **Proportion Check**: Window should be tall and narrow
3. **Canvas Check**: No missing areas or incorrect dimensions
4. **Plugin Context**: Verify it works in the host window context

### Step 5: Multiple Window Test
1. Open all three test windows simultaneously
2. **Memory Check**: No memory leaks or performance degradation
3. **Context Check**: Each window renders independently
4. **Interaction Check**: All windows respond to mouse/keyboard

### Step 6: Stress Testing
1. Rapidly open/close windows
2. Resize windows quickly
3. Move windows around screen
4. **Stability Check**: No crashes or rendering corruption

## Reporting Results

### For Each Test Case, Report:

#### PASS Example:
```
✅ Settings Panel Test - PASS
- No magenta flash observed
- Title bar visible and functional
- Dragging works correctly
- No bottom bar artifacts
- Smooth 60fps animation
- Mouse events responding
- Clean shutdown
```

#### FAIL Example:
```
❌ Waveform Editor Test - FAIL
- Purple flash on window creation (0.2 seconds)
- Dragging not working in title area
- Dark bar visible at bottom (approximately 22px height)
- Animation stuttering below 30fps
- Mouse clicks not registering in UI elements
```

### System Information to Include:
- OS Version (e.g., macOS 15.5, Windows 11, Ubuntu 22.04)
- JUCE Version
- Graphics Hardware (integrated vs dedicated GPU)
- Display Configuration (single/multiple monitors, scaling)
- Any relevant error messages from console/logs

### Critical Issues to Report Immediately:
1. **Magenta Flash**: Any purple/magenta coloring during window creation
2. **Non-Draggable**: Windows that cannot be moved when they should be
3. **Bottom Bars**: Dark bars at window bottom (suggests layout issue)
4. **Crashes**: Any application crashes during testing
5. **Performance**: Frame rates below 30fps or stuttering

### Success Criteria Summary:
The fix is successful if ALL test cases show:
- No magenta flashes
- Proper window dragging functionality
- No visual artifacts (bars, flickering)
- Smooth 60fps performance
- Correct mouse event handling
- Clean initialization and shutdown

## Notes for Developers

If any tests fail, check:
1. OpenGL context initialization timing
2. Canvas viewport sizing
3. Event forwarding implementation
4. Thread safety in resize operations
5. Proper cleanup in destructors

The solution should work across all plugin formats (VST3, AU, CLAP) and standalone applications.
