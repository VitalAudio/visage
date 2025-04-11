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

#if VISAGE_MAC
#include "windowing_macos.h"

#include "visage_utils/file_system.h"
#include "visage_utils/time_utils.h"

#include <map>

namespace visage {

  class NativeWindowLookup {
  public:
    static NativeWindowLookup& instance() {
      static NativeWindowLookup instance;
      return instance;
    }

    void addWindow(WindowMac* window) { native_window_lookup_[window->nativeHandle()] = window; }

    void removeWindow(WindowMac* window) {
      if (native_window_lookup_.count(window->nativeHandle()))
        native_window_lookup_.erase(window->nativeHandle());
    }

    bool anyWindowOpen() const {
      for (auto& window : native_window_lookup_) {
        if (window.second->isShowing())
          return true;
      }
      return false;
    }

    WindowMac* findWindow(void* handle) {
      auto it = native_window_lookup_.find((void*)handle);
      return it != native_window_lookup_.end() ? it->second : nullptr;
    }

  private:
    NativeWindowLookup() = default;
    ~NativeWindowLookup() = default;

    std::map<void*, WindowMac*> native_window_lookup_;
  };

  class InitialMetalLayer {
  public:
    static CAMetalLayer* layer() { return instance().metal_layer_; }

  private:
    static InitialMetalLayer& instance() {
      static InitialMetalLayer instance;
      return instance;
    }

    InitialMetalLayer() {
      metal_layer_ = [CAMetalLayer layer];
      metal_layer_.colorspace = CGColorSpaceCreateWithName(kCGColorSpaceDisplayP3);
    }

    CAMetalLayer* metal_layer_ = nullptr;
  };

  std::string readClipboardText() {
    NSPasteboard* pasteboard = [NSPasteboard generalPasteboard];
    NSString* clipboard_text = [pasteboard stringForType:NSPasteboardTypeString];
    if (clipboard_text != nil)
      return [clipboard_text UTF8String];
    return "";
  }

  void setClipboardText(const std::string& text) {
    NSPasteboard* pasteboard = [NSPasteboard generalPasteboard];
    [pasteboard clearContents];
    NSString* ns_text = [NSString stringWithUTF8String:text.c_str()];
    [pasteboard setString:ns_text forType:NSPasteboardTypeString];
  }

  void setCursorStyle(MouseCursor style) {
    static const NSCursor* arrow_cursor = [NSCursor arrowCursor];
    static const NSCursor* ibeam_cursor = [NSCursor IBeamCursor];
    static const NSCursor* crosshair_cursor = [NSCursor crosshairCursor];
    static const NSCursor* pointing_cursor = [NSCursor pointingHandCursor];
    static const NSCursor* horizontal_resize_cursor = [NSCursor resizeLeftRightCursor];
    static const NSCursor* vertical_resize_cursor = [NSCursor resizeUpDownCursor];
    static const NSCursor* multi_directional_resize_cursor = [NSCursor openHandCursor];

    const NSCursor* cursor = nil;
    switch (style) {
    case MouseCursor::Arrow: cursor = arrow_cursor; break;
    case MouseCursor::IBeam: cursor = ibeam_cursor; break;
    case MouseCursor::Crosshair: cursor = crosshair_cursor; break;
    case MouseCursor::Pointing: cursor = pointing_cursor; break;
    case MouseCursor::HorizontalResize: cursor = horizontal_resize_cursor; break;
    case MouseCursor::VerticalResize: cursor = vertical_resize_cursor; break;
    case MouseCursor::MultiDirectionalResize: cursor = multi_directional_resize_cursor; break;
    default: return;
    }
    [cursor set];
  }

  Point cursorPosition() {
    CGEventRef event = CGEventCreate(nullptr);
    CGPoint cursor = CGEventGetLocation(event);
    CFRelease(event);
    return { static_cast<float>(cursor.x), static_cast<float>(cursor.y) };
  }

  void setCursorVisible(bool visible) {
    if (visible)
      [NSCursor unhide];
    else
      [NSCursor hide];
  }

  CGRect activeWindowBounds() {
    NSArray* windows = [NSApp windows];

    for (NSWindow* window in windows) {
      if ([window isMainWindow])
        return [window frame];
    }

    return NSMakeRect(0, 0, 0, 0);
  }

  void setCursorScreenPosition(Point screen_position) {
    CGPoint position = CGPointMake(screen_position.x, screen_position.y);
    CGAssociateMouseAndMouseCursorPosition(false);
    CGWarpMouseCursorPosition(position);
    CGAssociateMouseAndMouseCursorPosition(true);
  }

  void setCursorPosition(Point window_position) {
    CGRect window_bounds = activeWindowBounds();
    float x = window_bounds.origin.x + window_position.x;
    float y = window_bounds.origin.y + window_position.y;
    setCursorScreenPosition({ x, y });
  }

  static Point windowBorderSize(NSWindow* window_handle) {
    if (window_handle == nullptr)
      return { 0, 0 };
    NSRect frame = [window_handle frame];
    NSRect content_rect = [window_handle contentRectForFrameRect:frame];
    float x = frame.size.width - content_rect.size.width;
    float y = frame.size.height - content_rect.size.height;
    return { x, y };
  }

  bool isMobileDevice() {
    return false;
  }

  KeyCode translateKeyCode(int mac_key_code) {
    switch (mac_key_code) {
    case kVK_ANSI_A: return KeyCode::A;
    case kVK_ANSI_S: return KeyCode::S;
    case kVK_ANSI_D: return KeyCode::D;
    case kVK_ANSI_F: return KeyCode::F;
    case kVK_ANSI_H: return KeyCode::H;
    case kVK_ANSI_G: return KeyCode::G;
    case kVK_ANSI_Z: return KeyCode::Z;
    case kVK_ANSI_X: return KeyCode::X;
    case kVK_ANSI_C: return KeyCode::C;
    case kVK_ANSI_V: return KeyCode::V;
    case kVK_ANSI_B: return KeyCode::B;
    case kVK_ANSI_Q: return KeyCode::Q;
    case kVK_ANSI_W: return KeyCode::W;
    case kVK_ANSI_E: return KeyCode::E;
    case kVK_ANSI_R: return KeyCode::R;
    case kVK_ANSI_Y: return KeyCode::Y;
    case kVK_ANSI_T: return KeyCode::T;
    case kVK_ANSI_1: return KeyCode::Number1;
    case kVK_ANSI_2: return KeyCode::Number2;
    case kVK_ANSI_3: return KeyCode::Number3;
    case kVK_ANSI_4: return KeyCode::Number4;
    case kVK_ANSI_6: return KeyCode::Number6;
    case kVK_ANSI_5: return KeyCode::Number5;
    case kVK_ANSI_Equal: return KeyCode::Equals;
    case kVK_ANSI_9: return KeyCode::Number9;
    case kVK_ANSI_7: return KeyCode::Number7;
    case kVK_ANSI_Minus: return KeyCode::Minus;
    case kVK_ANSI_8: return KeyCode::Number8;
    case kVK_ANSI_0: return KeyCode::Number0;
    case kVK_ANSI_RightBracket: return KeyCode::RightBracket;
    case kVK_ANSI_O: return KeyCode::O;
    case kVK_ANSI_U: return KeyCode::U;
    case kVK_ANSI_LeftBracket: return KeyCode::LeftBracket;
    case kVK_ANSI_I: return KeyCode::I;
    case kVK_ANSI_P: return KeyCode::P;
    case kVK_ANSI_L: return KeyCode::L;
    case kVK_ANSI_J: return KeyCode::J;
    case kVK_ANSI_Quote: return KeyCode::Apostrophe;
    case kVK_ANSI_K: return KeyCode::K;
    case kVK_ANSI_Semicolon: return KeyCode::Semicolon;
    case kVK_ANSI_Backslash: return KeyCode::Backslash;
    case kVK_ANSI_Comma: return KeyCode::Comma;
    case kVK_ANSI_Slash: return KeyCode::Slash;
    case kVK_ANSI_N: return KeyCode::N;
    case kVK_ANSI_M: return KeyCode::M;
    case kVK_ANSI_Period: return KeyCode::Period;
    case kVK_ANSI_Grave: return KeyCode::Grave;
    case kVK_ANSI_KeypadDecimal: return KeyCode::KPDecimal;
    case kVK_ANSI_KeypadMultiply: return KeyCode::KPMultiply;
    case kVK_ANSI_KeypadPlus: return KeyCode::KPPlus;
    case kVK_ANSI_KeypadClear: return KeyCode::KPClear;
    case kVK_ANSI_KeypadDivide: return KeyCode::KPDivide;
    case kVK_ANSI_KeypadEnter: return KeyCode::KPEnter;
    case kVK_ANSI_KeypadMinus: return KeyCode::KPMinus;
    case kVK_ANSI_KeypadEquals: return KeyCode::KPEquals;
    case kVK_ANSI_Keypad0: return KeyCode::KP0;
    case kVK_ANSI_Keypad1: return KeyCode::KP1;
    case kVK_ANSI_Keypad2: return KeyCode::KP2;
    case kVK_ANSI_Keypad3: return KeyCode::KP3;
    case kVK_ANSI_Keypad4: return KeyCode::KP4;
    case kVK_ANSI_Keypad5: return KeyCode::KP5;
    case kVK_ANSI_Keypad6: return KeyCode::KP6;
    case kVK_ANSI_Keypad7: return KeyCode::KP7;
    case kVK_ANSI_Keypad8: return KeyCode::KP8;
    case kVK_ANSI_Keypad9: return KeyCode::KP9;
    case kVK_Return: return KeyCode::Return;
    case kVK_Tab: return KeyCode::Tab;
    case kVK_Space: return KeyCode::Space;
    case kVK_Delete: return KeyCode::Backspace;
    case kVK_Escape: return KeyCode::Escape;
    case kVK_Command: return KeyCode::LGui;
    case kVK_Shift: return KeyCode::LShift;
    case kVK_CapsLock: return KeyCode::CapsLock;
    case kVK_Option: return KeyCode::LAlt;
    case kVK_Control: return KeyCode::LCtrl;
    case kVK_RightCommand: return KeyCode::RGui;
    case kVK_RightShift: return KeyCode::RShift;
    case kVK_RightOption: return KeyCode::RAlt;
    case kVK_RightControl: return KeyCode::RCtrl;
    case kVK_VolumeUp: return KeyCode::VolumeUp;
    case kVK_VolumeDown: return KeyCode::VolumeDown;
    case kVK_Mute: return KeyCode::Mute;
    case kVK_F1: return KeyCode::F1;
    case kVK_F2: return KeyCode::F2;
    case kVK_F3: return KeyCode::F3;
    case kVK_F4: return KeyCode::F4;
    case kVK_F5: return KeyCode::F5;
    case kVK_F6: return KeyCode::F6;
    case kVK_F7: return KeyCode::F7;
    case kVK_F8: return KeyCode::F8;
    case kVK_F9: return KeyCode::F9;
    case kVK_F10: return KeyCode::F10;
    case kVK_F11: return KeyCode::F11;
    case kVK_F12: return KeyCode::F12;
    case kVK_F13: return KeyCode::F13;
    case kVK_F14: return KeyCode::F14;
    case kVK_F15: return KeyCode::F15;
    case kVK_F16: return KeyCode::F16;
    case kVK_F17: return KeyCode::F17;
    case kVK_F18: return KeyCode::F18;
    case kVK_F19: return KeyCode::F19;
    case kVK_F20: return KeyCode::F20;
    case kVK_Help: return KeyCode::Help;
    case kVK_Home: return KeyCode::Home;
    case kVK_PageUp: return KeyCode::PageUp;
    case kVK_ForwardDelete: return KeyCode::Delete;
    case kVK_End: return KeyCode::End;
    case kVK_PageDown: return KeyCode::PageDown;
    case kVK_LeftArrow: return KeyCode::Left;
    case kVK_RightArrow: return KeyCode::Right;
    case kVK_DownArrow: return KeyCode::Down;
    case kVK_UpArrow: return KeyCode::Up;
    default: return KeyCode::Unknown;
    }
  }
}

@implementation VisageDraggingSource
- (NSDragOperation)draggingSession:(NSDraggingSession*)session
    sourceOperationMaskForDraggingContext:(NSDraggingContext)context {
  return NSDragOperationCopy;
}
@end

@implementation VisageAppViewDelegate
- (instancetype)initWithWindow:(visage::WindowMac*)window {
  self = [super init];
  self.visage_window = window;
  self.start_microseconds = visage::time::microseconds();
  return self;
}

- (void)drawWindow {
  long long ms = visage::time::microseconds();
  self.visage_window->drawCallback((ms - self.start_microseconds) / 1000000.0);
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {
  self.visage_window->handleNativeResize(size.width, size.height);
}

- (void)drawInMTKView:(MTKView*)view {
  if (!view.currentDrawable || !view.currentRenderPassDescriptor)
    return;

  view.layer.contentsScale = self.visage_window->dpiScale();
  [self drawWindow];
}
@end

@implementation VisageAppView
- (instancetype)initWithFrame:(NSRect)frame_rect inWindow:(visage::WindowMac*)window {
  self = [super initWithFrame:frame_rect];
  self.visage_window = window;
  self.device = MTLCreateSystemDefaultDevice();
  self.clearColor = MTLClearColorMake(0.1, 0.1, 0.1, 1.0);
  self.enableSetNeedsDisplay = NO;
  self.framebufferOnly = YES;
  self.layerContentsPlacement = NSViewLayerContentsPlacementTopLeft;
  self.preferredFramesPerSecond = 120;

  [self registerForDraggedTypes:@[NSPasteboardTypeFileURL]];
  self.drag_source = [[VisageDraggingSource alloc] init];

  return self;
}

- (BOOL)acceptsFirstResponder {
  return YES;
}

- (void)keyDown:(NSEvent*)event {
  bool command = [event modifierFlags] & NSEventModifierFlagCommand;
  bool q_or_w = [[event charactersIgnoringModifiers] isEqualToString:@"q"] ||
                [[event charactersIgnoringModifiers] isEqualToString:@"w"];
  if (self.allow_quit && command && q_or_w) {
    [NSApp stop:nil];
    return;
  }

  int modifiers = [self keyboardModifiers:event];
  if ((modifiers & visage::kModifierCmd) == 0)
    [self interpretKeyEvents:[NSArray arrayWithObject:event]];

  visage::KeyCode key_code = visage::translateKeyCode([event keyCode]);
  if (!self.visage_window->handleKeyDown(key_code, modifiers, [event isARepeat]))
    [super keyDown:event];
}

- (void)keyUp:(NSEvent*)event {
  visage::KeyCode key_code = visage::translateKeyCode([event keyCode]);
  if (!self.visage_window->handleKeyUp(key_code, [self keyboardModifiers:event]))
    [super keyUp:event];
}

- (void)insertText:(id)string {
  self.visage_window->handleTextInput([string UTF8String]);
}

- (void)moveLeft:(id)sender {
}
- (void)moveRight:(id)sender {
}
- (void)moveUp:(id)sender {
}
- (void)moveDown:(id)sender {
}
- (void)deleteForward:(id)sender {
}
- (void)deleteBackward:(id)sender {
}
- (void)insertTab:(id)sender {
}
- (void)insertBacktab:(id)sender {
}
- (void)insertNewline:(id)sender {
}
- (void)insertParagraphSeparator:(id)sender {
}
- (void)moveToBeginningOfLine:(id)sender {
}
- (void)moveToEndOfLine:(id)sender {
}
- (void)moveToBeginningOfDocument:(id)sender {
}
- (void)moveToEndOfDocument:(id)sender {
}
- (void)cancelOperation:(id)sender {
}

- (visage::Point)eventPosition:(NSEvent*)event {
  NSPoint location = [event locationInWindow];
  NSPoint view_location = [self convertPoint:location fromView:nil];
  CGFloat view_height = self.frame.size.height;
  return visage::Point(view_location.x, view_height - view_location.y) * self.visage_window->dpiScale();
}

- (visage::Point)dragPosition:(id<NSDraggingInfo>)sender {
  NSPoint drag_point = [self convertPoint:[sender draggingLocation] fromView:nil];
  CGFloat view_height = self.frame.size.height;
  return visage::Point(drag_point.x, view_height - drag_point.y) * self.visage_window->dpiScale();
}

- (NSPoint)mouseScreenPosition {
  NSPoint result = [NSEvent mouseLocation];
  if ([[NSScreen screens] count] == 0)
    return result;

  result.y = [[[NSScreen screens] objectAtIndex:0] frame].size.height - result.y;
  return result;
}

- (int)mouseButtonState {
  NSUInteger buttons = [NSEvent pressedMouseButtons];
  int result = 0;
  if (buttons & (1 << 0))
    result = result | visage::kMouseButtonLeft;
  if (buttons & (1 << 1))
    result = result | visage::kMouseButtonRight;
  if (buttons & (1 << 2))
    result = result | visage::kMouseButtonMiddle;
  return result;
}

- (int)keyboardModifiers:(NSEvent*)event {
  NSUInteger flags = [event modifierFlags];
  int result = 0;
  if (flags & NSEventModifierFlagCommand)
    result = result | visage::kModifierCmd;
  if (flags & NSEventModifierFlagControl)
    result = result | visage::kModifierMacCtrl;
  if (flags & NSEventModifierFlagOption)
    result = result | visage::kModifierOption;
  if (flags & NSEventModifierFlagShift)
    result = result | visage::kModifierShift;
  return result;
}

- (void)checkRelativeMode {
  if (self.visage_window->mouseRelativeMode()) {
    CGAssociateMouseAndMouseCursorPosition(false);
    CGWarpMouseCursorPosition(self.mouse_down_screen_position);
    CGAssociateMouseAndMouseCursorPosition(true);
  }
}

- (void)makeKeyWindow {
  if (!self.visage_window->isPopup())
    [self.window makeKeyWindow];
}

- (void)scrollWheel:(NSEvent*)event {
  static constexpr float kPreciseScrollingScale = 0.008f;
  visage::Point point = [self eventPosition:event];
  float delta_x = [event deltaX];
  float precise_x = delta_x;
  float delta_y = [event deltaY];
  float precise_y = delta_y;
  if ([event hasPreciseScrollingDeltas]) {
    precise_x = [event scrollingDeltaX] * kPreciseScrollingScale;
    precise_y = [event scrollingDeltaY] * kPreciseScrollingScale;
  }
  self.visage_window->handleMouseWheel(delta_x, delta_y, precise_x, precise_y, point.x, point.y,
                                       [self mouseButtonState], [self keyboardModifiers:event],
                                       [event momentumPhase] != NSEventPhaseNone);
}

- (void)mouseMoved:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.visage_window->handleMouseMove(point.x, point.y, [self mouseButtonState],
                                      [self keyboardModifiers:event]);
}

- (void)mouseEntered:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.visage_window->handleMouseMove(point.x, point.y, [self mouseButtonState],
                                      [self keyboardModifiers:event]);
}

- (void)mouseExited:(NSEvent*)event {
  self.visage_window->handleMouseLeave([self mouseButtonState], [self keyboardModifiers:event]);
}

- (void)mouseDown:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.mouse_down_screen_position = [self mouseScreenPosition];
  self.visage_window->handleMouseDown(visage::kMouseButtonLeft, point.x, point.y,
                                      [self mouseButtonState], [self keyboardModifiers:event]);
  [self makeKeyWindow];
  if (self.visage_window->isDragDropSource()) {
    visage::File file = self.visage_window->startDragDropSource();
    NSString* path = [NSString stringWithUTF8String:file.string().c_str()];
    NSURL* url = [NSURL fileURLWithPath:path];

    NSImage* image = [[NSWorkspace sharedWorkspace] iconForFile:path];
    NSPasteboardItem* pasteboard_item = [[NSPasteboardItem alloc] init];
    [pasteboard_item setString:url.absoluteString forType:NSPasteboardTypeFileURL];

    NSDraggingItem* dragging_item = [[NSDraggingItem alloc] initWithPasteboardWriter:pasteboard_item];
    NSPoint drag_position = [self convertPoint:event.locationInWindow fromView:nil];
    drag_position.x -= image.size.width / 2;
    drag_position.y -= image.size.height / 2;
    [dragging_item
        setDraggingFrame:NSMakeRect(drag_position.x, drag_position.y, image.size.width, image.size.height)
                contents:image];

    [self beginDraggingSessionWithItems:[NSArray arrayWithObject:dragging_item]
                                  event:event
                                 source:self.drag_source];
  }
}

- (void)mouseUp:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.visage_window->handleMouseUp(visage::kMouseButtonLeft, point.x, point.y,
                                    [self mouseButtonState], [self keyboardModifiers:event]);
}

- (void)mouseDragged:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.visage_window->handleMouseMove(point.x, point.y, [self mouseButtonState],
                                      [self keyboardModifiers:event]);
  [self checkRelativeMode];
}

- (void)rightMouseDown:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.mouse_down_screen_position = [self mouseScreenPosition];
  self.visage_window->handleMouseDown(visage::kMouseButtonRight, point.x, point.y,
                                      [self mouseButtonState], [self keyboardModifiers:event]);
  [self makeKeyWindow];
}

- (void)rightMouseUp:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.visage_window->handleMouseUp(visage::kMouseButtonRight, point.x, point.y,
                                    [self mouseButtonState], [self keyboardModifiers:event]);
}

- (void)rightMouseDragged:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.visage_window->handleMouseMove(point.x, point.y, [self mouseButtonState],
                                      [self keyboardModifiers:event]);
  [self checkRelativeMode];
}

- (void)otherMouseDown:(NSEvent*)event {
  if ([event buttonNumber] != 2)
    return;

  visage::Point point = [self eventPosition:event];
  self.mouse_down_screen_position = [self mouseScreenPosition];
  self.visage_window->handleMouseDown(visage::kMouseButtonMiddle, point.x, point.y,
                                      [self mouseButtonState], [self keyboardModifiers:event]);
  [self makeKeyWindow];
}

- (void)otherMouseUp:(NSEvent*)event {
  if ([event buttonNumber] != 2)
    return;

  visage::Point point = [self eventPosition:event];
  self.visage_window->handleMouseUp(visage::kMouseButtonMiddle, point.x, point.y,
                                    [self mouseButtonState], [self keyboardModifiers:event]);
}

- (void)otherMouseDragged:(NSEvent*)event {
  visage::Point point = [self eventPosition:event];
  self.visage_window->handleMouseMove(point.x, point.y, [self mouseButtonState],
                                      [self keyboardModifiers:event]);
  [self checkRelativeMode];
}

- (void)viewDidChangeBackingProperties {
  [super viewDidChangeBackingProperties];
  if (self.visage_window)
    self.visage_window->resetBackingScale();
}

- (void)viewWillMoveToWindow:(NSWindow*)new_window {
  [super viewWillMoveToWindow:new_window];

  if (new_window) {
    [new_window setAcceptsMouseMovedEvents:YES];
    [new_window setIgnoresMouseEvents:NO];
    [new_window makeFirstResponder:self];
    if (self.visage_window)
      self.visage_window->setParentWindow(new_window);

    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(windowOcclusionChanged:)
                                                 name:NSWindowDidChangeOcclusionStateNotification
                                               object:new_window];
  }
  else if (self.visage_window)
    self.visage_window->closeWindow();
}

- (void)windowOcclusionChanged:(NSNotification*)notification {
  NSWindow* window = notification.object;
  if (self.visage_window)
    self.visage_window->setVisible(window.occlusionState & NSWindowOcclusionStateVisible);
}

- (std::vector<std::string>)dropFileList:(id<NSDraggingInfo>)sender {
  NSPasteboard* pasteboard = [sender draggingPasteboard];
  NSArray* classes = @[[NSURL class]];
  NSDictionary* options = @{ NSPasteboardURLReadingFileURLsOnlyKey: @YES };
  NSArray* file_urls = [pasteboard readObjectsForClasses:classes options:options];

  std::vector<std::string> result;
  if (file_urls) {
    for (NSURL* file_url in file_urls)
      result.emplace_back([[file_url path] UTF8String]);
  }
  return result;
}

- (NSDragOperation)dragFiles:(id<NSDraggingInfo>)sender {
  visage::Point drag_point = [self dragPosition:sender];
  std::vector<std::string> files = [self dropFileList:sender];
  if (self.visage_window->handleFileDrag(drag_point.x, drag_point.y, files))
    return NSDragOperationCopy;
  return NSDragOperationNone;
}

- (NSDragOperation)draggingEntered:(id<NSDraggingInfo>)sender {
  return [self dragFiles:sender];
}

- (NSDragOperation)draggingUpdated:(id<NSDraggingInfo>)sender {
  return [self dragFiles:sender];
}

- (BOOL)performDragOperation:(id<NSDraggingInfo>)sender {
  visage::Point drag_point = [self dragPosition:sender];
  std::vector<std::string> files = [self dropFileList:sender];
  if (self.visage_window->handleFileDrop(drag_point.x, drag_point.y, files))
    return NSDragOperationCopy;
  return NSDragOperationNone;
}

- (BOOL)prepareForDragOperation:(id<NSDraggingInfo>)sender {
  return YES;
}

- (void)draggingExited:(id<NSDraggingInfo>)sender {
  self.visage_window->handleFileDragLeave();
}

@end

@implementation VisageAppWindowDelegate
- (void)windowWillClose:(NSNotification*)notification {
  NSWindow* ns_window = notification.object;
  NSView* view = ns_window.contentView;
  visage::WindowMac* window = visage::NativeWindowLookup::instance().findWindow((__bridge void*)view);
  if (window)
    window->closeWindow();
}

- (void)windowWillStartLiveResize:(NSNotification*)notification {
  self.resizing_vertical = false;
  self.resizing_horizontal = false;
}

- (NSSize)windowWillResize:(NSWindow*)sender toSize:(NSSize)frame_size {
  if (!self.visage_window->isFixedAspectRatio())
    return frame_size;

  NSSize current_frame = [self.window_handle frame].size;
  if (current_frame.width != frame_size.width)
    self.resizing_horizontal = true;
  if (current_frame.height != frame_size.height)
    self.resizing_vertical = true;

  visage::Point max_dimensions = visage::Point(self.visage_window->maxWindowDimensions());
  visage::Point min_dimensions = visage::Point(self.visage_window->minWindowDimensions());
  visage::Point borders = visage::windowBorderSize(self.window_handle);
  visage::Point dimensions(static_cast<float>(frame_size.width - borders.x),
                           static_cast<float>(frame_size.height - borders.y));
  float aspect_ratio = self.visage_window->aspectRatio();
  dimensions = adjustBoundsForAspectRatio(dimensions, min_dimensions, max_dimensions, aspect_ratio,
                                          self.resizing_horizontal, self.resizing_vertical);

  return NSMakeSize(dimensions.x + borders.x, dimensions.y + borders.y);
}

@end

@implementation VisageAppDelegate
- (void)applicationDidFinishLaunching:(NSNotification*)notification {
  self.window_delegate = [[VisageAppWindowDelegate alloc] init];
  self.window_delegate.visage_window = self.visage_window;
  self.window_delegate.window_handle = self.window_handle;
  [self.window_handle setDelegate:self.window_delegate];

  [NSApp activateIgnoringOtherApps:YES];
}

- (void)applicationWillTerminate:(NSNotification*)notification {
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender {
  return NO;
}

- (NSApplicationTerminateReply)applicationShouldTerminate:(NSApplication*)sender {
  [NSApp stop:nil];
  return NSTerminateCancel;
}

@end

namespace visage {
  bool WindowMac::running_event_loop_ = false;

  void WindowMac::runEventLoop() {
    running_event_loop_ = true;

    @autoreleasepool {
      NSApplication* app = [NSApplication sharedApplication];
      VisageAppDelegate* delegate = [[VisageAppDelegate alloc] init];
      delegate.visage_window = this;
      delegate.window_handle = window_handle_;
      [app setDelegate:delegate];
      [app run];
    }
  }

  void* WindowMac::initWindow() const {
    return (__bridge void*)InitialMetalLayer::layer();
  }

  void showMessageBox(std::string title, std::string message) {
    dispatch_async(dispatch_get_main_queue(), ^{
      @autoreleasepool {
        NSAlert* alert = [[NSAlert alloc] init];
        [alert setMessageText:[NSString stringWithUTF8String:title.c_str()]];
        [alert setInformativeText:[NSString stringWithUTF8String:message.c_str()]];
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
      }
    });
  }

  float defaultDpiScale() {
    NSScreen* screen = [NSScreen mainScreen];
    CGRect screen_frame = [screen frame];
    return screen.backingScaleFactor;
  }

  static IBounds computeWindowBoundsWithScale(const Dimension& x, const Dimension& y,
                                              const Dimension& w, const Dimension& h, float& scale) {
    scale = defaultDpiScale();
    NSScreen* screen = [NSScreen mainScreen];
    CGRect screen_frame = [screen frame];
    scale = screen.backingScaleFactor;
    int screen_width = screen_frame.size.width * scale;
    int screen_height = screen_frame.size.height * scale;
    int x_pos = x.computeInt(scale, screen_width, screen_height, 0);
    int y_pos = y.computeInt(scale, screen_width, screen_height, 0);

    for (NSScreen* s in [NSScreen screens]) {
      if (NSPointInRect(CGPointMake(x_pos, y_pos), [s frame])) {
        screen = s;
        break;
      }
    }

    screen_frame = [screen frame];
    scale = screen.backingScaleFactor;
    screen_width = screen_frame.size.width * scale;
    screen_height = screen_frame.size.height * scale;
    int width = w.computeInt(scale, screen_width, screen_height, 0);
    int height = h.computeInt(scale, screen_width, screen_height, 0);

    int default_x = (screen_width - width) / 2;
    int default_y = (screen_height - height) / 2;
    x_pos = x.computeInt(scale, screen_width, screen_height, default_x);
    y_pos = y.computeInt(scale, screen_width, screen_height, default_y);

    return { x_pos, screen_height - y_pos - height, width, height };
  }

  IBounds computeWindowBounds(const Dimension& x, const Dimension& y, const Dimension& w,
                              const Dimension& h) {
    float scale = 1.0f;
    return computeWindowBoundsWithScale(x, y, w, h, scale);
  }

  std::unique_ptr<Window> createWindow(const Dimension& x, const Dimension& y, const Dimension& width,
                                       const Dimension& height, Window::Decoration decoration) {
    float scale = 1.0f;
    IBounds bounds = computeWindowBoundsWithScale(x, y, width, height, scale);
    return std::make_unique<WindowMac>(bounds.x(), bounds.y(), bounds.width(), bounds.height(),
                                       scale, decoration);
  }

  void* headlessWindowHandle() {
    return (__bridge void*)InitialMetalLayer::layer();
  }

  std::unique_ptr<Window> createPluginWindow(const Dimension& width, const Dimension& height,
                                             void* parent_handle) {
    float scale = 1.0f;
    IBounds bounds = computeWindowBoundsWithScale({}, {}, width, height, scale);
    return std::make_unique<WindowMac>(bounds.width(), bounds.height(), scale, parent_handle);
  }

  WindowMac::WindowMac(int x, int y, int width, int height, float scale, Decoration decoration) :
      Window(width, height), decoration_(decoration) {
    setDpiScale(scale);
    last_content_rect_ = NSMakeRect(x / scale, y / scale, width / scale, height / scale);
    NSRect rect = last_content_rect_;

    rect.origin.x = 0;
    rect.origin.y = 0;
    view_ = [[VisageAppView alloc] initWithFrame:rect inWindow:this];
    view_delegate_ = [[VisageAppViewDelegate alloc] initWithWindow:this];
    view_.delegate = view_delegate_;
    view_.allow_quit = true;

    createWindow();
    NativeWindowLookup::instance().addWindow(this);
  }

  WindowMac::WindowMac(int width, int height, float scale, void* parent_handle) :
      Window(width, height) {
    parent_view_ = (__bridge NSView*)parent_handle;
    CGRect view_frame = CGRectMake(0.0f, 0.0f, width / scale, height / scale);

    view_ = [[VisageAppView alloc] initWithFrame:view_frame inWindow:this];
    view_delegate_ = [[VisageAppViewDelegate alloc] initWithWindow:this];
    view_.delegate = view_delegate_;
    view_.allow_quit = false;
    [parent_view_ addSubview:view_];

    NativeWindowLookup::instance().addWindow(this);
  }

  WindowMac::~WindowMac() {
    NativeWindowLookup::instance().removeWindow(this);
    hide();
    view_.visage_window = nullptr;

    if (parent_view_)
      [view_ removeFromSuperview];
  }

  void WindowMac::createWindow() {
    static const NSUInteger kNativeStyle = NSWindowStyleMaskTitled | NSWindowStyleMaskResizable |
                                           NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskClosable;
    static const NSUInteger kClientStyle = kNativeStyle | NSWindowStyleMaskFullSizeContentView;
    static const NSUInteger kPopupStyle = NSWindowStyleMaskBorderless;

    int style_mask = kNativeStyle;
    if (decoration_ == Decoration::Popup)
      style_mask = kPopupStyle;
    else if (decoration_ == Decoration::Client)
      style_mask = kClientStyle;

    NSWindow* window = [[NSWindow alloc] initWithContentRect:last_content_rect_
                                                   styleMask:style_mask
                                                     backing:NSBackingStoreBuffered
                                                       defer:NO];
    if (decoration_ == Decoration::Popup)
      [window setLevel:NSStatusWindowLevel];
    else if (decoration_ == Decoration::Client)
      window.titlebarAppearsTransparent = YES;

    window_handle_ = window;
    setDpiScale([window_handle_ backingScaleFactor]);
    int client_width = std::round(last_content_rect_.size.width * dpiScale());
    int client_height = std::round(last_content_rect_.size.height * dpiScale());
    handleResized(client_width, client_height);

    [window_handle_ setContentView:view_];
    [window_handle_ makeFirstResponder:view_];
  }

  void WindowMac::closeWindow() {
    last_content_rect_ = window_handle_.contentLayoutRect;
    NativeWindowLookup::instance().removeWindow(this);
    hide();
    window_handle_ = nullptr;

    if (running_event_loop_ && !NativeWindowLookup::instance().anyWindowOpen())
      [NSApp stop:nil];
  }

  void WindowMac::setParentWindow(NSWindow* window) {
    if (parent_view_ == nullptr)
      return;

    window_handle_ = window;
    [window_handle_ makeFirstResponder:view_];
    [NSApp activateIgnoringOtherApps:YES];
    resetBackingScale();
  }

  void WindowMac::resetBackingScale() {
    if (window_handle_)
      setDpiScale([window_handle_ backingScaleFactor]);
  }

  void WindowMac::windowContentsResized(int width, int height) {
    NSRect frame = [window_handle_ frame];
    int x = frame.origin.x;
    int y = frame.origin.y;

    float w = width / dpiScale();
    float h = height / dpiScale();
    Point borders = windowBorderSize(window_handle_);
    frame.size.width = w + borders.x;
    frame.size.height = h + borders.y;

    [view_ setFrameSize:CGSizeMake(w, h)];
    if (parent_view_ == nullptr) {
      [window_handle_ setFrame:NSMakeRect(x, y, frame.size.width, frame.size.height)
                       display:YES
                       animate:true];
    }
  }

  void WindowMac::show() {
    if (parent_view_ == nullptr) {
      if (window_handle_ == nullptr)
        createWindow();

      if (isPopup())
        [window_handle_ orderFront:nil];
      else
        [window_handle_ makeKeyAndOrderFront:nil];
    }
    notifyShow();
  }

  void WindowMac::showMaximized() {
    if (window_handle_ == nullptr)
      createWindow();

    [window_handle_ zoom:nil];
    [window_handle_ makeKeyAndOrderFront:nil];
    notifyShow();
  }

  void WindowMac::hide() {
    if (window_handle_ && parent_view_ == nullptr) {
      [window_handle_ orderOut:nil];
      notifyHide();
    }
  }

  bool WindowMac::isShowing() const {
    return window_handle_ && [window_handle_ isVisible];
  }

  void WindowMac::setWindowTitle(const std::string& title) {
    [window_handle_ setTitle:[NSString stringWithUTF8String:title.c_str()]];
  }

  IPoint WindowMac::maxWindowDimensions() const {
    Point borders = windowBorderSize(window_handle_);

    NSScreen* screen = [window_handle_ screen];
    NSRect visible_frame = [screen visibleFrame];

    int display_width = visible_frame.size.width - borders.x;
    int display_height = visible_frame.size.height - borders.y;
    float aspect_ratio = aspectRatio();

    return { std::min<int>(display_width, display_height * aspect_ratio),
             std::min<int>(display_height, display_width / aspect_ratio) };
  }

  IPoint WindowMac::minWindowDimensions() const {
    float minimum_scale = minimumWindowScale();
    NSScreen* screen = [window_handle_ screen];
    NSRect visible_frame = [screen visibleFrame];

    int min_display_width = minimum_scale * visible_frame.size.width;
    int min_display_height = minimum_scale * visible_frame.size.height;
    float aspect_ratio = aspectRatio();

    return { std::max<int>(min_display_width, min_display_height * aspect_ratio),
             std::max<int>(min_display_height, min_display_width / aspect_ratio) };
  }

  void WindowMac::handleNativeResize(int width, int height) {
    handleResized(width, height);
  }
}

#endif
