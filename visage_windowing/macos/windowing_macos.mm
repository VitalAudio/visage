/* Copyright Vital Audio, LLC
 *
 * visage is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * visage is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with visage.  If not, see <http://www.gnu.org/licenses/>.
 */

#if VISAGE_MAC
#include "windowing_macos.h"

#include "visage_utils/file_system.h"
#include "visage_utils/time_utils.h"

namespace visage {
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
    return { (int)cursor.x, (int)cursor.y };
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
    int x = window_bounds.origin.x + window_position.x;
    int y = window_bounds.origin.y + window_position.y;
    setCursorScreenPosition({ x, y });
  }

  Point windowBorderSize(NSWindow* window_handle) {
    if (window_handle == nullptr)
      return { 0, 0 };
    NSRect frame = [window_handle frame];
    NSRect content_rect = [window_handle contentRectForFrameRect:frame];
    return { (int)std::round(frame.size.width - content_rect.size.width),
             (int)std::round(frame.size.height - content_rect.size.height) };
  }

  float windowPixelScale() {
    if (NSScreen.mainScreen == nullptr)
      return 1.0f;

    return [NSScreen.mainScreen backingScaleFactor];
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

@implementation DraggingSource
- (NSDragOperation)draggingSession:(NSDraggingSession*)session
    sourceOperationMaskForDraggingContext:(NSDraggingContext)context {
  return NSDragOperationCopy;
}
@end

@implementation AppViewDelegate
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

  [self drawWindow];
}
@end

@implementation AppView
- (instancetype)initWithFrame:(NSRect)frame_rect inWindow:(visage::WindowMac*)window {
  self = [super initWithFrame:frame_rect];
  self.visage_window = window;
  self.device = MTLCreateSystemDefaultDevice();
  self.clearColor = MTLClearColorMake(0.1, 0.1, 0.1, 1.0);
  self.enableSetNeedsDisplay = NO;
  self.framebufferOnly = YES;
  self.layerContentsPlacement = NSViewLayerContentsPlacementTopLeft;

  [self registerForDraggedTypes:@[NSPasteboardTypeFileURL]];
  self.drag_source = [[DraggingSource alloc] init];

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

- (void)paste:(id)sender {
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

- (visage::Point)eventPosition:(NSEvent*)event {
  NSPoint location = [event locationInWindow];
  NSPoint view_location = [self convertPoint:location fromView:nil];
  CGFloat view_height = self.frame.size.height;
  return visage::Point(view_location.x, view_height - view_location.y);
}

- (visage::Point)dragPosition:(id<NSDraggingInfo>)sender {
  NSPoint drag_point = [self convertPoint:[sender draggingLocation] fromView:nil];
  CGFloat view_height = self.frame.size.height;
  return visage::Point(drag_point.x, view_height - drag_point.y);
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

- (void)scrollWheel:(NSEvent*)event {
  static constexpr float kPreciseScrollingScale = 0.02f;
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
  [self.window makeKeyWindow];
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
  [self.window makeKeyWindow];
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
  [self.window makeKeyWindow];
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

- (void)viewWillMoveToWindow:(NSWindow*)new_window {
  [super viewWillMoveToWindow:new_window];

  if (new_window) {
    [new_window setAcceptsMouseMovedEvents:YES];
    [new_window setIgnoresMouseEvents:NO];
    [new_window makeFirstResponder:self];
    self.visage_window->setPixelScale([new_window backingScaleFactor]);
    self.visage_window->setDpiScale([new_window backingScaleFactor]);

    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(windowOcclusionChanged:)
                                                 name:NSWindowDidChangeOcclusionStateNotification
                                               object:new_window];
  }
}

- (void)windowOcclusionChanged:(NSNotification*)notification {
  NSWindow* window = notification.object;
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

@implementation AppWindowDelegate
- (void)windowWillClose:(NSNotification*)notification {
  [NSApp stop:nil];
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

  visage::Point max_dimensions = self.visage_window->maxWindowDimensions();
  visage::Point min_dimensions = self.visage_window->minWindowDimensions();
  visage::Point borders = visage::windowBorderSize(self.window_handle);
  visage::Point dimensions = visage::Point(std::round(frame_size.width - borders.x),
                                           std::round(frame_size.height - borders.y));
  float aspect_ratio = self.visage_window->aspectRatio();
  dimensions = adjustBoundsForAspectRatio(dimensions, min_dimensions, max_dimensions, aspect_ratio,
                                          self.resizing_horizontal, self.resizing_vertical);

  return NSMakeSize(dimensions.x + borders.x, dimensions.y + borders.y);
}

@end

@implementation AppDelegate
- (void)applicationDidFinishLaunching:(NSNotification*)notification {
  self.window_delegate = [[AppWindowDelegate alloc] init];
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
  void WindowMac::runEventLoop() {
    @autoreleasepool {
      NSApplication* app = [NSApplication sharedApplication];
      AppDelegate* delegate = [[AppDelegate alloc] init];
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

  std::unique_ptr<Window> createWindow(int x, int y, int width, int height) {
    return std::make_unique<WindowMac>(x, y, width, height);
  }

  std::unique_ptr<Window> createPluginWindow(int width, int height, void* parent_handle) {
    return std::make_unique<WindowMac>(width, height, parent_handle);
  }

  Bounds scaledWindowBounds(float aspect_ratio, float display_scale, int x, int y) {
    NSScreen* screen = [NSScreen mainScreen];
    if (x != Window::kNotSet && y != Window::kNotSet) {
      for (NSScreen* s in [NSScreen screens]) {
        if (NSPointInRect(CGPointMake(x, y), [s frame])) {
          screen = s;
          break;
        }
      }
    }

    CGRect screen_frame = [screen frame];
    int scaled_width = screen_frame.size.width * display_scale;
    int scaled_height = screen_frame.size.height * display_scale;
    int width = std::min<int>(scaled_width, scaled_height * aspect_ratio);
    int height = std::min<int>(scaled_height, scaled_width / aspect_ratio);
    if (x == Window::kNotSet)
      x = (screen_frame.size.width - width) / 2;
    if (y == Window::kNotSet)
      y = (screen_frame.size.height - height) / 2;

    return { x, y, width, height };
  }

  WindowMac::WindowMac(int x, int y, int width, int height) : Window(width, height) {
    static const NSUInteger kWindowStyleMask = NSWindowStyleMaskTitled | NSWindowStyleMaskResizable |
                                               NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskClosable;

    NSRect content_rect = NSMakeRect(x, y, width, height);
    NSWindow* window = [[NSWindow alloc] initWithContentRect:content_rect
                                                   styleMask:kWindowStyleMask
                                                     backing:NSBackingStoreBuffered
                                                       defer:NO];
    window_handle_ = window;
    content_rect.origin.x = 0;
    content_rect.origin.y = 0;
    view_ = [[AppView alloc] initWithFrame:content_rect inWindow:this];
    view_delegate_ = [[AppViewDelegate alloc] initWithWindow:this];
    view_.delegate = view_delegate_;
    view_.allow_quit = true;

    setPixelScale([window_handle_ backingScaleFactor]);
    setDpiScale([window_handle_ backingScaleFactor]);
    int client_width = std::round(content_rect.size.width * pixelScale());
    int client_height = std::round(content_rect.size.height * pixelScale());
    handleResized(client_width, client_height);

    [window_handle_ setContentView:view_];
    [window_handle_ makeFirstResponder:view_];
  }

  WindowMac::WindowMac(int width, int height, void* parent_handle) : Window(width, height) {
    parent_view_ = (__bridge NSView*)parent_handle;

    if (parent_view_.window) {
      setPixelScale([parent_view_.window backingScaleFactor]);
      setDpiScale([parent_view_.window backingScaleFactor]);
      window_handle_ = parent_view_.window;
    }

    CGRect view_frame = CGRectMake(0.0f, 0.0f, width / pixelScale(), height / pixelScale());

    view_ = [[AppView alloc] initWithFrame:view_frame inWindow:this];
    view_delegate_ = [[AppViewDelegate alloc] initWithWindow:this];
    view_.delegate = view_delegate_;
    view_.allow_quit = false;
    [parent_view_ addSubview:view_];

    if (window_handle_)
      [window_handle_ makeFirstResponder:view_];

    [NSApp activateIgnoringOtherApps:YES];
  }

  WindowMac::~WindowMac() = default;

  void WindowMac::windowContentsResized(int width, int height) {
    NSRect frame = [window_handle_ frame];
    int x = frame.origin.x;
    int y = frame.origin.y;

    Point borders = windowBorderSize(window_handle_);
    frame.size.width = width + borders.x;
    frame.size.height = height + borders.y;

    bool animate = true;  //TODO !isFixedAspectRatio();
    [window_handle_ setFrame:NSMakeRect(x, y, frame.size.width, frame.size.height)
                     display:YES
                     animate:animate];
    [view_ setFrameSize:CGSizeMake(width, height)];
  }

  void WindowMac::show() {
    if (parent_view_ && parent_view_.window) {
      [parent_view_.window makeKeyWindow];
      [parent_view_.window makeKeyAndOrderFront:nil];
    }
    else {
      [window_handle_ makeKeyWindow];
      [window_handle_ makeKeyAndOrderFront:nil];
    }
  }

  void WindowMac::hide() {
    [window_handle_ orderOut:nil];
  }

  void WindowMac::setWindowTitle(const std::string& title) {
    [window_handle_ setTitle:[NSString stringWithUTF8String:title.c_str()]];
  }

  Point WindowMac::maxWindowDimensions() const {
    Point borders = windowBorderSize(window_handle_);

    NSScreen* screen = [window_handle_ screen];
    NSRect visible_frame = [screen visibleFrame];

    int display_width = visible_frame.size.width - borders.x;
    int display_height = visible_frame.size.height - borders.y;
    float aspect_ratio = aspectRatio();

    return { std::min<int>(display_width, display_height * aspect_ratio),
             std::min<int>(display_height, display_width / aspect_ratio) };
  }

  Point WindowMac::minWindowDimensions() const {
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
