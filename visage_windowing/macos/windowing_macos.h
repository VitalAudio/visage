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

#pragma once

#if VISAGE_MAC
#include "windowing.h"

#include <Carbon/Carbon.h>
#include <Cocoa/Cocoa.h>
#include <MetalKit/MetalKit.h>

namespace visage {
  class WindowMac;
}

@interface VisageDraggingSource : NSObject <NSDraggingSource>
@end

@interface VisageAppViewDelegate : NSObject <MTKViewDelegate>
@property(nonatomic) visage::WindowMac* visage_window;
@property long long start_microseconds;
@end

@interface VisageAppView : MTKView <NSDraggingDestination>
@property(nonatomic) visage::WindowMac* visage_window;
@property(strong) VisageDraggingSource* drag_source;
@property bool allow_quit;
@property NSPoint mouse_down_screen_position;

- (instancetype)initWithFrame:(NSRect)frame_rect inWindow:(visage::WindowMac*)window;
@end

@interface VisageAppWindowDelegate : NSObject <NSWindowDelegate>
@property(nonatomic) visage::WindowMac* visage_window;
@property(nonatomic, retain) NSWindow* window_handle;
@property bool resizing_horizontal;
@property bool resizing_vertical;
@end

@interface VisageAppDelegate : NSObject <NSApplicationDelegate>
@property(nonatomic, retain) NSWindow* window_handle;
@property(nonatomic, strong) VisageAppWindowDelegate* window_delegate;
@property visage::WindowMac* visage_window;
@end

namespace visage {
  class WindowMac : public Window {
  public:
    WindowMac(int x, int y, int width, int height, float scale, Decoration decoration);
    WindowMac(int width, int height, float scale, void* parent_handle);

    ~WindowMac() override;

    void createWindow();
    void closeWindow();
    void setParentWindow(NSWindow* window);
    void resetBackingScale();

    void* nativeHandle() const override { return (__bridge void*)view_; }
    void* initWindow() const override;

    void runEventLoop() override;
    void windowContentsResized(int width, int height) override;
    void show() override;
    void showMaximized() override;
    void hide() final;
    void close() final;
    bool isShowing() const override;

    void setWindowTitle(const std::string& title) override;
    IPoint maxWindowDimensions() const override;
    IPoint minWindowDimensions() const override;

    void handleNativeResize(int width, int height);
    bool isPopup() const { return decoration_ == Decoration::Popup; }

  private:
    static bool running_event_loop_;
    NSWindow* window_handle_ = nullptr;
    NSView* parent_view_ = nullptr;
    VisageAppView* view_ = nullptr;
    VisageAppViewDelegate* view_delegate_ = nullptr;
    NSRect last_content_rect_ {};
    Decoration decoration_ = Decoration::Native;
  };
}

#endif
