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

@interface DraggingSource : NSObject <NSDraggingSource>
@end

@interface AppViewDelegate : NSObject <MTKViewDelegate>
@property(nonatomic) visage::WindowMac* visage_window;
@property long long start_microseconds;
@end

@interface AppView : MTKView <NSDraggingDestination>
@property(nonatomic) visage::WindowMac* visage_window;
@property(strong) DraggingSource* drag_source;
@property bool allow_quit;
@property NSPoint mouse_down_screen_position;

- (instancetype)initWithFrame:(NSRect)frame_rect inWindow:(visage::WindowMac*)window;
@end

@interface AppWindowDelegate : NSObject <NSWindowDelegate>
@property(nonatomic) visage::WindowMac* visage_window;
@property(nonatomic, retain) NSWindow* window_handle;
@property bool resizing_horizontal;
@property bool resizing_vertical;
@end

@interface AppDelegate : NSObject <NSApplicationDelegate>
@property(nonatomic, retain) NSWindow* window_handle;
@property(nonatomic, strong) AppWindowDelegate* window_delegate;
@property visage::WindowMac* visage_window;
@end

namespace visage {
  class WindowMac : public Window {
  public:
    WindowMac(int x, int y, int width, int height, bool popup);
    WindowMac(int width, int height, void* parent_handle);

    ~WindowMac() override;

    void* nativeHandle() const override { return (__bridge void*)view_; }
    void* initWindow() const override;

    void runEventLoop() override;
    void windowContentsResized(int width, int height) override;
    void show() override;
    void showMaximized() override;
    void hide() override;
    void setWindowTitle(const std::string& title) override;
    Point maxWindowDimensions() const override;
    Point minWindowDimensions() const override;

    void handleNativeResize(int width, int height);
    bool isPopup() const { return popup_; }

  private:
    NSWindow* window_handle_ = nullptr;
    NSView* parent_view_ = nullptr;
    AppView* view_ = nullptr;
    AppViewDelegate* view_delegate_ = nullptr;
    bool popup_ = false;
  };

}

#endif
