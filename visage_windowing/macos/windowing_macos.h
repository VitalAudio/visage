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
