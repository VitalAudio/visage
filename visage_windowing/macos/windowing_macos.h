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
@end

@interface AppView : MTKView <NSDraggingDestination>
@property(nonatomic) visage::WindowMac* visage_window;
@property(strong) DraggingSource* drag_source_;
- (instancetype)initWithFrame:(NSRect)frame_rect inWindow:(visage::WindowMac*)window;
- (void)drawView;
- (void)resize:(CGSize)size;
@property bool allow_quit;
@end

@interface AppWindowDelegate : NSObject <NSWindowDelegate>
@property(nonatomic) visage::WindowMac* visage_window;
@property(nonatomic, retain) NSWindow* window_handle;
@end

@interface AppDelegate : NSObject <NSApplicationDelegate>
@property(nonatomic, retain) NSWindow* window_handle;
@property(nonatomic, assign) AppWindowDelegate* window_delegate;
@property visage::WindowMac* visage_window;
@end

namespace visage {
  class WindowMac : public Window {
  public:
    WindowMac(int x, int y, int width, int height);
    WindowMac(int width, int height, void* parent_handle);

    ~WindowMac() override;

    void* nativeHandle() const override { return view_; }
    void* getInitWindow() const override;

    void setNativeWindowHandle(NSWindow* window);
    AppView* getView() { return view_; }

    void runEventLoop() override;
    void windowContentsResized(int width, int height) override;
    void show() override;
    void hide() override;
    void setWindowTitle(const std::string& title) override;
    Point getMaxWindowDimensions() const override;
    Point getMinWindowDimensions() const override;

    void handleNativeResize(int width, int height);

  private:
    NSWindow* window_handle_ = nullptr;
    NSView* parent_view_ = nullptr;
    AppView* view_ = nullptr;
    AppViewDelegate* view_delegate_ = nullptr;
  };

}

#endif
