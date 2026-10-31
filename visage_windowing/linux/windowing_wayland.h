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

#if VISAGE_LINUX && VISAGE_WAYLAND
#include "visage_windowing/windowing.h"

#include <memory>
#include <string>
#include <vector>

struct wl_callback;
struct wl_output;
struct wl_subsurface;
struct wl_surface;
struct wp_fractional_scale_v1;
struct wp_viewport;
struct xdg_surface;
struct xdg_toplevel;
struct zwp_locked_pointer_v1;
struct zwp_relative_pointer_v1;
struct zxdg_toplevel_decoration_v1;

namespace visage {
  class WaylandConnection;

  class WindowWayland : public Window {
  public:
    static constexpr int kMinWidth = 80;
    static constexpr int kMinHeight = 80;
    static constexpr int kClientResizeBorder = 8;
    static constexpr long long kStalledFrameUs = 100000;
    static constexpr int kShadowLeft = 20;
    static constexpr int kShadowTop = 20;
    static constexpr int kShadowRight = 20;
    static constexpr int kShadowBottom = 20;
    static constexpr int kTitleBarHeight = 36;
    static constexpr int kTitleButtonSize = 22;
    static constexpr int kTitleButtonMargin = 8;
    static constexpr int kTitleButtonSpacing = 11;
    static constexpr int kTitleButtonTop = 7;
    static constexpr int kFrameResizeBorder = 10;
    static constexpr int kFrameCornerRadius = 15;
    static constexpr int kTitleDragThreshold = 8;

    enum class FrameArea {
      Outside,
      Content,
      TitleBar,
      Button,
      Resize
    };

    static WindowWayland* lastActiveWindow() { return last_active_window_; }
    static void setLastActiveWindow(WindowWayland* window) { last_active_window_ = window; }

    WindowWayland(int width, int height, float scale, Decoration decoration);
    ~WindowWayland() override;

    void runEventLoop() override;
    void* nativeHandle() const override { return surface_; }
    void* graphicsHandle() const override { return content_surface_; }
    void* globalDisplay() const override;
    bool waylandDisplay() const override { return true; }
    void* frameDecorationHandle() const override { return frame_surface_; }
    void frameDecorationChanged() override;
    int posixFd() const override;

    void windowContentsResized(int width, int height) override;
    void show() override;
    void showMaximized() override;
    void hide() override;
    void close() override;
    bool isShowing() const override { return shown_; }
    bool isClosed() const override;
    void setWindowTitle(const std::string& title) override;
    IPoint maxWindowDimensions() const override;
    void framePresented() override { frame_presented_ = true; }

    wl_surface* surface() const { return surface_; }
    wl_output* output() const { return output_; }
    bool closeRequested() const { return close_requested_; }
    void clearCloseRequest() { close_requested_ = false; }
    IPoint lastPointerPosition() const { return last_pointer_position_; }

    long long nextTickMicroseconds() const;
    void tick();
    void frameDone();
    void frameSurfaceDone();
    void outputEntered(wl_output* output);
    void outputLeft(wl_output* output);
    void outputChanged(wl_output* output);
    void setIntegerScale(int scale);
    void setFractionalScale(float scale);
    void toplevelConfigure(int width, int height, bool maximized, bool constrained, bool suspended,
                           bool activated);
    void surfaceConfigure(uint32_t serial);
    void requestClose() { close_requested_ = true; }
    void requestParentCommit() { parent_commit_pending_ = true; }

    void pointerEnter(double x, double y);
    void pointerLeave();
    void pointerMotion(double x, double y);
    void pointerRelativeMotion(double dx, double dy);
    void pointerButton(MouseButton button, bool pressed);
    void pointerWheel(float delta_x, float delta_y);
    IPoint contentPoint(IPoint point) const;
    void setPointerHint(IPoint native_position);
    void setPointerPosition(IPoint native_position);
    void unlockPointer();

    void finishDragDropSource();

  private:
    static WindowWayland* last_active_window_;

    int resizeOperationForPosition(int x, int y) const;
    bool framed() const { return frame_subsurface_ != nullptr; }
    int shadowLeft() const { return framed() && !constrained_ ? kShadowLeft : 0; }
    int shadowTop() const { return framed() && !constrained_ ? kShadowTop : 0; }
    int shadowRight() const { return framed() && !constrained_ ? kShadowRight : 0; }
    int shadowBottom() const { return framed() && !constrained_ ? kShadowBottom : 0; }
    int titleBarHeight() const { return framed() ? kTitleBarHeight : 0; }
    int contentOffsetX() const;
    int contentOffsetY() const;
    FrameArea frameArea(IPoint point, int* resize_operation, FrameButton* button = nullptr) const;
    IBounds frameButtonBounds(FrameButton button) const;
    void updateFrameHover(IPoint point);
    void framePointerButton(MouseButton button, bool pressed);
    void drawFrameDecoration();
    void createRole(bool maximized);
    void waitForConfigure();
    void commitParent();
    void waitForPresentedFrame();
    void drawConfigured();
    void destroyRole();
    void setMinimumSize();
    void applyLogicalSize();
    void applySize(int width, int height, bool constrained);
    void changeScale(float scale);
    void lockPointer();
    void updateCursorForPosition(int x, int y);

    WaylandConnection* wayland_ = nullptr;
    Decoration decoration_ = Decoration::Native;
    wl_surface* surface_ = nullptr;
    wp_viewport* viewport_ = nullptr;
    wl_surface* content_surface_ = nullptr;
    wp_viewport* content_viewport_ = nullptr;
    wl_subsurface* subsurface_ = nullptr;
    wl_surface* frame_surface_ = nullptr;
    wp_viewport* frame_viewport_ = nullptr;
    wl_subsurface* frame_subsurface_ = nullptr;
    wl_callback* frame_surface_callback_ = nullptr;
    bool frame_dirty_ = false;
    bool active_ = false;
    bool constrained_ = false;
    FrameButton hovered_button_ = FrameButton::NoButton;
    FrameButton pressed_button_ = FrameButton::NoButton;
    bool pointer_in_content_ = false;
    bool content_grab_ = false;
    IPoint frame_pointer_;
    long long last_title_click_ms_ = 0;
    bool title_pressed_ = false;
    IPoint title_press_position_;
    bool parent_buffer_attached_ = false;
    bool parent_commit_pending_ = false;
    wp_fractional_scale_v1* fractional_scale_ = nullptr;
    xdg_surface* xdg_surface_ = nullptr;
    xdg_toplevel* xdg_toplevel_ = nullptr;
    zxdg_toplevel_decoration_v1* decoration_object_ = nullptr;
    wl_callback* frame_callback_ = nullptr;
    zwp_locked_pointer_v1* locked_pointer_ = nullptr;
    zwp_relative_pointer_v1* relative_pointer_ = nullptr;
    wl_output* output_ = nullptr;
    std::vector<wl_output*> entered_outputs_;

    std::string title_ = VISAGE_APPLICATION_NAME;
    int logical_width_ = 0;
    int logical_height_ = 0;
    int pending_width_ = 0;
    int pending_height_ = 0;
    bool pending_maximized_ = false;
    bool pending_constrained_ = false;
    bool pending_suspended_ = false;
    bool pending_activated_ = false;
    bool maximized_ = false;
    bool configured_ = false;
    bool waiting_for_configure_ = false;
    bool shown_ = false;
    bool close_requested_ = false;
    bool fractional_ = false;
    bool preferred_scale_ = false;

    bool frame_presented_ = false;
    bool waiting_for_frame_ = false;
    bool frame_in_flight_ = false;
    long long frame_wait_start_us_ = 0;
    long long next_tick_us_ = 0;
    long long start_draw_microseconds_ = 0;

    int hover_window_operation_ = 0;
    IPoint mouse_down_position_;
    IPoint last_pointer_position_;
    float relative_remainder_x_ = 0.0f;
    float relative_remainder_y_ = 0.0f;
    bool drag_drop_source_active_ = false;
    std::vector<std::string> drag_drop_files_;
  };

  bool waylandAvailable();
  std::string readClipboardTextWayland();
  void setClipboardTextWayland(const std::string& text);
  void setCursorStyleWayland(MouseCursor style);
  void setCursorVisibleWayland(bool visible);
  Point cursorPositionWayland();
  void setCursorPositionWayland(Point window_position);
  void setCursorScreenPositionWayland(Point window_position);
  void showMessageBoxWayland(std::string title, std::string message);
  float defaultDpiScaleWayland();
  IBounds computeWindowBoundsWayland(const Dimension& x, const Dimension& y, const Dimension& width,
                                     const Dimension& height);
  std::unique_ptr<Window> createWindowWayland(const Dimension& x, const Dimension& y,
                                              const Dimension& width, const Dimension& height,
                                              Window::Decoration decoration);
  void closeApplicationWayland();
  int displayFpsWayland();
}

#endif
