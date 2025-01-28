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

#if VISAGE_LINUX && VISAGE_WAYLAND
#include "windowing_wayland.h"

#include "visage_utils/thread_utils.h"

#include <cstring>
#include <sstream>
#include <xkbcommon/xkbcommon.h>

namespace visage {
  static std::string _clipboard_text;

  struct MonitorInfo {
    int mm_width = 0;
    int mm_height = 0;
    int width = 0;
    int height = 0;
    int pixel_scale = 1;
    int refresh_rate = 60000;
  };

  class WaylandHandler {
  public:
    static WaylandHandler& instance() {
      static WaylandHandler callbacks;
      return callbacks;
    }

    static void registerWindow(WindowWayland* window) {
      instance().windows_[window->surface()] = window;
      instance().top_levels_[window->topLevel()] = window;
    }

    static void unregisterWindow(WindowWayland* window) {
      instance().windows_.erase(window->surface());
      instance().top_levels_.erase(window->topLevel());

      if (window == instance().current_window_)
        instance().current_window_ = nullptr;
    }

    static MonitorInfo defaultMonitor() { return instance().monitor_info_.begin()->second; }
    static wl_display* display() { return instance().display_; }
    static wl_compositor* compositor() { return instance().compositor_; }
    static xdg_wm_base* windowManager() { return instance().xdg_wm_base_; }
    static auto decorationManager() { return instance().decoration_manager_; }

    static int fd() { return wl_display_get_fd(instance().display_); }

    static void handleConfigure(void* data, struct xdg_surface* xdg_surface, uint32_t serial) {
      xdg_surface_ack_configure(xdg_surface, serial);
    }

    static void handleFrameCallback(void* data, struct wl_callback* callback, uint32_t callback_data) {
      static struct wl_callback_listener callback_handler = { handleFrameCallback };

      struct wl_surface* surface = static_cast<struct wl_surface*>(data);

      if (instance().windows_.count(surface)) {
        double current_time = (time::microseconds() - instance().start_microseconds_) / 1000000.0;
        instance().windows_[surface]->drawCallback(current_time);
      }

      struct wl_callback* new_callback = wl_surface_frame(surface);
      wl_callback_add_listener(new_callback, &callback_handler, surface);
      wl_surface_commit(surface);
      wl_callback_destroy(callback);
    }

    static void handleToplevelConfigure(void* data, struct xdg_toplevel* xdg_toplevel,
                                        int32_t width, int32_t height, struct wl_array* states) {
      VISAGE_LOG("CONFIGURE");
      VISAGE_LOG(width);
      VISAGE_LOG(height);
    }

    static void handleClose(void* data, struct xdg_toplevel* xdg_toplevel) { VISAGE_LOG("test"); }

    static void handleConfigureBounds(void* data, struct xdg_toplevel* xdg_toplevel, int32_t width,
                                      int32_t height) {
      VISAGE_LOG(width);
      VISAGE_LOG(height);
    }

    static void handleWmCapabilities(void* data, struct xdg_toplevel* xdg_toplevel,
                                     struct wl_array* capabilities) {
      VISAGE_LOG("test2");
    }

    static void handleDecorationConfigure(void* data, struct zxdg_toplevel_decoration_v1* decoration,
                                          uint32_t mode) { }

  private:
    static void handlePointerMotion(void* data, struct wl_pointer* pointer, uint32_t time,
                                    wl_fixed_t surface_x, wl_fixed_t surface_y) { }

    static void handlePointerEnter(void* data, struct wl_pointer* pointer, uint32_t serial,
                                   struct wl_surface* surface, wl_fixed_t surface_x, wl_fixed_t surface_y) {
      if (instance().windows_.count(surface)) {
        instance().current_window_ = instance().windows_[surface];
        instance().current_serial_ = serial;
      }
    }

    static void handlePointerLeave(void* data, struct wl_pointer* pointer, uint32_t serial,
                                   struct wl_surface* surface) { }

    static void handlePointerButton(void* data, struct wl_pointer* pointer, uint32_t serial,
                                    uint32_t time, uint32_t button, uint32_t state) {
      if (instance().current_window_) {
        if (state == WL_POINTER_BUTTON_STATE_PRESSED)
          xdg_toplevel_move(instance().current_window_->topLevel(), instance().seat_, serial);
      }
    }

    static void handlePointerAxis(void* data, struct wl_pointer* pointer, uint32_t time,
                                  uint32_t axis, wl_fixed_t value) {
      double scroll_value = wl_fixed_to_double(value);
    }

    static void handlePointerFrame(void* data, struct wl_pointer* wl_pointer) { }

    static void handlePointerAxisSource(void* data, struct wl_pointer* wl_pointer, uint32_t axis_source) {
    }

    static void handlePointerAxisStop(void* data, struct wl_pointer* wl_pointer, uint32_t time,
                                      uint32_t axis) { }

    static void handlePointerAxisDiscrete(void* data, struct wl_pointer* wl_pointer, uint32_t axis,
                                          int32_t discrete) { }

    static void handlePointerAxisValue120(void* data, struct wl_pointer* wl_pointer, uint32_t axis,
                                          int32_t value120) { }

    static void handlePointerAxisRelativeDirection(void* data, struct wl_pointer* wl_pointer,
                                                   uint32_t axis, uint32_t direction) { }

    static void handleGeometry(void* data, struct wl_output* output, int32_t x, int32_t y,
                               int32_t physical_width, int32_t physical_height, int32_t subpixel,
                               const char* make, const char* model, int32_t transform) {
      WaylandHandler* self = static_cast<WaylandHandler*>(data);
      self->monitor_info_[output].mm_width = physical_width;
      self->monitor_info_[output].mm_height = physical_height;
    }

    static void handleMode(void* data, struct wl_output* output, uint32_t flags, int32_t width,
                           int32_t height, int32_t refresh_rate) {
      WaylandHandler* self = static_cast<WaylandHandler*>(data);
      self->monitor_info_[output].width = width;
      self->monitor_info_[output].height = height;
      self->monitor_info_[output].refresh_rate = refresh_rate;
    }

    static void handleDone(void* data, struct wl_output* output) { }

    static void handleScale(void* data, struct wl_output* output, int32_t factor) {
      WaylandHandler* self = static_cast<WaylandHandler*>(data);
      self->monitor_info_[output].pixel_scale = factor;
    }

    static void handleName(void* data, struct wl_output* wl_output, const char* name) { }
    static void handleDescription(void* data, struct wl_output* wl_output, const char* description) { }

    static void handleSeat(void* data, struct wl_seat* seat, uint32_t capabilities) {
      static const struct wl_pointer_listener pointer_listener = { handlePointerEnter,
                                                                   handlePointerLeave,
                                                                   handlePointerMotion,
                                                                   handlePointerButton,
                                                                   handlePointerAxis,
                                                                   handlePointerFrame,
                                                                   handlePointerAxisSource,
                                                                   handlePointerAxisStop,
                                                                   handlePointerAxisDiscrete,
                                                                   handlePointerAxisValue120,
                                                                   handlePointerAxisRelativeDirection };
      WaylandHandler* self = static_cast<WaylandHandler*>(data);
      self->seat_ = seat;

      if (capabilities & WL_SEAT_CAPABILITY_POINTER) {
        struct wl_pointer* pointer = wl_seat_get_pointer(seat);
        wl_pointer_add_listener(pointer, &pointer_listener, nullptr);
        wl_display_roundtrip(self->display_);
      }
    }

    static void handleSeatName(void* data, struct wl_seat* wl_seat, const char* name) { }

    static void handlePing(void* data, xdg_wm_base* wm_base, uint32_t serial) {
      xdg_wm_base_pong(wm_base, serial);
    }

    static void handleRegistryAdd(void* data, wl_registry* registry, uint32_t id,
                                  const char* interface, uint32_t version) {
      static constexpr struct wl_output_listener output_listener = {
        handleGeometry, handleMode, handleDone, handleScale, handleName, handleDescription
      };

      static const struct wl_seat_listener seat_listener = { handleSeat, handleSeatName };

      WaylandHandler* self = static_cast<WaylandHandler*>(data);
      if (std::strcmp(interface, "wl_compositor") == 0) {
        self->compositor_ = static_cast<wl_compositor*>(wl_registry_bind(registry, id, &wl_compositor_interface,
                                                                         version));
      }
      else if (std::strcmp(interface, "wl_output") == 0) {
        wl_output* output = static_cast<wl_output*>(wl_registry_bind(registry, id,
                                                                     &wl_output_interface, version));
        wl_output_add_listener(output, &output_listener, self);
        wl_display_roundtrip(self->display_);
      }
      else if (std::string(interface) == "wl_seat") {
        struct wl_seat** seat = static_cast<struct wl_seat**>(data);
        *seat = static_cast<wl_seat*>(wl_registry_bind(registry, id, &wl_seat_interface, version));
        wl_seat_add_listener(*seat, &seat_listener, self);
        wl_display_roundtrip(self->display_);
      }
      else if (strcmp(interface, "zxdg_decoration_manager_v1") == 0) {
        auto decoration = wl_registry_bind(registry, id, &zxdg_decoration_manager_v1_interface, 1);
        instance().decoration_manager_ = static_cast<zxdg_decoration_manager_v1*>(decoration);
      }
      else if (std::strcmp(interface, xdg_wm_base_interface.name) == 0) {
        self->xdg_wm_base_ = static_cast<xdg_wm_base*>(wl_registry_bind(registry, id, &xdg_wm_base_interface,
                                                                        version));

        static const xdg_wm_base_listener wm_base_listener = { handlePing };
        xdg_wm_base_add_listener(self->xdg_wm_base_, &wm_base_listener, self);
        wl_display_roundtrip(self->display_);
      }
    }

    static void handleRegistryRemove(void* data, wl_registry* registry, uint32_t id) { }

    WaylandHandler() {
      static const wl_registry_listener registry_listener = { handleRegistryAdd, handleRegistryRemove };

      display_ = wl_display_connect(nullptr);
      registry_ = wl_display_get_registry(display_);
      wl_registry_add_listener(registry_, &registry_listener, this);
      wl_display_roundtrip(display_);
      start_microseconds_ = time::microseconds();
    }

    ~WaylandHandler() {
      xdg_wm_base_destroy(xdg_wm_base_);
      wl_compositor_destroy(compositor_);
      wl_registry_destroy(registry_);
      wl_display_disconnect(display_);
    }

    std::map<wl_output*, MonitorInfo> monitor_info_;
    std::map<wl_surface*, WindowWayland*> windows_;
    std::map<xdg_toplevel*, WindowWayland*> top_levels_;

    wl_display* display_ = nullptr;
    wl_registry* registry_ = nullptr;
    wl_compositor* compositor_ = nullptr;
    wl_seat* seat_ = nullptr;
    xdg_wm_base* xdg_wm_base_ = nullptr;
    zxdg_decoration_manager_v1* decoration_manager_ = nullptr;
    WindowWayland* current_window_ = nullptr;
    uint32_t current_serial_ = 0;
    unsigned long long start_microseconds_ = 0;
  };

  std::string readClipboardText() {
    return "";
  }

  void setClipboardText(const std::string& text) {
    _clipboard_text = text;
  }

  void setCursorStyle(MouseCursor style) { }

  void setCursorVisible(bool visible) { }

  Point cursorPosition() {
    return { 0, 0 };
  }

  void setCursorPosition(Point window_position) { }

  void setCursorScreenPosition(Point window_position) { }

  Point cursorScreenPosition() {
    return { 0, 0 };
  }

  float windowPixelScale() {
    return 1.0f;
  }

  void showMessageBox(std::string title, std::string message) { }

  Bounds computeWindowBounds(const Dimension& x, const Dimension& y, const Dimension& width,
                             const Dimension& height) {
    return { 0, 0, 0, 0 };
  }

  std::unique_ptr<Window> createWindow(const Dimension& x, const Dimension& y,
                                       const Dimension& width, const Dimension& height, bool popup) {
    MonitorInfo monitor = WaylandHandler::defaultMonitor();
    int window_w = width.compute(monitor.pixel_scale, monitor.width, monitor.height);
    int window_h = height.compute(monitor.pixel_scale, monitor.width, monitor.height);
    int default_x = (monitor.width - window_w) / 2;
    int default_y = (monitor.height - window_h) / 2;
    int window_x = x.computeWithDefault(monitor.pixel_scale, monitor.width, monitor.height, default_x);
    int window_y = y.computeWithDefault(monitor.pixel_scale, monitor.width, monitor.height, default_y);
    return std::make_unique<WindowWayland>(window_x, window_y, window_w, window_h, popup);
  }

  std::unique_ptr<Window> createPluginWindow(const Dimension& width, const Dimension& height,
                                             void* parent_handle) {
    return nullptr;
  }

  WindowWayland* WindowWayland::last_active_window_ = nullptr;

  WindowWayland::WindowWayland(int x, int y, int width, int height, bool popup) :
      Window(width, height) {
    static wl_callback_listener callback_handler = { WaylandHandler::handleFrameCallback };
    static xdg_toplevel_listener toplevel_listener = { WaylandHandler::handleToplevelConfigure,
                                                       WaylandHandler::handleClose,
                                                       WaylandHandler::handleConfigureBounds,
                                                       WaylandHandler::handleWmCapabilities };
    static zxdg_toplevel_decoration_v1_listener decoration_listener = { WaylandHandler::handleDecorationConfigure };

    MonitorInfo monitor = WaylandHandler::defaultMonitor();

    setPixelScale(monitor.pixel_scale);
    setDpiScale(monitor.pixel_scale);
    surface_ = wl_compositor_create_surface(WaylandHandler::compositor());
    wl_surface_set_buffer_scale(surface_, monitor.pixel_scale);
    xdg_surface_ = xdg_wm_base_get_xdg_surface(WaylandHandler::windowManager(), surface_);
    xdg_top_level_ = xdg_surface_get_toplevel(xdg_surface_);

    xdg_toplevel_add_listener(xdg_top_level_, &toplevel_listener, nullptr);

    xdg_toplevel_set_title(xdg_top_level_, "Wayland Window Example");
    static const xdg_surface_listener surface_listener = { WaylandHandler::handleConfigure };
    xdg_surface_add_listener(xdg_surface_, &surface_listener, nullptr);

    wl_display_roundtrip(WaylandHandler::display());
    wl_surface_commit(surface_);

    if (WaylandHandler::decorationManager()) {
      decoration_ = zxdg_decoration_manager_v1_get_toplevel_decoration(WaylandHandler::decorationManager(),
                                                                       xdg_top_level_);
      zxdg_toplevel_decoration_v1_add_listener(decoration_, &decoration_listener, nullptr);
      zxdg_toplevel_decoration_v1_set_mode(decoration_, ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE);
    }

    WaylandHandler::registerWindow(this);

    struct wl_callback* callback = wl_surface_frame(surface_);
    wl_callback_add_listener(callback, &callback_handler, surface_);
  }

  WindowWayland::WindowWayland(int width, int height, void* parent_handle) :
      Window(width, height) { }

  WindowWayland::~WindowWayland() {
    WaylandHandler::unregisterWindow(this);

    xdg_toplevel_destroy(xdg_top_level_);
    xdg_surface_destroy(xdg_surface_);
    wl_surface_destroy(surface_);
  }

  void WindowWayland::setFixedAspectRatio(bool fixed) { }

  visage::Point WindowWayland::retrieveWindowDimensions() {
    return { 0, 0 };
  }

  int WindowWayland::mouseButtonState() const {
    return 0;
  }

  int WindowWayland::modifierState() const {
    return 0;
  }

  static KeyCode translateKeyCode(xkb_keysym_t keysym) {
    switch (keysym) {
    case XKB_KEY_a: return KeyCode::A;
    case XKB_KEY_b: return KeyCode::B;
    case XKB_KEY_c: return KeyCode::C;
    case XKB_KEY_d: return KeyCode::D;
    case XKB_KEY_e: return KeyCode::E;
    case XKB_KEY_f: return KeyCode::F;
    case XKB_KEY_g: return KeyCode::G;
    case XKB_KEY_h: return KeyCode::H;
    case XKB_KEY_i: return KeyCode::I;
    case XKB_KEY_j: return KeyCode::J;
    case XKB_KEY_k: return KeyCode::K;
    case XKB_KEY_l: return KeyCode::L;
    case XKB_KEY_m: return KeyCode::M;
    case XKB_KEY_n: return KeyCode::N;
    case XKB_KEY_o: return KeyCode::O;
    case XKB_KEY_p: return KeyCode::P;
    case XKB_KEY_q: return KeyCode::Q;
    case XKB_KEY_r: return KeyCode::R;
    case XKB_KEY_s: return KeyCode::S;
    case XKB_KEY_t: return KeyCode::T;
    case XKB_KEY_u: return KeyCode::U;
    case XKB_KEY_v: return KeyCode::V;
    case XKB_KEY_w: return KeyCode::W;
    case XKB_KEY_x: return KeyCode::X;
    case XKB_KEY_y: return KeyCode::Y;
    case XKB_KEY_z: return KeyCode::Z;
    case XKB_KEY_A: return KeyCode::A;
    case XKB_KEY_B: return KeyCode::B;
    case XKB_KEY_C: return KeyCode::C;
    case XKB_KEY_D: return KeyCode::D;
    case XKB_KEY_E: return KeyCode::E;
    case XKB_KEY_F: return KeyCode::F;
    case XKB_KEY_G: return KeyCode::G;
    case XKB_KEY_H: return KeyCode::H;
    case XKB_KEY_I: return KeyCode::I;
    case XKB_KEY_J: return KeyCode::J;
    case XKB_KEY_K: return KeyCode::K;
    case XKB_KEY_L: return KeyCode::L;
    case XKB_KEY_M: return KeyCode::M;
    case XKB_KEY_N: return KeyCode::N;
    case XKB_KEY_O: return KeyCode::O;
    case XKB_KEY_P: return KeyCode::P;
    case XKB_KEY_Q: return KeyCode::Q;
    case XKB_KEY_R: return KeyCode::R;
    case XKB_KEY_S: return KeyCode::S;
    case XKB_KEY_T: return KeyCode::T;
    case XKB_KEY_U: return KeyCode::U;
    case XKB_KEY_V: return KeyCode::V;
    case XKB_KEY_W: return KeyCode::W;
    case XKB_KEY_X: return KeyCode::X;
    case XKB_KEY_Y: return KeyCode::Y;
    case XKB_KEY_Z: return KeyCode::Z;
    case XKB_KEY_1: return KeyCode::Number1;
    case XKB_KEY_2: return KeyCode::Number2;
    case XKB_KEY_3: return KeyCode::Number3;
    case XKB_KEY_4: return KeyCode::Number4;
    case XKB_KEY_5: return KeyCode::Number5;
    case XKB_KEY_6: return KeyCode::Number6;
    case XKB_KEY_7: return KeyCode::Number7;
    case XKB_KEY_8: return KeyCode::Number8;
    case XKB_KEY_9: return KeyCode::Number9;
    case XKB_KEY_0: return KeyCode::Number0;
    case XKB_KEY_Return: return KeyCode::Return;
    case XKB_KEY_Escape: return KeyCode::Escape;
    case XKB_KEY_BackSpace: return KeyCode::Backspace;
    case XKB_KEY_Tab: return KeyCode::Tab;
    case XKB_KEY_space: return KeyCode::Space;
    case XKB_KEY_minus: return KeyCode::Minus;
    case XKB_KEY_equal: return KeyCode::Equals;
    case XKB_KEY_bracketleft: return KeyCode::LeftBracket;
    case XKB_KEY_bracketright: return KeyCode::RightBracket;
    case XKB_KEY_backslash: return KeyCode::Backslash;
    case XKB_KEY_semicolon: return KeyCode::Semicolon;
    case XKB_KEY_apostrophe: return KeyCode::Apostrophe;
    case XKB_KEY_grave: return KeyCode::Grave;
    case XKB_KEY_comma: return KeyCode::Comma;
    case XKB_KEY_period: return KeyCode::Period;
    case XKB_KEY_slash: return KeyCode::Slash;
    case XKB_KEY_Caps_Lock: return KeyCode::CapsLock;
    case XKB_KEY_F1: return KeyCode::F1;
    case XKB_KEY_F2: return KeyCode::F2;
    case XKB_KEY_F3: return KeyCode::F3;
    case XKB_KEY_F4: return KeyCode::F4;
    case XKB_KEY_F5: return KeyCode::F5;
    case XKB_KEY_F6: return KeyCode::F6;
    case XKB_KEY_F7: return KeyCode::F7;
    case XKB_KEY_F8: return KeyCode::F8;
    case XKB_KEY_F9: return KeyCode::F9;
    case XKB_KEY_F10: return KeyCode::F10;
    case XKB_KEY_F11: return KeyCode::F11;
    case XKB_KEY_F12: return KeyCode::F12;
    case XKB_KEY_Print: return KeyCode::PrintScreen;
    case XKB_KEY_Scroll_Lock: return KeyCode::ScrollLock;
    case XKB_KEY_Pause: return KeyCode::Pause;
    case XKB_KEY_Insert: return KeyCode::Insert;
    case XKB_KEY_Home: return KeyCode::Home;
    case XKB_KEY_Page_Up: return KeyCode::PageUp;
    case XKB_KEY_Delete: return KeyCode::Delete;
    case XKB_KEY_End: return KeyCode::End;
    case XKB_KEY_Page_Down: return KeyCode::PageDown;
    case XKB_KEY_Right: return KeyCode::Right;
    case XKB_KEY_Left: return KeyCode::Left;
    case XKB_KEY_Down: return KeyCode::Down;
    case XKB_KEY_Up: return KeyCode::Up;
    case XKB_KEY_Num_Lock: return KeyCode::NumLock;
    case XKB_KEY_KP_Divide: return KeyCode::KPDivide;
    case XKB_KEY_KP_Multiply: return KeyCode::KPMultiply;
    case XKB_KEY_KP_Subtract: return KeyCode::KPMinus;
    case XKB_KEY_KP_Add: return KeyCode::KPPlus;
    case XKB_KEY_KP_Enter: return KeyCode::KPEnter;
    case XKB_KEY_KP_1: return KeyCode::KP1;
    case XKB_KEY_KP_2: return KeyCode::KP2;
    case XKB_KEY_KP_3: return KeyCode::KP3;
    case XKB_KEY_KP_4: return KeyCode::KP4;
    case XKB_KEY_KP_5: return KeyCode::KP5;
    case XKB_KEY_KP_6: return KeyCode::KP6;
    case XKB_KEY_KP_7: return KeyCode::KP7;
    case XKB_KEY_KP_8: return KeyCode::KP8;
    case XKB_KEY_KP_9: return KeyCode::KP9;
    case XKB_KEY_KP_0: return KeyCode::KP0;
    case XKB_KEY_KP_Decimal: return KeyCode::KPPeriod;
    default: return KeyCode::Unknown;
    }
  }

  void WindowWayland::processPluginFdEvents() { }

  void* WindowWayland::globalDisplay() const {
    return WaylandHandler::display();
  }

  int WindowWayland::posixFd() const {
    return WaylandHandler::fd();
  }

  void WindowWayland::runEventLoop() {
    while (wl_display_dispatch(WaylandHandler::display()) != -1) {
    }
  }

  void WindowWayland::windowContentsResized(int width, int height) { }

  void WindowWayland::show() { }

  void WindowWayland::showMaximized() { }

  void WindowWayland::hide() { }

  void WindowWayland::setWindowTitle(const std::string& title) { }

  Point WindowWayland::maxWindowDimensions() const {
    return { 0, 0 };
  }

  Point WindowWayland::minWindowDimensions() const {
    return { 0, 0 };
  }
}
#endif
