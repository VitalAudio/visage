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

#include "visage_utils/string_utils.h"
#include "visage_utils/time_utils.h"
#include "windowing_x11.h"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <clocale>
#include <cmath>
#include <csignal>
#include <cstring>
#include <ctime>
#include <cursor-shape-v1-client-protocol.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <fractional-scale-v1-client-protocol.h>
#include <linux/input-event-codes.h>
#include <map>
#include <pointer-constraints-unstable-v1-client-protocol.h>
#include <poll.h>
#include <pthread.h>
#include <relative-pointer-unstable-v1-client-protocol.h>
#include <sstream>
#include <sys/mman.h>
#include <unistd.h>
#include <viewporter-client-protocol.h>
#include <wayland-client.h>
#include <wayland-cursor.h>
#include <xdg-decoration-unstable-v1-client-protocol.h>
#include <xdg-shell-client-protocol.h>
#include <xkbcommon/xkbcommon-compose.h>
#include <xkbcommon/xkbcommon.h>

struct libdecor;
struct libdecor_state;

namespace visage {
  static std::string _wayland_clipboard_text;

  static constexpr int kWaylandScaleDenominator = 120;
  static constexpr int kDataReadTimeoutMs = 250;
  static constexpr int kMaxPollMs = 100;
  static constexpr int kDefaultCursorSize = 24;
  static constexpr double kScrollPixelsPerNotch = 10.0;
  static constexpr unsigned char kFirstPrintable = 0x20;
  static constexpr unsigned char kDelete = 0x7f;
  static constexpr const char* kUriListType = "text/uri-list";
  static constexpr const char* kTextTypes[] = { "text/plain;charset=utf-8", "UTF8_STRING",
                                                "text/plain", "TEXT", "STRING" };

  struct LibDecorInterface {
    void (*error)(libdecor* context, int error, const char* message);
    void (*reserved[10])();
  };

  struct LibDecorFrameInterface {
    void (*configure)(libdecor_frame* frame, libdecor_configuration* configuration, void* user_data);
    void (*close)(libdecor_frame* frame, void* user_data);
    void (*commit)(libdecor_frame* frame, void* user_data);
    void (*dismiss_popup)(libdecor_frame* frame, const char* seat_name, void* user_data);
    void (*reserved[10])();
  };

  struct LibDecor {
    static constexpr int kStateMaximized = 1 << 1;
    static constexpr int kStateFullscreen = 1 << 2;
    static constexpr int kStateTiled = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
    static constexpr int kStateSuspended = 1 << 7;

    template<typename T>
    bool load(T& function, const char* name) {
      function = reinterpret_cast<T>(dlsym(library, name));
      return function != nullptr;
    }

    bool load() {
      library = dlopen("libdecor-0.so.0", RTLD_NOW | RTLD_LOCAL);
      if (library == nullptr)
        return false;

      return load(create, "libdecor_new") && load(unref, "libdecor_unref") &&
             load(dispatch, "libdecor_dispatch") && load(decorate, "libdecor_decorate") &&
             load(frame_unref, "libdecor_frame_unref") && load(set_title, "libdecor_frame_set_title") &&
             load(set_app_id, "libdecor_frame_set_app_id") &&
             load(set_min_content_size, "libdecor_frame_set_min_content_size") &&
             load(set_maximized, "libdecor_frame_set_maximized") &&
             load(commit, "libdecor_frame_commit") && load(map, "libdecor_frame_map") &&
             load(state_new, "libdecor_state_new") && load(state_free, "libdecor_state_free") &&
             load(content_size, "libdecor_configuration_get_content_size") &&
             load(window_state, "libdecor_configuration_get_window_state");
    }

    void* library = nullptr;
    libdecor* context = nullptr;
    libdecor* (*create)(wl_display*, LibDecorInterface*) = nullptr;
    void (*unref)(libdecor*) = nullptr;
    int (*dispatch)(libdecor*, int) = nullptr;
    libdecor_frame* (*decorate)(libdecor*, wl_surface*, LibDecorFrameInterface*, void*) = nullptr;
    void (*frame_unref)(libdecor_frame*) = nullptr;
    void (*set_title)(libdecor_frame*, const char*) = nullptr;
    void (*set_app_id)(libdecor_frame*, const char*) = nullptr;
    void (*set_min_content_size)(libdecor_frame*, int, int) = nullptr;
    void (*set_maximized)(libdecor_frame*) = nullptr;
    void (*commit)(libdecor_frame*, libdecor_state*, libdecor_configuration*) = nullptr;
    void (*map)(libdecor_frame*) = nullptr;
    libdecor_state* (*state_new)(int, int) = nullptr;
    void (*state_free)(libdecor_state*) = nullptr;
    bool (*content_size)(libdecor_configuration*, libdecor_frame*, int*, int*) = nullptr;
    bool (*window_state)(libdecor_configuration*, int*) = nullptr;
  };

  class WaylandConnection;

  struct WaylandOutput {
    WaylandConnection* connection = nullptr;
    wl_output* output = nullptr;
    uint32_t name = 0;
    int width = 0;
    int height = 0;
    int refresh_mhz = 0;
    int scale = 1;
  };

  static bool writeAll(int fd, const std::string& data) {
    sigset_t pipe_set, previous_set;
    sigemptyset(&pipe_set);
    sigaddset(&pipe_set, SIGPIPE);
    pthread_sigmask(SIG_BLOCK, &pipe_set, &previous_set);

    size_t written = 0;
    while (written < data.size()) {
      ssize_t result = write(fd, data.data() + written, data.size() - written);
      if (result < 0 && errno == EINTR)
        continue;
      if (result <= 0)
        break;
      written += result;
    }

    timespec no_wait = {};
    while (sigtimedwait(&pipe_set, nullptr, &no_wait) > 0)
      continue;
    pthread_sigmask(SIG_SETMASK, &previous_set, nullptr);
    return written == data.size();
  }

  static std::vector<std::string> uriListFiles(const std::string& uri_list) {
    std::vector<std::string> files;
    std::stringstream stream(uri_list);
    std::string line;
    while (std::getline(stream, line)) {
      std::string trimmed = String(line).trim().toUtf8();
      if (!trimmed.empty() && trimmed[0] != '#')
        files.push_back(filePath(trimmed));
    }
    return files;
  }

  class WaylandConnection {
  public:
    static WaylandConnection* globalInstance() {
      static WaylandConnection* connection = new WaylandConnection();
      return connection;
    }

    WaylandConnection() {
      display_ = wl_display_connect(nullptr);
      if (display_ == nullptr)
        return;

      registry_ = wl_display_get_registry(display_);
      wl_registry_add_listener(registry_, &kRegistryListener, this);
      wl_display_roundtrip(display_);
      if (compositor_ == nullptr || wm_base_ == nullptr || shm_ == nullptr) {
        wl_display_disconnect(display_);
        display_ = nullptr;
        return;
      }

      wl_display_roundtrip(display_);
      xkb_context_ = xkb_context_new(XKB_CONTEXT_NO_FLAGS);
    }

    WaylandConnection(const WaylandConnection&) = delete;

    wl_display* display() const { return display_; }
    wl_compositor* compositor() const { return compositor_; }
    wp_viewporter* viewporter() const { return viewporter_; }
    wp_fractional_scale_manager_v1* fractionalScaleManager() const {
      return fractional_scale_manager_;
    }
    xdg_wm_base* wmBase() const { return wm_base_; }
    zxdg_decoration_manager_v1* decorationManager() const { return decoration_manager_; }
    wl_seat* seat() const { return seat_; }
    uint32_t compositorVersion() const { return compositor_version_; }

    void addWindow(WindowWayland* window) { windows_.push_back(window); }

    void removeWindow(WindowWayland* window) {
      windows_.erase(std::remove(windows_.begin(), windows_.end(), window), windows_.end());
      if (pointer_focus_ == window)
        pointer_focus_ = nullptr;
      if (keyboard_focus_ == window) {
        keyboard_focus_ = nullptr;
        repeat_key_ = 0;
      }
      if (drag_drop_window_ == window)
        drag_drop_window_ = nullptr;
      if (drag_drop_source_window_ == window)
        drag_drop_source_window_ = nullptr;
      if (WindowWayland::lastActiveWindow() == window)
        WindowWayland::setLastActiveWindow(nullptr);
    }

    void closeWindow(WindowWayland* window) {
      if (std::find(open_windows_.begin(), open_windows_.end(), window) != open_windows_.end())
        open_windows_.erase(std::find(open_windows_.begin(), open_windows_.end(), window));
    }

    void openWindow(WindowWayland* window) {
      if (std::find(open_windows_.begin(), open_windows_.end(), window) == open_windows_.end())
        open_windows_.push_back(window);
    }

    bool isOpen(const WindowWayland* window) const {
      return std::find(open_windows_.begin(), open_windows_.end(), window) != open_windows_.end();
    }

    bool anyWindowShowing() const {
      return std::any_of(open_windows_.begin(), open_windows_.end(),
                         [](WindowWayland* window) { return window->isShowing(); });
    }

    void closeAll() const {
      auto windows = open_windows_;
      for (WindowWayland* window : windows)
        window->close();
    }

    WindowWayland* findWindow(wl_surface* surface) const {
      if (surface == nullptr)
        return nullptr;
      for (WindowWayland* window : windows_) {
        if (window->surface() == surface)
          return window;
      }
      return nullptr;
    }

    WaylandOutput* findOutput(wl_output* output) const {
      for (const auto& wayland_output : outputs_) {
        if (wayland_output->output == output)
          return wayland_output.get();
      }
      return nullptr;
    }

    const WaylandOutput* activeOutput() const {
      WindowWayland* window = WindowWayland::lastActiveWindow();
      if (window && window->output()) {
        if (const WaylandOutput* output = findOutput(window->output()))
          return output;
      }
      return outputs_.empty() ? nullptr : outputs_.front().get();
    }

    float defaultScale() const {
      if (last_scale_ > 0.0f)
        return last_scale_;
      const WaylandOutput* output = activeOutput();
      return output ? output->scale : 1.0f;
    }

    void setLastScale(float scale) { last_scale_ = scale; }

    LibDecor* libDecor() {
      if (libdecor_tried_)
        return libdecor_.context ? &libdecor_ : nullptr;

      libdecor_tried_ = true;
      if (!libdecor_.load())
        return nullptr;
      libdecor_.context = libdecor_.create(display_, &kLibDecorInterface);
      return libdecor_.context ? &libdecor_ : nullptr;
    }

    bool dispatch(int timeout_ms) {
      while (wl_display_prepare_read(display_) != 0)
        wl_display_dispatch_pending(display_);

      if (wl_display_flush(display_) < 0 && errno != EAGAIN) {
        wl_display_cancel_read(display_);
        return false;
      }

      pollfd fd = { wl_display_get_fd(display_), POLLIN, 0 };
      int result = poll(&fd, 1, timeout_ms);
      if (result > 0 && (fd.revents & POLLIN)) {
        if (wl_display_read_events(display_) < 0)
          return false;
      }
      else
        wl_display_cancel_read(display_);

      wl_display_dispatch_pending(display_);
      if (libdecor_.context)
        libdecor_.dispatch(libdecor_.context, 0);
      return wl_display_get_error(display_) == 0;
    }

    long long nextDeadline() const {
      long long deadline = LLONG_MAX;
      for (WindowWayland* window : windows_)
        deadline = std::min(deadline, window->nextTickMicroseconds());
      if (repeat_key_)
        deadline = std::min(deadline, next_repeat_us_);
      return deadline;
    }

    void runTimers() {
      long long now = time::microseconds();
      if (repeat_key_ && now >= next_repeat_us_) {
        next_repeat_us_ = now + 1000000 / std::max(1, repeat_rate_);
        handleKey(repeat_key_, true, true);
      }

      auto windows = windows_;
      for (WindowWayland* window : windows) {
        if (std::find(windows_.begin(), windows_.end(), window) != windows_.end() &&
            time::microseconds() >= window->nextTickMicroseconds())
          window->tick();
      }
    }

    bool processCloseRequests() {
      bool closed = false;
      auto windows = windows_;
      for (WindowWayland* window : windows) {
        if (std::find(windows_.begin(), windows_.end(), window) == windows_.end() ||
            !window->closeRequested())
          continue;

        window->clearCloseRequest();
        if (window->handleCloseRequested()) {
          closeWindow(window);
          window->hide();
          closed = true;
        }
      }
      return closed;
    }

    int modifierState() const {
      if (xkb_state_ == nullptr)
        return 0;

      int result = 0;
      if (xkb_state_mod_name_is_active(xkb_state_, XKB_MOD_NAME_SHIFT, XKB_STATE_MODS_EFFECTIVE) > 0)
        result |= kModifierShift;
      if (xkb_state_mod_name_is_active(xkb_state_, XKB_MOD_NAME_CTRL, XKB_STATE_MODS_EFFECTIVE) > 0)
        result |= kModifierRegCtrl;
      if (xkb_state_mod_name_is_active(xkb_state_, XKB_MOD_NAME_ALT, XKB_STATE_MODS_EFFECTIVE) > 0)
        result |= kModifierAlt;
      if (xkb_state_mod_name_is_active(xkb_state_, XKB_MOD_NAME_LOGO, XKB_STATE_MODS_EFFECTIVE) > 0)
        result |= kModifierMeta;
      return result;
    }

    int buttonState() const { return button_state_; }
    uint32_t inputSerial() const { return input_serial_; }

    void setCursor(MouseCursor style) {
      cursor_style_ = style;
      applyCursor();
    }

    void startMove(xdg_toplevel* toplevel) {
      if (seat_ && toplevel)
        xdg_toplevel_move(toplevel, seat_, button_serial_);
    }

    void startResize(xdg_toplevel* toplevel, uint32_t edges) {
      if (seat_ && toplevel)
        xdg_toplevel_resize(toplevel, seat_, button_serial_, edges);
    }

    zwp_locked_pointer_v1* lockPointer(wl_surface* surface) {
      if (pointer_constraints_ == nullptr || relative_pointer_manager_ == nullptr || pointer_ == nullptr)
        return nullptr;
      return zwp_pointer_constraints_v1_lock_pointer(pointer_constraints_, surface, pointer_, nullptr,
                                                     ZWP_POINTER_CONSTRAINTS_V1_LIFETIME_PERSISTENT);
    }

    zwp_relative_pointer_v1* relativePointer(WindowWayland* window) {
      zwp_relative_pointer_v1* relative_pointer =
          zwp_relative_pointer_manager_v1_get_relative_pointer(relative_pointer_manager_, pointer_);
      zwp_relative_pointer_v1_add_listener(relative_pointer, &kRelativePointerListener, window);
      return relative_pointer;
    }

    void startDragDrop(WindowWayland* window, const std::vector<std::string>& files) {
      if (data_device_ == nullptr)
        return;

      drag_drop_source_files_ = files;
      drag_drop_source_window_ = window;
      wl_data_source* source = wl_data_device_manager_create_data_source(data_device_manager_);
      wl_data_source_add_listener(source, &kDragSourceListener, this);
      wl_data_source_offer(source, kUriListType);
      if (data_device_manager_version_ >= 3)
        wl_data_source_set_actions(source, WL_DATA_DEVICE_MANAGER_DND_ACTION_COPY);
      wl_data_device_start_drag(data_device_, source, window->surface(), nullptr, button_serial_);
    }

    std::string readClipboard() {
      if (selection_source_)
        return _wayland_clipboard_text;
      if (selection_offer_ == nullptr)
        return {};

      const std::vector<std::string>& types = offer_types_[selection_offer_];
      for (const char* text_type : kTextTypes) {
        if (std::find(types.begin(), types.end(), text_type) == types.end())
          continue;

        std::string text;
        if (!receive(selection_offer_, text_type, text))
          return {};
        if (!text.empty() && text.back() == 0)
          text.pop_back();
        return std::strcmp(text_type, "STRING") == 0 ? latin1ToUtf8(text) : text;
      }
      return {};
    }

    void setClipboard(const std::string& text) {
      _wayland_clipboard_text = text;
      if (data_device_ == nullptr)
        return;

      wl_data_source* source = wl_data_device_manager_create_data_source(data_device_manager_);
      wl_data_source_add_listener(source, &kSelectionSourceListener, this);
      for (const char* text_type : kTextTypes)
        wl_data_source_offer(source, text_type);
      wl_data_device_set_selection(data_device_, source, input_serial_);
      if (selection_source_)
        wl_data_source_destroy(selection_source_);
      selection_source_ = source;
      wl_display_flush(display_);
    }

  private:
    static const wl_registry_listener kRegistryListener;
    static const wl_output_listener kOutputListener;
    static const wl_seat_listener kSeatListener;
    static const wl_pointer_listener kPointerListener;
    static const wl_keyboard_listener kKeyboardListener;
    static const xdg_wm_base_listener kWmBaseListener;
    static const wl_data_device_listener kDataDeviceListener;
    static const wl_data_offer_listener kDataOfferListener;
    static const wl_data_source_listener kSelectionSourceListener;
    static const wl_data_source_listener kDragSourceListener;
    static const zwp_relative_pointer_v1_listener kRelativePointerListener;
    static LibDecorInterface kLibDecorInterface;

    void addGlobal(wl_registry* registry, uint32_t name, const char* interface, uint32_t version) {
      auto bind = [&](const wl_interface* wayland_interface, uint32_t max_version) {
        return wl_registry_bind(registry, name, wayland_interface, std::min(version, max_version));
      };

      if (std::strcmp(interface, wl_compositor_interface.name) == 0) {
        compositor_version_ = std::min(version, 6u);
        compositor_ = static_cast<wl_compositor*>(bind(&wl_compositor_interface, 6));
      }
      else if (std::strcmp(interface, wl_shm_interface.name) == 0)
        shm_ = static_cast<wl_shm*>(bind(&wl_shm_interface, 1));
      else if (std::strcmp(interface, xdg_wm_base_interface.name) == 0) {
        wm_base_ = static_cast<xdg_wm_base*>(bind(&xdg_wm_base_interface, 6));
        xdg_wm_base_add_listener(wm_base_, &kWmBaseListener, this);
      }
      else if (std::strcmp(interface, zxdg_decoration_manager_v1_interface.name) == 0)
        decoration_manager_ = static_cast<zxdg_decoration_manager_v1*>(bind(&zxdg_decoration_manager_v1_interface,
                                                                            1));
      else if (std::strcmp(interface, wp_viewporter_interface.name) == 0)
        viewporter_ = static_cast<wp_viewporter*>(bind(&wp_viewporter_interface, 1));
      else if (std::strcmp(interface, wp_fractional_scale_manager_v1_interface.name) == 0)
        fractional_scale_manager_ = static_cast<wp_fractional_scale_manager_v1*>(bind(&wp_fractional_scale_manager_v1_interface,
                                                                                      1));
      else if (std::strcmp(interface, wp_cursor_shape_manager_v1_interface.name) == 0)
        cursor_shape_manager_ = static_cast<wp_cursor_shape_manager_v1*>(bind(&wp_cursor_shape_manager_v1_interface,
                                                                              1));
      else if (std::strcmp(interface, zwp_pointer_constraints_v1_interface.name) == 0)
        pointer_constraints_ = static_cast<zwp_pointer_constraints_v1*>(bind(&zwp_pointer_constraints_v1_interface,
                                                                             1));
      else if (std::strcmp(interface, zwp_relative_pointer_manager_v1_interface.name) == 0)
        relative_pointer_manager_ = static_cast<zwp_relative_pointer_manager_v1*>(bind(&zwp_relative_pointer_manager_v1_interface,
                                                                                       1));
      else if (std::strcmp(interface, wl_data_device_manager_interface.name) == 0) {
        data_device_manager_version_ = std::min(version, 3u);
        data_device_manager_ = static_cast<wl_data_device_manager*>(bind(&wl_data_device_manager_interface,
                                                                         3));
        createDataDevice();
      }
      else if (std::strcmp(interface, wl_seat_interface.name) == 0 && seat_ == nullptr) {
        seat_ = static_cast<wl_seat*>(bind(&wl_seat_interface, 5));
        wl_seat_add_listener(seat_, &kSeatListener, this);
        createDataDevice();
      }
      else if (std::strcmp(interface, wl_output_interface.name) == 0) {
        auto output = std::make_unique<WaylandOutput>();
        output->name = name;
        output->connection = this;
        output->output = static_cast<wl_output*>(bind(&wl_output_interface, 2));
        wl_output_add_listener(output->output, &kOutputListener, output.get());
        outputs_.push_back(std::move(output));
      }
    }

    void removeGlobal(uint32_t name) {
      for (auto it = outputs_.begin(); it != outputs_.end(); ++it) {
        if ((*it)->name != name)
          continue;

        for (WindowWayland* window : windows_)
          window->outputLeft((*it)->output);
        wl_output_destroy((*it)->output);
        outputs_.erase(it);
        return;
      }
    }

    void createDataDevice() {
      if (data_device_ == nullptr && seat_ && data_device_manager_) {
        data_device_ = wl_data_device_manager_get_data_device(data_device_manager_, seat_);
        wl_data_device_add_listener(data_device_, &kDataDeviceListener, this);
      }
    }

    void outputChanged(WaylandOutput* output) {
      for (WindowWayland* window : windows_)
        window->outputChanged(output->output);
    }

    void seatCapabilities(uint32_t capabilities) {
      bool has_pointer = capabilities & WL_SEAT_CAPABILITY_POINTER;
      if (has_pointer && pointer_ == nullptr) {
        pointer_ = wl_seat_get_pointer(seat_);
        wl_pointer_add_listener(pointer_, &kPointerListener, this);
        if (cursor_shape_manager_)
          cursor_shape_device_ = wp_cursor_shape_manager_v1_get_pointer(cursor_shape_manager_, pointer_);
      }
      else if (!has_pointer && pointer_) {
        if (cursor_shape_device_)
          wp_cursor_shape_device_v1_destroy(cursor_shape_device_);
        cursor_shape_device_ = nullptr;
        wl_pointer_release(pointer_);
        pointer_ = nullptr;
        pointer_focus_ = nullptr;
      }

      bool has_keyboard = capabilities & WL_SEAT_CAPABILITY_KEYBOARD;
      if (has_keyboard && keyboard_ == nullptr) {
        keyboard_ = wl_seat_get_keyboard(seat_);
        wl_keyboard_add_listener(keyboard_, &kKeyboardListener, this);
      }
      else if (!has_keyboard && keyboard_) {
        wl_keyboard_release(keyboard_);
        keyboard_ = nullptr;
        keyboard_focus_ = nullptr;
        repeat_key_ = 0;
      }
    }

    IPoint nativePoint(WindowWayland* window, wl_fixed_t x, wl_fixed_t y) const {
      float scale = window->dpiScale();
      return { static_cast<int>(std::floor(wl_fixed_to_double(x) * scale)),
               static_cast<int>(std::floor(wl_fixed_to_double(y) * scale)) };
    }

    void pointerEnter(uint32_t serial, wl_surface* surface, wl_fixed_t x, wl_fixed_t y) {
      enter_serial_ = serial;
      pointer_focus_ = findWindow(surface);
      if (pointer_focus_ == nullptr)
        return;

      WindowWayland::setLastActiveWindow(pointer_focus_);
      applyCursor();
      IPoint point = nativePoint(pointer_focus_, x, y);
      pointer_focus_->pointerEnter(point.x, point.y);
    }

    void pointerLeave(wl_surface* surface) {
      WindowWayland* window = pointer_focus_;
      pointer_focus_ = nullptr;
      if (window && window->surface() == surface)
        window->pointerLeave();
    }

    void pointerMotion(wl_fixed_t x, wl_fixed_t y) {
      if (pointer_focus_ == nullptr)
        return;

      IPoint point = nativePoint(pointer_focus_, x, y);
      pointer_focus_->pointerMotion(point.x, point.y);
    }

    void pointerButton(uint32_t serial, uint32_t button, uint32_t state) {
      input_serial_ = serial;
      button_serial_ = serial;
      MouseButton mouse_button = kMouseButtonNone;
      if (button == BTN_LEFT)
        mouse_button = kMouseButtonLeft;
      else if (button == BTN_MIDDLE)
        mouse_button = kMouseButtonMiddle;
      else if (button == BTN_RIGHT)
        mouse_button = kMouseButtonRight;
      if (mouse_button == kMouseButtonNone)
        return;

      bool pressed = state == WL_POINTER_BUTTON_STATE_PRESSED;
      if (pressed)
        button_state_ |= mouse_button;
      else
        button_state_ &= ~mouse_button;

      if (pointer_focus_)
        pointer_focus_->pointerButton(serial, mouse_button, pressed);
    }

    void pointerAxis(uint32_t axis, wl_fixed_t value) {
      if (axis == WL_POINTER_AXIS_VERTICAL_SCROLL)
        scroll_y_ -= wl_fixed_to_double(value) / kScrollPixelsPerNotch;
      else
        scroll_x_ += wl_fixed_to_double(value) / kScrollPixelsPerNotch;
    }

    void pointerAxisDiscrete(uint32_t axis, int32_t discrete) {
      has_discrete_ = true;
      if (axis == WL_POINTER_AXIS_VERTICAL_SCROLL)
        discrete_y_ -= discrete;
      else
        discrete_x_ += discrete;
    }

    void pointerFrame() {
      float delta_x = has_discrete_ ? discrete_x_ : scroll_x_;
      float delta_y = has_discrete_ ? discrete_y_ : scroll_y_;
      if (pointer_focus_ && (delta_x || delta_y))
        pointer_focus_->pointerWheel(delta_x, delta_y);
      scroll_x_ = scroll_y_ = 0.0;
      discrete_x_ = discrete_y_ = 0;
      has_discrete_ = false;
    }

    void keyboardKeymap(uint32_t format, int fd, uint32_t size) {
      if (format != WL_KEYBOARD_KEYMAP_FORMAT_XKB_V1 || xkb_context_ == nullptr) {
        close(fd);
        return;
      }

      void* data = mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, 0);
      close(fd);
      if (data == MAP_FAILED)
        return;

      xkb_keymap* keymap = xkb_keymap_new_from_string(xkb_context_, static_cast<const char*>(data),
                                                      XKB_KEYMAP_FORMAT_TEXT_V1,
                                                      XKB_KEYMAP_COMPILE_NO_FLAGS);
      munmap(data, size);
      if (keymap == nullptr)
        return;

      if (xkb_state_)
        xkb_state_unref(xkb_state_);
      if (xkb_keymap_)
        xkb_keymap_unref(xkb_keymap_);
      xkb_keymap_ = keymap;
      xkb_state_ = xkb_state_new(keymap);

      if (compose_state_ == nullptr) {
        const char* locale = setlocale(LC_CTYPE, nullptr);
        for (const char* name : { "LC_ALL", "LC_CTYPE", "LANG" }) {
          const char* value = std::getenv(name);
          if (value && value[0]) {
            locale = value;
            break;
          }
        }
        xkb_compose_table* table = xkb_compose_table_new_from_locale(xkb_context_, locale ? locale : "C",
                                                                     XKB_COMPOSE_COMPILE_NO_FLAGS);
        if (table) {
          compose_state_ = xkb_compose_state_new(table, XKB_COMPOSE_STATE_NO_FLAGS);
          xkb_compose_table_unref(table);
        }
      }
    }

    void keyboardEnter(uint32_t serial, wl_surface* surface) {
      input_serial_ = serial;
      keyboard_focus_ = findWindow(surface);
      if (keyboard_focus_) {
        WindowWayland::setLastActiveWindow(keyboard_focus_);
        keyboard_focus_->handleFocusGained();
      }
    }

    void keyboardLeave(wl_surface* surface) {
      repeat_key_ = 0;
      WindowWayland* window = keyboard_focus_;
      keyboard_focus_ = nullptr;
      if (window && window->surface() == surface) {
        window->unlockPointer();
        window->handleFocusLost();
      }
    }

    xkb_keysym_t keySymbol(xkb_keycode_t code) const {
      const xkb_keysym_t* symbols = nullptr;
      int count = xkb_keymap_key_get_syms_by_level(xkb_keymap_, code, 0, 0, &symbols);
      xkb_keysym_t symbol = count > 0 ? symbols[0] : XKB_KEY_NoSymbol;
      if (symbol >= XKB_KEY_KP_Space && symbol <= XKB_KEY_KP_Equal)
        symbol = xkb_state_key_get_one_sym(xkb_state_, code);
      return symbol;
    }

    bool keyText(xkb_keycode_t code, std::string& text) const {
      static constexpr int kMaxCharacters = 64;
      char buffer[kMaxCharacters] {};
      xkb_keysym_t symbol = xkb_state_key_get_one_sym(xkb_state_, code);
      if (compose_state_ && xkb_compose_state_feed(compose_state_, symbol) == XKB_COMPOSE_FEED_ACCEPTED) {
        switch (xkb_compose_state_get_status(compose_state_)) {
        case XKB_COMPOSE_COMPOSING: return false;
        case XKB_COMPOSE_CANCELLED: xkb_compose_state_reset(compose_state_); return false;
        case XKB_COMPOSE_COMPOSED:
          xkb_compose_state_get_utf8(compose_state_, buffer, sizeof(buffer));
          xkb_compose_state_reset(compose_state_);
          text = buffer;
          return true;
        default: break;
        }
      }

      xkb_state_key_get_utf8(xkb_state_, code, buffer, sizeof(buffer));
      text = buffer;
      return true;
    }

    void keyboardKey(uint32_t serial, uint32_t key, uint32_t state) {
      input_serial_ = serial;
      bool pressed = state == WL_KEYBOARD_KEY_STATE_PRESSED;
      xkb_keycode_t code = key + 8;
      if (pressed && xkb_keymap_ && xkb_keymap_key_repeats(xkb_keymap_, code) && repeat_rate_ > 0) {
        repeat_key_ = code;
        next_repeat_us_ = time::microseconds() + repeat_delay_ms_ * 1000LL;
      }
      else if (!pressed && repeat_key_ == code)
        repeat_key_ = 0;

      handleKey(code, pressed, false);
    }

    void handleKey(xkb_keycode_t code, bool pressed, bool repeat) {
      WindowWayland* window = keyboard_focus_;
      if (window == nullptr || xkb_state_ == nullptr)
        return;

      int modifier_state = modifierState();
      KeyCode key_code = translateKeyCode(keySymbol(code));
      if (!pressed) {
        if (key_code != KeyCode::Unknown)
          window->handleKeyUp(key_code, modifier_state & ~modifierForKeyCode(key_code));
        return;
      }

      std::string text;
      if (!keyText(code, text))
        return;

      if (key_code != KeyCode::Unknown)
        window->handleKeyDown(key_code, modifier_state | modifierForKeyCode(key_code), repeat);

      bool shortcut = modifier_state & (kModifierRegCtrl | kModifierAlt | kModifierMeta);
      unsigned char first = text.empty() ? 0 : text[0];
      if (!shortcut && first >= kFirstPrintable && first != kDelete && keyboard_focus_ == window)
        window->handleTextInput(text);
    }

    void keyboardModifiers(uint32_t depressed, uint32_t latched, uint32_t locked, uint32_t group) {
      if (xkb_state_)
        xkb_state_update_mask(xkb_state_, depressed, latched, locked, 0, 0, group);
    }

    void keyboardRepeat(int32_t rate, int32_t delay) {
      repeat_rate_ = rate;
      repeat_delay_ms_ = delay;
      if (rate <= 0)
        repeat_key_ = 0;
    }

    static wp_cursor_shape_device_v1_shape cursorShape(MouseCursor style) {
      switch (style) {
      case MouseCursor::IBeam: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_TEXT;
      case MouseCursor::Crosshair: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_CROSSHAIR;
      case MouseCursor::Pointing: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_POINTER;
      case MouseCursor::Drag: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_GRAB;
      case MouseCursor::Dragging: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_GRABBING;
      case MouseCursor::HorizontalResize: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_EW_RESIZE;
      case MouseCursor::VerticalResize: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NS_RESIZE;
      case MouseCursor::TopLeftResize: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NW_RESIZE;
      case MouseCursor::TopRightResize: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NE_RESIZE;
      case MouseCursor::BottomLeftResize: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_SW_RESIZE;
      case MouseCursor::BottomRightResize: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_SE_RESIZE;
      case MouseCursor::MultiDirectionalResize: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_ALL_SCROLL;
      default: return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_DEFAULT;
      }
    }

    static std::vector<const char*> cursorNames(MouseCursor style) {
      switch (style) {
      case MouseCursor::IBeam: return { "text", "xterm" };
      case MouseCursor::Crosshair: return { "crosshair" };
      case MouseCursor::Pointing: return { "pointer", "hand2" };
      case MouseCursor::Drag: return { "grab", "openhand", "hand1" };
      case MouseCursor::Dragging: return { "grabbing", "closedhand", "fleur" };
      case MouseCursor::HorizontalResize: return { "ew-resize", "sb_h_double_arrow" };
      case MouseCursor::VerticalResize: return { "ns-resize", "sb_v_double_arrow" };
      case MouseCursor::TopLeftResize: return { "nw-resize", "top_left_corner" };
      case MouseCursor::TopRightResize: return { "ne-resize", "top_right_corner" };
      case MouseCursor::BottomLeftResize: return { "sw-resize", "bottom_left_corner" };
      case MouseCursor::BottomRightResize: return { "se-resize", "bottom_right_corner" };
      case MouseCursor::MultiDirectionalResize: return { "all-scroll", "fleur" };
      default: return { "default", "left_ptr" };
      }
    }

    void applyCursor() {
      if (pointer_ == nullptr || pointer_focus_ == nullptr)
        return;

      if (cursor_style_ == MouseCursor::Invisible) {
        wl_pointer_set_cursor(pointer_, enter_serial_, nullptr, 0, 0);
        return;
      }

      if (cursor_shape_device_) {
        wp_cursor_shape_device_v1_set_shape(cursor_shape_device_, enter_serial_,
                                            cursorShape(cursor_style_));
        return;
      }

      int scale = std::max(1, static_cast<int>(std::ceil(pointer_focus_->dpiScale())));
      if (cursor_theme_ == nullptr || cursor_theme_scale_ != scale) {
        if (cursor_theme_)
          wl_cursor_theme_destroy(cursor_theme_);
        const char* size_setting = std::getenv("XCURSOR_SIZE");
        int size = size_setting ? std::atoi(size_setting) : 0;
        size = size > 0 ? size : kDefaultCursorSize;
        cursor_theme_ = wl_cursor_theme_load(std::getenv("XCURSOR_THEME"), size * scale, shm_);
        cursor_theme_scale_ = scale;
      }
      if (cursor_theme_ == nullptr)
        return;

      wl_cursor* cursor = nullptr;
      for (const char* name : cursorNames(cursor_style_)) {
        cursor = wl_cursor_theme_get_cursor(cursor_theme_, name);
        if (cursor)
          break;
      }
      if (cursor == nullptr || cursor->image_count == 0)
        return;

      wl_cursor_image* image = cursor->images[0];
      wl_buffer* buffer = wl_cursor_image_get_buffer(image);
      if (buffer == nullptr)
        return;

      if (cursor_surface_ == nullptr)
        cursor_surface_ = wl_compositor_create_surface(compositor_);
      wl_surface_set_buffer_scale(cursor_surface_, scale);
      wl_surface_attach(cursor_surface_, buffer, 0, 0);
      wl_surface_damage(cursor_surface_, 0, 0, INT_MAX, INT_MAX);
      wl_surface_commit(cursor_surface_);
      wl_pointer_set_cursor(pointer_, enter_serial_, cursor_surface_, image->hotspot_x / scale,
                            image->hotspot_y / scale);
    }

    bool receive(wl_data_offer* offer, const char* type, std::string& result) {
      int fds[2];
      if (pipe2(fds, O_CLOEXEC))
        return false;

      wl_data_offer_receive(offer, type, fds[1]);
      close(fds[1]);
      wl_display_flush(display_);

      bool complete = false;
      char buffer[4096];
      long long deadline = time::milliseconds() + kDataReadTimeoutMs;
      while (true) {
        long long remaining = deadline - time::milliseconds();
        pollfd fd = { fds[0], POLLIN, 0 };
        if (remaining <= 0 || poll(&fd, 1, remaining) <= 0)
          break;

        ssize_t length = read(fds[0], buffer, sizeof(buffer));
        if (length < 0 && errno == EINTR)
          continue;
        if (length <= 0) {
          complete = length == 0;
          break;
        }
        result.append(buffer, length);
      }
      close(fds[0]);
      return complete;
    }

    void dataOffer(wl_data_offer* offer) {
      offer_types_[offer];
      wl_data_offer_add_listener(offer, &kDataOfferListener, this);
    }

    void destroyOffer(wl_data_offer* offer) {
      if (offer == nullptr)
        return;
      offer_types_.erase(offer);
      wl_data_offer_destroy(offer);
    }

    void dataSelection(wl_data_offer* offer) {
      if (selection_offer_ != offer)
        destroyOffer(selection_offer_);
      selection_offer_ = offer;
    }

    void dragEnter(uint32_t serial, wl_surface* surface, wl_fixed_t x, wl_fixed_t y, wl_data_offer* offer) {
      if (drag_drop_offer_ != offer)
        destroyOffer(drag_drop_offer_);
      drag_drop_offer_ = offer;
      drag_drop_serial_ = serial;
      drag_drop_window_ = findWindow(surface);
      drag_drop_files_.clear();
      if (offer == nullptr || drag_drop_window_ == nullptr)
        return;

      const std::vector<std::string>& types = offer_types_[offer];
      bool uri_list = std::find(types.begin(), types.end(), kUriListType) != types.end();
      if (!uri_list) {
        wl_data_offer_accept(offer, serial, nullptr);
        return;
      }

      if (drag_drop_source_window_)
        drag_drop_files_ = drag_drop_source_files_;
      else {
        std::string uri_list_data;
        if (receive(offer, kUriListType, uri_list_data))
          drag_drop_files_ = uriListFiles(uri_list_data);
      }

      if (data_device_manager_version_ >= 3)
        wl_data_offer_set_actions(offer, WL_DATA_DEVICE_MANAGER_DND_ACTION_COPY,
                                  WL_DATA_DEVICE_MANAGER_DND_ACTION_COPY);
      dragMotion(x, y);
    }

    void dragMotion(wl_fixed_t x, wl_fixed_t y) {
      if (drag_drop_window_ == nullptr || drag_drop_offer_ == nullptr)
        return;

      drag_drop_position_ = nativePoint(drag_drop_window_, x, y);
      bool accepted = drag_drop_window_->handleFileDrag(drag_drop_position_.x,
                                                        drag_drop_position_.y, drag_drop_files_);
      if (drag_drop_offer_)
        wl_data_offer_accept(drag_drop_offer_, drag_drop_serial_, accepted ? kUriListType : nullptr);
    }

    void dragLeave() {
      if (drag_drop_window_)
        drag_drop_window_->handleFileDragLeave();
      drag_drop_window_ = nullptr;
      destroyOffer(drag_drop_offer_);
      drag_drop_offer_ = nullptr;
    }

    void dragDrop() {
      bool success = false;
      if (drag_drop_window_)
        success = drag_drop_window_->handleFileDrop(drag_drop_position_.x, drag_drop_position_.y,
                                                    drag_drop_files_);
      if (drag_drop_offer_ && success && data_device_manager_version_ >= 3)
        wl_data_offer_finish(drag_drop_offer_);
      drag_drop_window_ = nullptr;
      destroyOffer(drag_drop_offer_);
      drag_drop_offer_ = nullptr;
    }

    void finishDragSource(wl_data_source* source) {
      wl_data_source_destroy(source);
      WindowWayland* window = drag_drop_source_window_;
      drag_drop_source_window_ = nullptr;
      drag_drop_source_files_.clear();
      if (window)
        window->finishDragDropSource();
    }

    wl_display* display_ = nullptr;
    wl_registry* registry_ = nullptr;
    wl_compositor* compositor_ = nullptr;
    uint32_t compositor_version_ = 0;
    wl_shm* shm_ = nullptr;
    xdg_wm_base* wm_base_ = nullptr;
    zxdg_decoration_manager_v1* decoration_manager_ = nullptr;
    wp_viewporter* viewporter_ = nullptr;
    wp_fractional_scale_manager_v1* fractional_scale_manager_ = nullptr;
    wp_cursor_shape_manager_v1* cursor_shape_manager_ = nullptr;
    zwp_pointer_constraints_v1* pointer_constraints_ = nullptr;
    zwp_relative_pointer_manager_v1* relative_pointer_manager_ = nullptr;
    wl_data_device_manager* data_device_manager_ = nullptr;
    uint32_t data_device_manager_version_ = 0;
    wl_seat* seat_ = nullptr;
    wl_pointer* pointer_ = nullptr;
    wl_keyboard* keyboard_ = nullptr;
    wl_data_device* data_device_ = nullptr;
    wp_cursor_shape_device_v1* cursor_shape_device_ = nullptr;
    std::vector<std::unique_ptr<WaylandOutput>> outputs_;
    std::vector<WindowWayland*> windows_;
    std::vector<WindowWayland*> open_windows_;
    float last_scale_ = 0.0f;

    LibDecor libdecor_;
    bool libdecor_tried_ = false;

    WindowWayland* pointer_focus_ = nullptr;
    WindowWayland* keyboard_focus_ = nullptr;
    uint32_t enter_serial_ = 0;
    uint32_t button_serial_ = 0;
    uint32_t input_serial_ = 0;
    int button_state_ = 0;
    double scroll_x_ = 0.0;
    double scroll_y_ = 0.0;
    int discrete_x_ = 0;
    int discrete_y_ = 0;
    bool has_discrete_ = false;

    MouseCursor cursor_style_ = MouseCursor::Arrow;
    wl_cursor_theme* cursor_theme_ = nullptr;
    int cursor_theme_scale_ = 0;
    wl_surface* cursor_surface_ = nullptr;

    xkb_context* xkb_context_ = nullptr;
    xkb_keymap* xkb_keymap_ = nullptr;
    xkb_state* xkb_state_ = nullptr;
    xkb_compose_state* compose_state_ = nullptr;
    int repeat_rate_ = 25;
    int repeat_delay_ms_ = 600;
    xkb_keycode_t repeat_key_ = 0;
    long long next_repeat_us_ = 0;

    std::map<wl_data_offer*, std::vector<std::string>> offer_types_;
    wl_data_offer* selection_offer_ = nullptr;
    wl_data_source* selection_source_ = nullptr;
    wl_data_offer* drag_drop_offer_ = nullptr;
    uint32_t drag_drop_serial_ = 0;
    WindowWayland* drag_drop_window_ = nullptr;
    IPoint drag_drop_position_;
    std::vector<std::string> drag_drop_files_;
    WindowWayland* drag_drop_source_window_ = nullptr;
    std::vector<std::string> drag_drop_source_files_;
  };

  const wl_registry_listener WaylandConnection::kRegistryListener = {
    [](void* data, wl_registry* registry, uint32_t name, const char* interface, uint32_t version) {
      static_cast<WaylandConnection*>(data)->addGlobal(registry, name, interface, version);
    },
    [](void* data, wl_registry*, uint32_t name) {
      static_cast<WaylandConnection*>(data)->removeGlobal(name);
    },
  };

  const wl_output_listener WaylandConnection::kOutputListener = {
    [](void*, wl_output*, int32_t, int32_t, int32_t, int32_t, int32_t, const char*, const char*, int32_t) {},
    [](void* data, wl_output*, uint32_t flags, int32_t width, int32_t height, int32_t refresh) {
      if ((flags & WL_OUTPUT_MODE_CURRENT) == 0)
        return;
      auto output = static_cast<WaylandOutput*>(data);
      output->width = width;
      output->height = height;
      output->refresh_mhz = refresh;
    },
    [](void* data, wl_output*) {
      auto output = static_cast<WaylandOutput*>(data);
      output->connection->outputChanged(output);
    },
    [](void* data, wl_output*, int32_t scale) {
      static_cast<WaylandOutput*>(data)->scale = std::max(1, scale);
    },
  };

  const wl_seat_listener WaylandConnection::kSeatListener = {
    [](void* data, wl_seat*, uint32_t capabilities) {
      static_cast<WaylandConnection*>(data)->seatCapabilities(capabilities);
    },
    [](void*, wl_seat*, const char*) {},
  };

  const wl_pointer_listener WaylandConnection::kPointerListener = {
    [](void* data, wl_pointer*, uint32_t serial, wl_surface* surface, wl_fixed_t x, wl_fixed_t y) {
      static_cast<WaylandConnection*>(data)->pointerEnter(serial, surface, x, y);
    },
    [](void* data, wl_pointer*, uint32_t, wl_surface* surface) {
      static_cast<WaylandConnection*>(data)->pointerLeave(surface);
    },
    [](void* data, wl_pointer*, uint32_t, wl_fixed_t x, wl_fixed_t y) {
      static_cast<WaylandConnection*>(data)->pointerMotion(x, y);
    },
    [](void* data, wl_pointer*, uint32_t serial, uint32_t, uint32_t button, uint32_t state) {
      static_cast<WaylandConnection*>(data)->pointerButton(serial, button, state);
    },
    [](void* data, wl_pointer*, uint32_t, uint32_t axis, wl_fixed_t value) {
      static_cast<WaylandConnection*>(data)->pointerAxis(axis, value);
    },
    [](void* data, wl_pointer*) { static_cast<WaylandConnection*>(data)->pointerFrame(); },
    [](void*, wl_pointer*, uint32_t) {},
    [](void*, wl_pointer*, uint32_t, uint32_t) {},
    [](void* data, wl_pointer*, uint32_t axis, int32_t discrete) {
      static_cast<WaylandConnection*>(data)->pointerAxisDiscrete(axis, discrete);
    },
  };

  const wl_keyboard_listener WaylandConnection::kKeyboardListener = {
    [](void* data, wl_keyboard*, uint32_t format, int32_t fd, uint32_t size) {
      static_cast<WaylandConnection*>(data)->keyboardKeymap(format, fd, size);
    },
    [](void* data, wl_keyboard*, uint32_t serial, wl_surface* surface, wl_array*) {
      static_cast<WaylandConnection*>(data)->keyboardEnter(serial, surface);
    },
    [](void* data, wl_keyboard*, uint32_t, wl_surface* surface) {
      static_cast<WaylandConnection*>(data)->keyboardLeave(surface);
    },
    [](void* data, wl_keyboard*, uint32_t serial, uint32_t, uint32_t key, uint32_t state) {
      static_cast<WaylandConnection*>(data)->keyboardKey(serial, key, state);
    },
    [](void* data, wl_keyboard*, uint32_t, uint32_t depressed, uint32_t latched, uint32_t locked,
       uint32_t group) {
      static_cast<WaylandConnection*>(data)->keyboardModifiers(depressed, latched, locked, group);
    },
    [](void* data, wl_keyboard*, int32_t rate, int32_t delay) {
      static_cast<WaylandConnection*>(data)->keyboardRepeat(rate, delay);
    },
  };

  const xdg_wm_base_listener WaylandConnection::kWmBaseListener = {
    [](void*, xdg_wm_base* wm_base, uint32_t serial) { xdg_wm_base_pong(wm_base, serial); },
  };

  const wl_data_device_listener WaylandConnection::kDataDeviceListener = {
    [](void* data, wl_data_device*, wl_data_offer* offer) {
      static_cast<WaylandConnection*>(data)->dataOffer(offer);
    },
    [](void* data, wl_data_device*, uint32_t serial, wl_surface* surface, wl_fixed_t x,
       wl_fixed_t y, wl_data_offer* offer) {
      static_cast<WaylandConnection*>(data)->dragEnter(serial, surface, x, y, offer);
    },
    [](void* data, wl_data_device*) { static_cast<WaylandConnection*>(data)->dragLeave(); },
    [](void* data, wl_data_device*, uint32_t, wl_fixed_t x, wl_fixed_t y) {
      static_cast<WaylandConnection*>(data)->dragMotion(x, y);
    },
    [](void* data, wl_data_device*) { static_cast<WaylandConnection*>(data)->dragDrop(); },
    [](void* data, wl_data_device*, wl_data_offer* offer) {
      static_cast<WaylandConnection*>(data)->dataSelection(offer);
    },
  };

  const wl_data_offer_listener WaylandConnection::kDataOfferListener = {
    [](void* data, wl_data_offer* offer, const char* type) {
      static_cast<WaylandConnection*>(data)->offer_types_[offer].push_back(type);
    },
    [](void*, wl_data_offer*, uint32_t) {},
    [](void*, wl_data_offer*, uint32_t) {},
  };

  const wl_data_source_listener WaylandConnection::kSelectionSourceListener = {
    [](void*, wl_data_source*, const char*) {},
    [](void*, wl_data_source*, const char* type, int32_t fd) {
      writeAll(fd, std::strcmp(type, "STRING") == 0 ? utf8ToLatin1(_wayland_clipboard_text) :
                                                      _wayland_clipboard_text);
      close(fd);
    },
    [](void* data, wl_data_source* source) {
      auto connection = static_cast<WaylandConnection*>(data);
      if (connection->selection_source_ == source)
        connection->selection_source_ = nullptr;
      wl_data_source_destroy(source);
    },
    [](void*, wl_data_source*) {},
    [](void*, wl_data_source*) {},
    [](void*, wl_data_source*, uint32_t) {},
  };

  const wl_data_source_listener WaylandConnection::kDragSourceListener = {
    [](void*, wl_data_source*, const char*) {},
    [](void* data, wl_data_source*, const char*, int32_t fd) {
      std::string uri_list;
      for (const std::string& file : static_cast<WaylandConnection*>(data)->drag_drop_source_files_)
        uri_list += fileUri(file) + "\r\n";
      writeAll(fd, uri_list);
      close(fd);
    },
    [](void* data, wl_data_source* source) {
      static_cast<WaylandConnection*>(data)->finishDragSource(source);
    },
    [](void*, wl_data_source*) {},
    [](void* data, wl_data_source* source) {
      static_cast<WaylandConnection*>(data)->finishDragSource(source);
    },
    [](void*, wl_data_source*, uint32_t) {},
  };

  const zwp_relative_pointer_v1_listener WaylandConnection::kRelativePointerListener = {
    [](void* data, zwp_relative_pointer_v1*, uint32_t, uint32_t, wl_fixed_t dx, wl_fixed_t dy,
       wl_fixed_t, wl_fixed_t) {
      static_cast<WindowWayland*>(data)->pointerRelativeMotion(wl_fixed_to_double(dx),
                                                               wl_fixed_to_double(dy));
    },
  };

  LibDecorInterface WaylandConnection::kLibDecorInterface = {
    [](libdecor*, int, const char* message) {
      VISAGE_LOG(std::string("libdecor error: ") + message);
    },
  };

  static const wl_surface_listener kSurfaceListener = {
    [](void* data, wl_surface*, wl_output* output) {
      static_cast<WindowWayland*>(data)->outputEntered(output);
    },
    [](void* data, wl_surface*, wl_output* output) {
      static_cast<WindowWayland*>(data)->outputLeft(output);
    },
    [](void* data, wl_surface*, int32_t scale) {
      static_cast<WindowWayland*>(data)->setIntegerScale(scale);
    },
    [](void*, wl_surface*, uint32_t) {},
  };

  static const wp_fractional_scale_v1_listener kFractionalScaleListener = {
    [](void* data, wp_fractional_scale_v1*, uint32_t scale) {
      static_cast<WindowWayland*>(data)->setFractionalScale(scale / static_cast<float>(kWaylandScaleDenominator));
    },
  };

  static const wl_callback_listener kFrameListener = {
    [](void* data, wl_callback*, uint32_t) { static_cast<WindowWayland*>(data)->frameDone(); },
  };

  static const xdg_surface_listener kXdgSurfaceListener = {
    [](void* data, xdg_surface*, uint32_t serial) {
      static_cast<WindowWayland*>(data)->surfaceConfigure(serial);
    },
  };

  static const xdg_toplevel_listener kXdgToplevelListener = {
    [](void* data, xdg_toplevel*, int32_t width, int32_t height, wl_array* states) {
      bool maximized = false;
      bool constrained = false;
      bool suspended = false;
      auto* state = static_cast<uint32_t*>(states->data);
      for (size_t i = 0; i < states->size / sizeof(uint32_t); ++i) {
        switch (state[i]) {
        case XDG_TOPLEVEL_STATE_MAXIMIZED: maximized = constrained = true; break;
        case XDG_TOPLEVEL_STATE_FULLSCREEN:
        case XDG_TOPLEVEL_STATE_TILED_LEFT:
        case XDG_TOPLEVEL_STATE_TILED_RIGHT:
        case XDG_TOPLEVEL_STATE_TILED_TOP:
        case XDG_TOPLEVEL_STATE_TILED_BOTTOM: constrained = true; break;
        case XDG_TOPLEVEL_STATE_SUSPENDED: suspended = true; break;
        default: break;
        }
      }
      static_cast<WindowWayland*>(data)->toplevelConfigure(width, height, maximized, constrained, suspended);
    },
    [](void* data, xdg_toplevel*) { static_cast<WindowWayland*>(data)->requestClose(); },
    [](void*, xdg_toplevel*, int32_t, int32_t) {},
    [](void*, xdg_toplevel*, wl_array*) {},
  };

  static const zxdg_toplevel_decoration_v1_listener kDecorationListener = {
    [](void*, zxdg_toplevel_decoration_v1*, uint32_t) {},
  };

  static LibDecorFrameInterface _libdecor_frame_interface = {
    [](libdecor_frame* frame, libdecor_configuration* configuration, void* data) {
      static_cast<WindowWayland*>(data)->decorationConfigure(frame, configuration);
    },
    [](libdecor_frame*, void* data) { static_cast<WindowWayland*>(data)->requestClose(); },
    [](libdecor_frame*, void* data) {
      wl_surface_commit(static_cast<WindowWayland*>(data)->surface());
    },
    [](libdecor_frame*, const char*, void*) {},
  };

  bool waylandAvailable() {
    return WaylandConnection::globalInstance()->display() != nullptr;
  }

  static WaylandConnection* connection() {
    WaylandConnection* wayland = WaylandConnection::globalInstance();
    return wayland->display() ? wayland : nullptr;
  }

  std::string readClipboardTextWayland() {
    WaylandConnection* wayland = connection();
    return wayland ? wayland->readClipboard() : std::string();
  }

  void setClipboardTextWayland(const std::string& text) {
    if (WaylandConnection* wayland = connection())
      wayland->setClipboard(text);
  }

  void setCursorStyleWayland(MouseCursor style) {
    if (WaylandConnection* wayland = connection())
      wayland->setCursor(style);
  }

  void setCursorVisibleWayland(bool visible) {
    setCursorStyleWayland(visible ? MouseCursor::Arrow : MouseCursor::Invisible);
  }

  Point cursorPositionWayland() {
    WindowWayland* window = WindowWayland::lastActiveWindow();
    if (window == nullptr)
      return { 0, 0 };
    return window->convertToLogical(window->lastPointerPosition());
  }

  void setCursorPositionWayland(Point window_position) {
    WindowWayland* window = WindowWayland::lastActiveWindow();
    if (window)
      window->setPointerHint(window->convertToNative(window_position));
  }

  void setCursorScreenPositionWayland(Point) { }

  void showMessageBoxWayland(std::string title, std::string message) {
    showMessageBoxX11(std::move(title), std::move(message));
  }

  float defaultDpiScaleWayland() {
    WaylandConnection* wayland = connection();
    return wayland ? wayland->defaultScale() : 1.0f;
  }

  static IPoint monitorSize(WaylandConnection* wayland, float scale) {
    static constexpr int kDefaultWidth = 1920;
    static constexpr int kDefaultHeight = 1080;

    const WaylandOutput* output = wayland ? wayland->activeOutput() : nullptr;
    if (output == nullptr || output->width <= 0 || output->height <= 0)
      return { static_cast<int>(std::round(kDefaultWidth * scale)),
               static_cast<int>(std::round(kDefaultHeight * scale)) };
    return { static_cast<int>(std::round(output->width * scale / output->scale)),
             static_cast<int>(std::round(output->height * scale / output->scale)) };
  }

  IBounds computeWindowBoundsWayland(const Dimension& x, const Dimension& y, const Dimension& width,
                                     const Dimension& height) {
    WaylandConnection* wayland = connection();
    float scale = defaultDpiScaleWayland();
    IPoint monitor = monitorSize(wayland, scale);
    int result_w = width.computeInt(scale, monitor.x, monitor.y, 100);
    int result_h = height.computeInt(scale, monitor.x, monitor.y, 100);
    int result_x = x.computeInt(scale, monitor.x, monitor.y, (monitor.x - result_w) / 2);
    int result_y = y.computeInt(scale, monitor.x, monitor.y, (monitor.y - result_h) / 2);
    return { result_x, result_y, result_w, result_h };
  }

  std::unique_ptr<Window> createWindowWayland(const Dimension& x, const Dimension& y,
                                              const Dimension& width, const Dimension& height,
                                              Window::Decoration decoration) {
    if (connection() == nullptr)
      return nullptr;

    IBounds bounds = computeWindowBoundsWayland(x, y, width, height);
    return std::make_unique<WindowWayland>(bounds.width(), bounds.height(),
                                           defaultDpiScaleWayland(), decoration);
  }

  void closeApplicationWayland() {
    if (WaylandConnection* wayland = connection())
      wayland->closeAll();
  }

  int displayFpsWayland() {
    static constexpr int kDefaultRefreshRate = 60;
    WaylandConnection* wayland = connection();
    const WaylandOutput* output = wayland ? wayland->activeOutput() : nullptr;
    if (output == nullptr || output->refresh_mhz <= 0)
      return kDefaultRefreshRate;
    return std::round(output->refresh_mhz / 1000.0);
  }

  WindowWayland* WindowWayland::last_active_window_ = nullptr;

  WindowWayland::WindowWayland(int width, int height, float scale, Decoration decoration) :
      Window(width, height), decoration_(decoration) {
    if (std::strcmp(setlocale(LC_CTYPE, nullptr), "C") == 0)
      setlocale(LC_CTYPE, "");

    wayland_ = WaylandConnection::globalInstance();
    setDpiScale(scale);
    setVisible(false);
    logical_width_ = std::max(1, static_cast<int>(std::round(width / scale)));
    logical_height_ = std::max(1, static_cast<int>(std::round(height / scale)));

    surface_ = wl_compositor_create_surface(wayland_->compositor());
    wl_surface_add_listener(surface_, &kSurfaceListener, this);
    if (wayland_->viewporter())
      viewport_ = wp_viewporter_get_viewport(wayland_->viewporter(), surface_);
    if (viewport_ && wayland_->fractionalScaleManager()) {
      fractional_scale_ = wp_fractional_scale_manager_v1_get_fractional_scale(wayland_->fractionalScaleManager(),
                                                                              surface_);
      wp_fractional_scale_v1_add_listener(fractional_scale_, &kFractionalScaleListener, this);
      fractional_ = true;
    }
    applyLogicalSize();

    start_draw_microseconds_ = time::microseconds();
    next_tick_us_ = start_draw_microseconds_;
    wayland_->addWindow(this);
    wayland_->openWindow(this);
  }

  WindowWayland::~WindowWayland() {
    wayland_->removeWindow(this);
    wayland_->closeWindow(this);
    unlockPointer();
    destroyRole();
    if (frame_callback_)
      wl_callback_destroy(frame_callback_);
    if (fractional_scale_)
      wp_fractional_scale_v1_destroy(fractional_scale_);
    if (viewport_)
      wp_viewport_destroy(viewport_);
    if (surface_)
      wl_surface_destroy(surface_);
    wl_display_flush(wayland_->display());
  }

  void* WindowWayland::globalDisplay() const {
    return wayland_->display();
  }

  int WindowWayland::posixFd() const {
    return wl_display_get_fd(wayland_->display());
  }

  long long WindowWayland::nextTickMicroseconds() const {
    if (waiting_for_frame_)
      return frame_wait_start_us_ + kStalledFrameUs;
    return next_tick_us_;
  }

  void WindowWayland::tick() {
    long long now = time::microseconds();
    const WaylandOutput* output = wayland_->findOutput(output_);
    int refresh_mhz = output && output->refresh_mhz > 0 ? output->refresh_mhz : 60000;
    next_tick_us_ = now + 1000000000LL / refresh_mhz;
    waiting_for_frame_ = false;

    bool drawable = configured_ && shown_ && isVisible();
    if (drawable && frame_callback_ == nullptr) {
      frame_callback_ = wl_surface_frame(surface_);
      wl_callback_add_listener(frame_callback_, &kFrameListener, this);
    }

    frame_presented_ = false;
    drawCallback((now - start_draw_microseconds_) / 1000000.0);
    if (frame_presented_ && frame_callback_) {
      waiting_for_frame_ = true;
      frame_wait_start_us_ = now;
    }
  }

  void WindowWayland::frameDone() {
    if (frame_callback_)
      wl_callback_destroy(frame_callback_);
    frame_callback_ = nullptr;
    waiting_for_frame_ = false;
    next_tick_us_ = time::microseconds();
  }

  void WindowWayland::outputEntered(wl_output* output) {
    if (wayland_->findOutput(output) == nullptr)
      return;

    if (std::find(entered_outputs_.begin(), entered_outputs_.end(), output) == entered_outputs_.end())
      entered_outputs_.push_back(output);
    output_ = output;
    outputChanged(output);
  }

  void WindowWayland::outputLeft(wl_output* output) {
    entered_outputs_.erase(std::remove(entered_outputs_.begin(), entered_outputs_.end(), output),
                           entered_outputs_.end());
    if (output_ == output)
      output_ = entered_outputs_.empty() ? nullptr : entered_outputs_.back();
    outputChanged(output);
  }

  void WindowWayland::outputChanged(wl_output*) {
    if (fractional_ || preferred_scale_)
      return;

    int scale = 0;
    for (wl_output* entered : entered_outputs_) {
      if (const WaylandOutput* output = wayland_->findOutput(entered))
        scale = std::max(scale, output->scale);
    }
    if (scale > 0)
      changeScale(scale);
  }

  void WindowWayland::setIntegerScale(int scale) {
    if (fractional_ || scale <= 0)
      return;

    preferred_scale_ = true;
    changeScale(scale);
  }

  void WindowWayland::setFractionalScale(float scale) {
    if (scale > 0.0f)
      changeScale(scale);
  }

  void WindowWayland::changeScale(float scale) {
    wayland_->setLastScale(scale);
    if (scale == dpiScale())
      return;

    int width = std::round(logical_width_ * scale);
    int height = std::round(logical_height_ * scale);
    setDpiScale(scale);
    setMinimumSize();
    handleAdjustResize(&width, &height, true, true);
    setNativeWindowSize(width, height);
  }

  void WindowWayland::applyLogicalSize() {
    float scale = dpiScale();
    logical_width_ = std::max(1, static_cast<int>(std::round(clientWidth() / scale)));
    logical_height_ = std::max(1, static_cast<int>(std::round(clientHeight() / scale)));
    if (viewport_) {
      wp_viewport_set_destination(viewport_, logical_width_, logical_height_);
      return;
    }

    int integer_scale = std::round(scale);
    bool divisible = integer_scale > 0 && clientWidth() % integer_scale == 0 &&
                     clientHeight() % integer_scale == 0;
    wl_surface_set_buffer_scale(surface_, divisible ? integer_scale : 1);
  }

  void WindowWayland::windowContentsResized(int, int) {
    applyLogicalSize();
  }

  void WindowWayland::setMinimumSize() {
    int min_width = kMinWidth;
    int min_height = kMinHeight;
    handleAdjustResize(&min_width, &min_height, true, true);
    int logical_min_width = std::ceil(min_width / dpiScale());
    int logical_min_height = std::ceil(min_height / dpiScale());
    if (libdecor_frame_)
      wayland_->libDecor()->set_min_content_size(libdecor_frame_, logical_min_width, logical_min_height);
    else if (xdg_toplevel_)
      xdg_toplevel_set_min_size(xdg_toplevel_, logical_min_width, logical_min_height);
  }

  void WindowWayland::applySize(int width, int height, bool constrained) {
    if (width > 0 && height > 0) {
      int native_width = std::round(width * dpiScale());
      int native_height = std::round(height * dpiScale());
      if (!constrained)
        handleAdjustResize(&native_width, &native_height, width != logical_width_, height != logical_height_);
      handleResized(native_width, native_height);
    }
    else
      handleResized(clientWidth(), clientHeight());
    applyLogicalSize();
  }

  void WindowWayland::toplevelConfigure(int width, int height, bool maximized, bool constrained,
                                        bool suspended) {
    pending_width_ = width;
    pending_height_ = height;
    pending_maximized_ = maximized;
    pending_constrained_ = constrained;
    pending_suspended_ = suspended;
  }

  void WindowWayland::surfaceConfigure(uint32_t serial) {
    xdg_surface_ack_configure(xdg_surface_, serial);
    maximized_ = pending_maximized_;
    applySize(pending_width_, pending_height_, pending_constrained_);

    configured_ = true;
    setVisible(!pending_suspended_);
    drawConfigured();
  }

  void WindowWayland::decorationConfigure(libdecor_frame* frame, libdecor_configuration* configuration) {
    LibDecor* libdecor = wayland_->libDecor();
    int width = 0;
    int height = 0;
    if (!libdecor->content_size(configuration, frame, &width, &height))
      width = height = 0;

    int state = 0;
    libdecor->window_state(configuration, &state);
    maximized_ = state & LibDecor::kStateMaximized;
    bool constrained = state &
                       (LibDecor::kStateMaximized | LibDecor::kStateFullscreen | LibDecor::kStateTiled);
    applySize(width, height, constrained);

    libdecor_state* decoration_state = libdecor->state_new(logical_width_, logical_height_);
    libdecor->commit(frame, decoration_state, configuration);
    libdecor->state_free(decoration_state);

    configured_ = true;
    setVisible((state & LibDecor::kStateSuspended) == 0);
    drawConfigured();
  }

  void WindowWayland::drawConfigured() {
    if (waiting_for_configure_) {
      next_tick_us_ = time::microseconds();
      wl_surface_commit(surface_);
      return;
    }

    tick();
    if (!frame_presented_)
      wl_surface_commit(surface_);
  }

  void WindowWayland::unmapSurface() {
    static constexpr int kWaitStepMs = 5;
    static const wl_callback_listener kUnmappedListener = {
      [](void* data, wl_callback*, uint32_t) { *static_cast<bool*>(data) = true; },
    };

    bool unmapped = false;
    wl_callback* callback = wl_surface_frame(surface_);
    wl_callback_add_listener(callback, &kUnmappedListener, &unmapped);
    wl_surface_attach(surface_, nullptr, 0, 0);
    wl_surface_commit(surface_);

    long long deadline = time::microseconds() + kStalledFrameUs;
    while (!unmapped && time::microseconds() < deadline) {
      if (!wayland_->dispatch(kWaitStepMs))
        break;
    }
    wl_callback_destroy(callback);
  }

  void WindowWayland::waitForConfigure() {
    static constexpr int kMaxConfigureRoundtrips = 10;

    waiting_for_configure_ = true;
    for (int i = 0; i < kMaxConfigureRoundtrips && shown_ && !configured_; ++i) {
      if (wl_display_roundtrip(wayland_->display()) < 0)
        break;
    }
    waiting_for_configure_ = false;
  }

  void WindowWayland::createRole(bool maximized) {
    configured_ = false;
    LibDecor* libdecor = nullptr;
    if (decoration_ == Decoration::Native && wayland_->decorationManager() == nullptr)
      libdecor = wayland_->libDecor();

    if (libdecor) {
      libdecor_frame_ = libdecor->decorate(libdecor->context, surface_, &_libdecor_frame_interface, this);
      libdecor->set_title(libdecor_frame_, title_.c_str());
      libdecor->set_app_id(libdecor_frame_, VISAGE_APPLICATION_NAME);
      setMinimumSize();
      if (maximized)
        libdecor->set_maximized(libdecor_frame_);
      libdecor->map(libdecor_frame_);
      return;
    }

    xdg_surface_ = xdg_wm_base_get_xdg_surface(wayland_->wmBase(), surface_);
    xdg_surface_add_listener(xdg_surface_, &kXdgSurfaceListener, this);
    xdg_toplevel_ = xdg_surface_get_toplevel(xdg_surface_);
    xdg_toplevel_add_listener(xdg_toplevel_, &kXdgToplevelListener, this);
    xdg_toplevel_set_title(xdg_toplevel_, title_.c_str());
    xdg_toplevel_set_app_id(xdg_toplevel_, VISAGE_APPLICATION_NAME);
    setMinimumSize();
    if (maximized)
      xdg_toplevel_set_maximized(xdg_toplevel_);

    if (wayland_->decorationManager()) {
      decoration_object_ = zxdg_decoration_manager_v1_get_toplevel_decoration(wayland_->decorationManager(),
                                                                              xdg_toplevel_);
      zxdg_toplevel_decoration_v1_add_listener(decoration_object_, &kDecorationListener, this);
      zxdg_toplevel_decoration_v1_set_mode(decoration_object_,
                                           decoration_ == Decoration::Native ?
                                               ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE :
                                               ZXDG_TOPLEVEL_DECORATION_V1_MODE_CLIENT_SIDE);
    }
    wl_surface_commit(surface_);
  }

  void WindowWayland::destroyRole() {
    if (libdecor_frame_) {
      wayland_->libDecor()->frame_unref(libdecor_frame_);
      libdecor_frame_ = nullptr;
    }
    if (decoration_object_)
      zxdg_toplevel_decoration_v1_destroy(decoration_object_);
    if (xdg_toplevel_)
      xdg_toplevel_destroy(xdg_toplevel_);
    if (xdg_surface_)
      xdg_surface_destroy(xdg_surface_);
    decoration_object_ = nullptr;
    xdg_toplevel_ = nullptr;
    xdg_surface_ = nullptr;
    configured_ = false;
  }

  void WindowWayland::show() {
    wayland_->openWindow(this);
    if (!shown_) {
      shown_ = true;
      createRole(false);
      waitForConfigure();
    }
    handleWindowShown();
  }

  void WindowWayland::showMaximized() {
    wayland_->openWindow(this);
    if (!shown_) {
      shown_ = true;
      createRole(true);
      waitForConfigure();
    }
    else if (libdecor_frame_)
      wayland_->libDecor()->set_maximized(libdecor_frame_);
    else if (xdg_toplevel_)
      xdg_toplevel_set_maximized(xdg_toplevel_);
    wl_display_flush(wayland_->display());
    handleWindowShown();
  }

  void WindowWayland::hide() {
    if (shown_) {
      shown_ = false;
      unlockPointer();
      destroyRole();
      setVisible(false);
      unmapSurface();
    }
    handleWindowHidden();
  }

  void WindowWayland::close() {
    requestClose();
  }

  bool WindowWayland::isClosed() const {
    return !wayland_->isOpen(this);
  }

  void WindowWayland::setWindowTitle(const std::string& title) {
    title_ = title;
    if (libdecor_frame_)
      wayland_->libDecor()->set_title(libdecor_frame_, title.c_str());
    else if (xdg_toplevel_)
      xdg_toplevel_set_title(xdg_toplevel_, title.c_str());
  }

  IPoint WindowWayland::maxWindowDimensions() const {
    return monitorSize(wayland_, dpiScale());
  }

  int WindowWayland::resizeOperationForPosition(int x, int y) const {
    if (decoration_ != Decoration::Client || maximized_)
      return 0;

    int border = kClientResizeBorder * dpiScale();
    int operation = 0;
    if (x <= border)
      operation = kResizeLeft;
    else if (x >= clientWidth() - border)
      operation = kResizeRight;
    if (y <= border)
      operation |= kResizeTop;
    else if (y >= clientHeight() - border)
      operation |= kResizeBottom;
    return operation;
  }

  void WindowWayland::updateCursorForPosition(int x, int y) {
    int last_hover_operation = hover_window_operation_;
    hover_window_operation_ = resizeOperationForPosition(x, y);
    if (hover_window_operation_)
      setCursorStyleWayland(windowResizeCursor(hover_window_operation_));
    else if (last_hover_operation)
      setCursorStyleWayland(MouseCursor::Arrow);
  }

  void WindowWayland::pointerEnter(double x, double y) {
    last_pointer_position_ = { static_cast<int>(x), static_cast<int>(y) };
    handleMouseEnter(x, y);
  }

  void WindowWayland::pointerLeave() {
    if (hover_window_operation_) {
      hover_window_operation_ = 0;
      setCursorStyleWayland(MouseCursor::Arrow);
    }
    handleMouseLeave(wayland_->buttonState(), wayland_->modifierState());
  }

  void WindowWayland::pointerMotion(double x, double y) {
    last_pointer_position_ = { static_cast<int>(x), static_cast<int>(y) };
    if (mouseRelativeMode()) {
      lockPointer();
      if (locked_pointer_)
        return;
    }
    else if (locked_pointer_)
      unlockPointer();

    updateCursorForPosition(x, y);
    handleMouseMove(x, y, wayland_->buttonState(), wayland_->modifierState());
  }

  void WindowWayland::pointerRelativeMotion(double dx, double dy) {
    if (!mouseRelativeMode()) {
      unlockPointer();
      return;
    }

    relative_remainder_x_ += dx * dpiScale();
    relative_remainder_y_ += dy * dpiScale();
    int delta_x = relative_remainder_x_;
    int delta_y = relative_remainder_y_;
    relative_remainder_x_ -= delta_x;
    relative_remainder_y_ -= delta_y;
    if (delta_x == 0 && delta_y == 0)
      return;

    handleMouseMove(mouse_down_position_.x + delta_x, mouse_down_position_.y + delta_y,
                    wayland_->buttonState(), wayland_->modifierState());
  }

  void WindowWayland::lockPointer() {
    if (locked_pointer_)
      return;

    locked_pointer_ = wayland_->lockPointer(surface_);
    if (locked_pointer_) {
      relative_remainder_x_ = relative_remainder_y_ = 0.0f;
      relative_pointer_ = wayland_->relativePointer(this);
    }
  }

  void WindowWayland::setPointerHint(IPoint native_position) {
    if (locked_pointer_ == nullptr)
      return;

    zwp_locked_pointer_v1_set_cursor_position_hint(locked_pointer_,
                                                   wl_fixed_from_double(native_position.x / dpiScale()),
                                                   wl_fixed_from_double(native_position.y / dpiScale()));
    wl_surface_commit(surface_);
  }

  void WindowWayland::unlockPointer() {
    if (locked_pointer_ == nullptr)
      return;

    setPointerHint(mouse_down_position_);
    zwp_locked_pointer_v1_destroy(locked_pointer_);
    locked_pointer_ = nullptr;
    if (relative_pointer_)
      zwp_relative_pointer_v1_destroy(relative_pointer_);
    relative_pointer_ = nullptr;
  }

  void WindowWayland::pointerButton(uint32_t serial, MouseButton button, bool pressed) {
    last_active_window_ = this;
    int x = last_pointer_position_.x;
    int y = last_pointer_position_.y;
    int modifiers = wayland_->modifierState();
    if (!pressed) {
      HitTestResult hit_test = currentHitTest();
      IPoint position = locked_pointer_ ? mouse_down_position_ : last_pointer_position_;
      handleMouseUp(button, position.x, position.y, wayland_->buttonState(), modifiers);
      if (!mouseRelativeMode() || wayland_->buttonState() == 0)
        unlockPointer();

      if (button != kMouseButtonLeft || handleHitTest(x, y) != hit_test)
        return;

      if (hit_test == HitTestResult::CloseButton)
        close();
      else if (hit_test == HitTestResult::MaximizeButton && xdg_toplevel_) {
        if (maximized_)
          xdg_toplevel_unset_maximized(xdg_toplevel_);
        else
          xdg_toplevel_set_maximized(xdg_toplevel_);
      }
      else if (hit_test == HitTestResult::MinimizeButton && xdg_toplevel_)
        xdg_toplevel_set_minimized(xdg_toplevel_);
      return;
    }

    mouse_down_position_ = { x, y };
    handleMouseDown(button, x, y, wayland_->buttonState(), modifiers);
    HitTestResult hit_test = handleHitTest(x, y);
    int operation = resizeOperationForPosition(x, y);
    if (button == kMouseButtonLeft && xdg_toplevel_ && (operation || hit_test == HitTestResult::TitleBar)) {
      if (operation) {
        uint32_t edges = ((operation & kResizeTop) ? XDG_TOPLEVEL_RESIZE_EDGE_TOP : 0) |
                         ((operation & kResizeBottom) ? XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM : 0) |
                         ((operation & kResizeLeft) ? XDG_TOPLEVEL_RESIZE_EDGE_LEFT : 0) |
                         ((operation & kResizeRight) ? XDG_TOPLEVEL_RESIZE_EDGE_RIGHT : 0);
        wayland_->startResize(xdg_toplevel_, edges);
      }
      else
        wayland_->startMove(xdg_toplevel_);
      handleMouseUp(button, x, y, wayland_->buttonState() & ~button, modifiers);
      return;
    }

    if (button == kMouseButtonLeft && isDragDropSource()) {
      drag_drop_source_active_ = true;
      drag_drop_files_ = { startDragDropSource() };
      wayland_->startDragDrop(this, drag_drop_files_);
      handleMouseUp(button, x, y, wayland_->buttonState() & ~button, modifiers);
    }
  }

  void WindowWayland::finishDragDropSource() {
    if (!drag_drop_source_active_)
      return;

    drag_drop_source_active_ = false;
    drag_drop_files_.clear();
    cleanupDragDropSource();
  }

  void WindowWayland::pointerWheel(float delta_x, float delta_y) {
    handleMouseWheel(delta_x, delta_y, last_pointer_position_.x, last_pointer_position_.y,
                     wayland_->buttonState(), wayland_->modifierState());
  }

  void WindowWayland::runEventLoop() {
    WaylandConnection* wayland = wayland_;
    while (true) {
      long long now = time::microseconds();
      long long deadline = wayland->nextDeadline();
      long long wait_ms = deadline == LLONG_MAX ? kMaxPollMs : (deadline - now + 999) / 1000;
      if (!wayland->dispatch(std::clamp<long long>(wait_ms, 0, kMaxPollMs)))
        return;

      wayland->runTimers();
      if (wayland->processCloseRequests() && !wayland->anyWindowShowing())
        return;
    }
  }
}

#endif
