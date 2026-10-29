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

#if VISAGE_LINUX
#include "windowing_x11.h"

#include "cursors/cursor_file.h"
#include "embedded/cursors.h"
#include "visage_utils/thread_utils.h"

#include <cctype>
#include <cerrno>
#include <clocale>
#include <cstring>
#include <dlfcn.h>
#include <poll.h>
#include <sstream>
#include <X11/cursorfont.h>
#include <X11/extensions/Xrandr.h>
#include <X11/Xutil.h>
#include <xcb/present.h>
#include <xcb/xcb.h>

namespace visage {
  static std::string _clipboard_text;

  class NativeWindowLookup {
  public:
    static NativeWindowLookup& instance() {
      static NativeWindowLookup instance;
      return instance;
    }

    void addWindow(WindowX11* window) { native_window_lookup_[window->nativeHandle()] = window; }

    void removeWindow(WindowX11* window) {
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

    void closeAll() const {
      auto windows = native_window_lookup_;
      for (auto& window : windows)
        window.second->close();
    }

    WindowX11* findWindow(::Window handle) {
      auto it = native_window_lookup_.find((void*)handle);
      return it != native_window_lookup_.end() ? it->second : nullptr;
    }

    WindowX11* pluginWindow() const {
      WindowX11* active = WindowX11::lastActiveWindow();
      if (active && active->isPluginWindow())
        return active;

      for (auto& window : native_window_lookup_) {
        if (window.second->isPluginWindow())
          return window.second;
      }
      return nullptr;
    }

  private:
    NativeWindowLookup() = default;
    ~NativeWindowLookup() = default;

    std::map<void*, WindowX11*> native_window_lookup_;
  };

  class SharedMessageWindow {
  public:
    static ::Window handle() {
      static SharedMessageWindow window;
      return window.window_handle_;
    }

  private:
    SharedMessageWindow() {
      X11Connection* x11 = X11Connection::globalInstance();
      ::Display* display = x11->display();
      window_handle_ = XCreateSimpleWindow(display, x11->rootWindow(), -100, -100, 1, 1, 0, 0, 0);
      XSelectInput(display, window_handle_, StructureNotifyMask | PropertyChangeMask);
      XFlush(display);
    }

    ~SharedMessageWindow() {
      XDestroyWindow(X11Connection::globalInstance()->display(), window_handle_);
    }

    ::Window window_handle_ = 0;
  };

  static std::string latin1ToUtf8(const std::string& text) {
    std::string result;
    for (unsigned char c : text) {
      if (c < 0x80)
        result += c;
      else {
        result += static_cast<char>(0xc0 | (c >> 6));
        result += static_cast<char>(0x80 | (c & 0x3f));
      }
    }
    return result;
  }

  static std::string utf8ToLatin1(const std::string& text) {
    std::string result;
    for (char32_t c : String::convertToUtf32(text))
      result += c < 0x100 ? static_cast<char>(c) : '?';
    return result;
  }

  static bool waitForWindowEvent(Display* display, ::Window window, int type, XEvent& event) {
    static constexpr int kSleepWait = 5;
    static constexpr int kMaxWaitTime = 250;
    for (int i = 0; i < kMaxWaitTime / kSleepWait; ++i) {
      if (XCheckTypedWindowEvent(display, window, type, &event))
        return true;
      Thread::sleep(kSleepWait);
    }
    return false;
  }

  static Atom takeWindowProperty(Display* display, ::Window window, Atom property, std::string& data) {
    Atom type = None;
    int format = 0;
    unsigned long num_items = 0, bytes_after = 0;
    unsigned char* value = nullptr;
    XGetWindowProperty(display, window, property, 0, ~0, True, AnyPropertyType, &type, &format,
                       &num_items, &bytes_after, &value);
    if (value) {
      if (format == 8)
        data.append(reinterpret_cast<char*>(value), num_items);
      XFree(value);
    }
    return type;
  }

  static bool readIncrementalProperty(Display* display, ::Window window, Atom property, Atom& type,
                                      std::string& data) {
    XEvent event;
    while (waitForWindowEvent(display, window, PropertyNotify, event)) {
      if (event.xproperty.atom != property || event.xproperty.state != PropertyNewValue)
        continue;

      std::string chunk;
      Atom chunk_type = takeWindowProperty(display, window, property, chunk);
      if (chunk_type == None)
        continue;

      type = chunk_type;
      if (chunk.empty())
        return true;
      data += chunk;
    }
    return false;
  }

  std::string readClipboardText() {
    X11Connection* x11 = X11Connection::globalInstance();
    if (x11->display() == nullptr)
      return {};
    Display* display = x11->display();
    ::Window window = SharedMessageWindow::handle();

    ::Window selection_owner = XGetSelectionOwner(display, x11->clipboard());
    if (selection_owner == None)
      return "";
    if (selection_owner == window || NativeWindowLookup::instance().findWindow(selection_owner))
      return _clipboard_text;

    std::string result;
    Atom type = None;
    Atom selection_property = XInternAtom(display, "VISAGE_SELECT", False);
    XEvent stale_event;
    while (XCheckTypedWindowEvent(display, window, SelectionNotify, &stale_event))
      continue;
    XDeleteProperty(display, window, selection_property);

    for (Atom target : { x11->utf8String(), XA_STRING }) {
      XConvertSelection(display, x11->clipboard(), target, selection_property, window, CurrentTime);

      XEvent event;
      if (!waitForWindowEvent(display, window, SelectionNotify, event))
        break;
      if (event.xselection.property != selection_property)
        continue;

      type = takeWindowProperty(display, window, selection_property, result);
      if (type == x11->incr() && !readIncrementalProperty(display, window, selection_property, type, result))
        type = None;
      break;
    }

    XEvent property_event;
    while (XCheckTypedWindowEvent(display, window, PropertyNotify, &property_event))
      continue;

    if (type != x11->utf8String() && type != XA_STRING)
      return "";
    if (!result.empty() && result.back() == 0)
      result.pop_back();
    return type == XA_STRING ? latin1ToUtf8(result) : result;
  }

  void setClipboardText(const std::string& text) {
    _clipboard_text = text;

    WindowX11* plugin_window = NativeWindowLookup::instance().pluginWindow();
    X11Connection* x11 = plugin_window ? plugin_window->x11Connection() : X11Connection::globalInstance();
    if (x11->display() == nullptr)
      return;

    ::Window owner = plugin_window ? (::Window)plugin_window->nativeHandle() :
                                     SharedMessageWindow::handle();
    X11Connection::DisplayLock lock(x11);
    XSetSelectionOwner(x11->display(), XA_PRIMARY, owner, CurrentTime);
    XSetSelectionOwner(x11->display(), x11->clipboard(), owner, CurrentTime);
    XSync(x11->display(), False);
  }

  void setCursorStyle(MouseCursor style) {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return;

    X11Connection* x11 = window->x11Connection();
    X11Connection::DisplayLock lock(x11);

    Cursor cursor;
    switch (style) {
    case MouseCursor::Invisible: cursor = x11->cursors().no_cursor; break;
    case MouseCursor::Arrow: cursor = x11->cursors().arrow_cursor; break;
    case MouseCursor::IBeam: cursor = x11->cursors().ibeam_cursor; break;
    case MouseCursor::Crosshair: cursor = x11->cursors().crosshair_cursor; break;
    case MouseCursor::Pointing: cursor = x11->cursors().pointing_cursor; break;
    case MouseCursor::Drag: cursor = x11->cursors().drag_cursor; break;
    case MouseCursor::Dragging: cursor = x11->cursors().dragging_cursor; break;
    case MouseCursor::HorizontalResize: cursor = x11->cursors().horizontal_resize_cursor; break;
    case MouseCursor::VerticalResize: cursor = x11->cursors().vertical_resize_cursor; break;
    case MouseCursor::TopLeftResize: cursor = x11->cursors().top_left_resize_cursor; break;
    case MouseCursor::TopRightResize: cursor = x11->cursors().top_right_resize_cursor; break;
    case MouseCursor::BottomLeftResize: cursor = x11->cursors().bottom_left_resize_cursor; break;
    case MouseCursor::BottomRightResize: cursor = x11->cursors().bottom_right_resize_cursor; break;
    case MouseCursor::MultiDirectionalResize:
      cursor = x11->cursors().multi_directional_resize_cursor;
      break;
    default: return;
    }

    XDefineCursor(x11->display(), (::Window)window->nativeHandle(), cursor);
    XFlush(x11->display());
  }

  static MouseCursor windowResizeCursor(int operation) {
    switch (operation) {
    case kResizeLeft | kResizeTop: return MouseCursor::TopLeftResize;
    case kResizeRight | kResizeTop: return MouseCursor::TopRightResize;
    case kResizeLeft | kResizeBottom: return MouseCursor::BottomLeftResize;
    case kResizeRight | kResizeBottom: return MouseCursor::BottomRightResize;
    case kResizeLeft:
    case kResizeRight: return MouseCursor::HorizontalResize;
    case kResizeTop:
    case kResizeBottom: return MouseCursor::VerticalResize;
    default: return MouseCursor::Arrow;
    }
  }

  void setCursorVisible(bool visible) {
    if (visible)
      setCursorStyle(MouseCursor::Arrow);
    else
      setCursorStyle(MouseCursor::Invisible);
  }

  Point cursorPosition() {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return { 0, 0 };

    X11Connection* x11 = X11Connection::globalInstance();
    X11Connection::DisplayLock lock(x11);
    ::Window root_return, child_return;
    int root_x = 0, root_y = 0;
    int win_x = 0, win_y = 0;
    unsigned int mask_return = 0;

    XQueryPointer(x11->display(), (::Window)window->nativeHandle(), &root_return, &child_return,
                  &root_x, &root_y, &win_x, &win_y, &mask_return);
    return window->convertToLogical(IPoint(win_x, win_y));
  }

  static void setNativeCursorPosition(IPoint native_position) {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return;

    X11Connection* x11 = window->x11Connection();
    X11Connection::DisplayLock lock(x11);
    XWarpPointer(x11->display(), None, (::Window)window->nativeHandle(), 0, 0, 0, 0,
                 native_position.x, native_position.y);
    XFlush(x11->display());
  }

  void setCursorPosition(Point window_position) {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return;

    setNativeCursorPosition(window->convertToNative(window_position));
  }

  void setCursorScreenPosition(Point window_position) {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return;

    X11Connection* x11 = X11Connection::globalInstance();
    X11Connection::DisplayLock lock(x11);

    IPoint position = window->convertToNative(window_position);
    XWarpPointer(x11->display(), None, x11->rootWindow(), 0, 0, 0, 0, position.x, position.y);
    XFlush(x11->display());
  }

  static IPoint nativeCursorScreenPosition() {
    X11Connection* x11 = X11Connection::globalInstance();
    if (x11->display() == nullptr)
      return {};

    X11Connection::DisplayLock lock(x11);

    ::Window root_return, child_return;
    int root_x = 0, root_y = 0;
    int win_x = 0, win_y = 0;
    unsigned int mask_return = 0;

    XQueryPointer(x11->display(), x11->rootWindow(), &root_return, &child_return, &root_x, &root_y,
                  &win_x, &win_y, &mask_return);
    return { root_x, root_y };
  }

  bool isMobileDevice() {
    return false;
  }

  static double refreshRate(XRRScreenResources* screen_resources, XRRCrtcInfo* info) {
    for (int i = 0; i < screen_resources->nmode; ++i) {
      if (screen_resources->modes[i].id == info->mode)
        return screen_resources->modes[i].dotClock * 1.0 /
               (screen_resources->modes[i].hTotal * screen_resources->modes[i].vTotal);
    }
    return MonitorInfo::kDefaultRefreshRate;
  }

  static constexpr float kInchToMm = 25.4f;
  static constexpr float kMinDpi = 50.0f;
  static constexpr float kMaxDpi = 500.0f;

  static float screenDpi(Display* display) {
    int screen = DefaultScreen(display);
    int width_mm = DisplayWidthMM(display, screen);
    if (width_mm <= 0)
      return Window::kDefaultDpi;
    return DisplayWidth(display, screen) * kInchToMm / width_mm;
  }

  static float monitorDpi(const XRRCrtcInfo* crtc, const XRROutputInfo* output, float screen_dpi) {
    bool aspect_ratio_as_size = output->mm_width == 160 &&
                                (output->mm_height == 90 || output->mm_height == 100);
    if (aspect_ratio_as_size)
      return Window::kDefaultDpi;

    float dpi = screen_dpi;
    if (output->mm_width) {
      bool rotated = crtc->rotation & (RR_Rotate_90 | RR_Rotate_270);
      dpi = (rotated ? crtc->height : crtc->width) * kInchToMm / output->mm_width;
    }
    if (dpi < kMinDpi || dpi > kMaxDpi)
      return Window::kDefaultDpi;
    return dpi;
  }

  static float resourceDpi(Display* display) {
    const char* resources = XResourceManagerString(display);
    const char* dpi_setting = resources ? std::strstr(resources, "Xft.dpi:") : nullptr;
    if (dpi_setting == nullptr)
      return 0.0f;

    float dpi = std::strtof(dpi_setting + std::strlen("Xft.dpi:"), nullptr);
    return dpi >= kMinDpi && dpi <= kMaxDpi ? dpi : 0.0f;
  }

  static float roundDpi(float dpi) {
    static constexpr float kDpiAdjustment = 4.0f / Window::kDefaultDpi;
    return std::round(dpi * kDpiAdjustment) / kDpiAdjustment;
  }

  static std::vector<MonitorInfo> monitors() {
    X11Connection* x11 = X11Connection::globalInstance();
    if (x11->display() == nullptr)
      return {};

    X11Connection::DisplayLock lock(x11);
    Display* display = x11->display();

    std::vector<MonitorInfo> result;
    float resource_dpi = resourceDpi(display);
    float screen_dpi = screenDpi(display);

    XRRScreenResources* screen_resources = XRRGetScreenResourcesCurrent(display, x11->rootWindow());
    if (screen_resources == nullptr)
      return result;

    for (int i = 0; i < screen_resources->noutput; ++i) {
      XRROutputInfo* output_info = XRRGetOutputInfo(display, screen_resources,
                                                    screen_resources->outputs[i]);
      if (output_info == nullptr)
        continue;

      XRRCrtcInfo* info = nullptr;
      if (output_info->crtc && output_info->connection == RR_Connected)
        info = XRRGetCrtcInfo(display, screen_resources, output_info->crtc);

      if (info) {
        MonitorInfo monitor_info;
        monitor_info.bounds = IBounds(info->x, info->y, info->width, info->height);
        float dpi = resource_dpi > 0.0f ? resource_dpi : monitorDpi(info, output_info, screen_dpi);
        monitor_info.dpi = roundDpi(dpi);
        monitor_info.refresh_rate = refreshRate(screen_resources, info);
        result.push_back(monitor_info);
        XRRFreeCrtcInfo(info);
      }
      XRRFreeOutputInfo(output_info);
    }

    XRRFreeScreenResources(screen_resources);
    return result;
  }

  static MonitorInfo screenMonitorInfo() {
    X11Connection* x11 = X11Connection::globalInstance();
    if (x11->display() == nullptr)
      return {};

    X11Connection::DisplayLock lock(x11);
    Display* display = x11->display();

    int screen = DefaultScreen(display);
    MonitorInfo result;
    result.bounds = IBounds(0, 0, DisplayWidth(display, screen), DisplayHeight(display, screen));
    float resource_dpi = resourceDpi(display);
    result.dpi = roundDpi(resource_dpi > 0.0f ? resource_dpi : screenDpi(display));
    return result;
  }

  const MonitorInfo& nearestMonitor(const std::vector<MonitorInfo>& monitors, IPoint point) {
    VISAGE_ASSERT(!monitors.empty());
    const MonitorInfo* nearest = &monitors.front();
    long long nearest_distance = -1;
    for (const MonitorInfo& monitor : monitors) {
      const IBounds& bounds = monitor.bounds;
      long long x_distance = std::max({ bounds.x() - point.x, 0, point.x - bounds.right() + 1 });
      long long y_distance = std::max({ bounds.y() - point.y, 0, point.y - bounds.bottom() + 1 });
      long long distance = x_distance * x_distance + y_distance * y_distance;
      if (nearest_distance < 0 || distance < nearest_distance) {
        nearest = &monitor;
        nearest_distance = distance;
      }
    }
    return *nearest;
  }

  MonitorInfo monitorForWindow(const std::vector<MonitorInfo>& monitors, const MonitorInfo& current,
                               IBounds window_bounds) {
    if (monitors.empty())
      return current;

    IPoint center = window_bounds.topLeft() +
                    IPoint(window_bounds.width() / 2, window_bounds.height() / 2);
    const MonitorInfo& target = nearestMonitor(monitors, center);
    if (target.bounds == current.bounds || target.dpi == current.dpi)
      return target;

    float scale = target.dpi / current.dpi;
    IPoint scaled_center = window_bounds.topLeft() +
                           IPoint(std::round(window_bounds.width() * scale / 2.0f),
                                  std::round(window_bounds.height() * scale / 2.0f));
    if (nearestMonitor(monitors, scaled_center).bounds == target.bounds)
      return target;
    return current;
  }

  static MonitorInfo monitorInfoForPosition(IPoint point) {
    std::vector<MonitorInfo> all_monitors = monitors();
    if (all_monitors.empty())
      return screenMonitorInfo();
    return nearestMonitor(all_monitors, point);
  }

  static MonitorInfo activeMonitorInfo() {
    return monitorInfoForPosition(nativeCursorScreenPosition());
  }

  static void drawMessageBox(int width, int height, Display* display, ::Window window, GC gc,
                             const std::string& message) {
    XClearWindow(display, window);

    XFontStruct* font = XLoadQueryFont(display, "-misc-fixed-medium-r-*-*-24-*-*-*-*-*-*-*");
    if (!font) {
      VISAGE_LOG("Unable to load font");
      return;
    }

    int button_width = width / 2;
    int button_height = height / 8;
    int button_x = (width - button_width) / 2;
    int button_y = height - 2 * button_height;

    XSetFont(display, gc, font->fid);
    int text_width = XTextWidth(font, message.c_str(), message.length());
    XDrawString(display, window, gc, (width - text_width) / 2, button_y / 2 + 12, message.c_str(),
                message.size());

    XDrawRectangle(display, window, gc, button_x, button_y, button_width, button_height);
    int ok_width = XTextWidth(font, "OK", 2);
    XDrawString(display, window, gc, (width - ok_width) / 2, button_y + (button_height + 24) / 2, "OK", 2);

    XFreeFont(display, font);
  }

  static void setWindowName(X11Connection* x11, ::Window window, const std::string& name) {
    Atom net_wm_name = XInternAtom(x11->display(), "_NET_WM_NAME", False);
    Xutf8SetWMProperties(x11->display(), window, name.c_str(), nullptr, nullptr, 0, nullptr,
                         nullptr, nullptr);
    XChangeProperty(x11->display(), window, net_wm_name, x11->utf8String(), 8, PropModeReplace,
                    (const unsigned char*)name.c_str(), name.size());
  }

  static Bool isWindowEvent(Display* display, XEvent* event, XPointer window) {
    return event->xany.window == *reinterpret_cast<::Window*>(window);
  }

  static constexpr long kNetWmStateAdd = 1;
  static constexpr long kNetWmStateToggle = 2;

  static void sendMaximizeState(X11Connection* x11, ::Window window, long action) {
    Display* display = x11->display();
    XEvent event = {};
    event.type = ClientMessage;
    event.xclient.window = window;
    event.xclient.message_type = XInternAtom(display, "_NET_WM_STATE", False);
    event.xclient.format = 32;
    event.xclient.data.l[0] = action;
    event.xclient.data.l[1] = XInternAtom(display, "_NET_WM_STATE_MAXIMIZED_HORZ", False);
    event.xclient.data.l[2] = XInternAtom(display, "_NET_WM_STATE_MAXIMIZED_VERT", False);

    XSendEvent(display, x11->rootWindow(), False, SubstructureRedirectMask | SubstructureNotifyMask, &event);
    XFlush(display);
  }

  void showMessageBox(std::string title, std::string message) {
    IBounds bounds = computeWindowBounds(Dimension::viewMinPercent(30.0f),
                                         Dimension::viewMinPercent(20.0f));
    X11Connection* x11 = X11Connection::globalInstance();
    if (x11->display() == nullptr)
      return;

    Display* display = x11->display();
    int screen = DefaultScreen(display);

    ::Window message_window = XCreateSimpleWindow(display, x11->rootWindow(), bounds.x(),
                                                  bounds.y(), bounds.width(), bounds.height(), 0,
                                                  BlackPixel(display, screen),
                                                  WhitePixel(display, screen));
    setWindowName(x11, message_window, title);
    Atom wm_state = XInternAtom(display, "_NET_WM_STATE", False);
    Atom wm_state_above = XInternAtom(display, "_NET_WM_STATE_ABOVE", False);
    XChangeProperty(display, message_window, wm_state, XA_ATOM, 32, PropModeReplace,
                    (unsigned char*)&wm_state_above, 1);

    XSizeHints* size_hints = XAllocSizeHints();
    size_hints->flags = PMinSize | PMaxSize | USPosition | USSize | PPosition;
    size_hints->min_width = size_hints->max_width = bounds.width();
    size_hints->min_height = size_hints->max_height = bounds.height();
    size_hints->x = bounds.x();
    size_hints->y = bounds.y();
    XSetWMNormalHints(display, message_window, size_hints);
    XFree(size_hints);

    XSelectInput(display, message_window, ExposureMask | ButtonPressMask | KeyPressMask);
    GC gc = XCreateGC(display, message_window, 0, nullptr);
    Atom wm_delete_message = XInternAtom(display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(display, message_window, &wm_delete_message, 1);

    XMapWindow(display, message_window);
    XFlush(display);
    static constexpr int kSleepWait = 10;
    XEvent event;
    while (true) {
      if (!XCheckIfEvent(display, &event, isWindowEvent, reinterpret_cast<XPointer>(&message_window))) {
        Thread::sleep(kSleepWait);
        continue;
      }

      if (event.type == Expose)
        drawMessageBox(bounds.width(), bounds.height(), display, message_window, gc, message);

      else if (event.type == ButtonPress) {
        int button_width = bounds.width() / 2;
        int button_height = bounds.height() / 8;
        int button_x = (bounds.width() - button_width) / 2;
        int button_y = bounds.height() - 2 * button_height;

        if (event.xbutton.x >= button_x && event.xbutton.x <= button_x + button_width &&
            event.xbutton.y >= button_y && event.xbutton.y <= button_y + button_height) {
          break;
        }
      }
      else if (event.type == KeyPress || event.type == DestroyNotify)
        break;
      else if (event.type == ClientMessage) {
        if (event.xclient.data.l[0] == wm_delete_message)
          break;
      }
    }

    XFreeGC(display, gc);
    XDestroyWindow(display, message_window);
  }

  float defaultDpiScale() {
    MonitorInfo monitor_info = activeMonitorInfo();
    return monitor_info.dpi / Window::kDefaultDpi;
  }

  IBounds computeWindowBounds(const Dimension& x, const Dimension& y, const Dimension& width,
                              const Dimension& height) {
    MonitorInfo monitor_info = activeMonitorInfo();
    int monitor_width = monitor_info.bounds.width();
    int monitor_height = monitor_info.bounds.height();
    float dpi_scale = monitor_info.dpi / Window::kDefaultDpi;
    int result_w = width.computeInt(dpi_scale, monitor_width, monitor_height, 100);
    int result_h = height.computeInt(dpi_scale, monitor_width, monitor_height, 100);

    int result_x = x.computeInt(monitor_info.dpi / Window::kDefaultDpi, monitor_info.bounds.width(),
                                monitor_info.bounds.height(), (monitor_width - result_w) / 2);
    int result_y = y.computeInt(monitor_info.dpi / Window::kDefaultDpi, monitor_info.bounds.width(),
                                monitor_info.bounds.height(), (monitor_height - result_h) / 2);
    return { monitor_info.bounds.x() + result_x, monitor_info.bounds.y() + result_y, result_w, result_h };
  }

  std::unique_ptr<Window> createWindow(const Dimension& x, const Dimension& y, const Dimension& width,
                                       const Dimension& height, Window::Decoration decoration) {
    if (X11Connection::globalInstance()->display() == nullptr)
      return nullptr;

    IBounds bounds = computeWindowBounds(width, height);
    MonitorInfo monitor_info = activeMonitorInfo();
    int window_x = x.computeInt(monitor_info.dpi / Window::kDefaultDpi, monitor_info.bounds.width(),
                                monitor_info.bounds.height(), bounds.x());
    int window_y = y.computeInt(monitor_info.dpi / Window::kDefaultDpi, monitor_info.bounds.width(),
                                monitor_info.bounds.height(), bounds.y());
    return std::make_unique<WindowX11>(window_x, window_y, bounds.width(), bounds.height(), decoration);
  }

  void* headlessWindowHandle() {
    return nullptr;
  }

  void closeApplication() {
    NativeWindowLookup::instance().closeAll();
  }

  std::unique_ptr<Window> createPluginWindow(const Dimension& width, const Dimension& height,
                                             void* parent_handle) {
    if (X11Connection::globalInstance()->display() == nullptr)
      return nullptr;

    IBounds bounds = computeWindowBounds(width, height);
    auto window = std::make_unique<WindowX11>(bounds.width(), bounds.height(), parent_handle);
    if (window->x11Connection()->display() == nullptr)
      return nullptr;
    return window;
  }

  struct XcursorImage {
    unsigned int version;
    unsigned int size;
    unsigned int width;
    unsigned int height;
    unsigned int xhot;
    unsigned int yhot;
    unsigned int delay;
    unsigned int* pixels;
  };

  static Cursor embeddedCursor(void* xcursor, Display* display, const EmbeddedFile& file) {
    using ImageCreateFunction = XcursorImage* (*)(int, int);
    using ImageDestroyFunction = void (*)(XcursorImage*);
    using ImageLoadCursorFunction = Cursor (*)(Display*, const XcursorImage*);
    using DefaultSizeFunction = int (*)(Display*);

    auto default_size = reinterpret_cast<DefaultSizeFunction>(dlsym(xcursor,
                                                                    "XcursorGetDefaultSize"));
    auto image_create = reinterpret_cast<ImageCreateFunction>(dlsym(xcursor, "XcursorImageCreate"));
    auto image_destroy = reinterpret_cast<ImageDestroyFunction>(dlsym(xcursor,
                                                                      "XcursorImageDestroy"));
    auto image_load_cursor =
        reinterpret_cast<ImageLoadCursorFunction>(dlsym(xcursor, "XcursorImageLoadCursor"));
    cursor_file::Entry entry;
    int target_size = default_size ? default_size(display) : 0;
    if (!image_create || !image_destroy || !image_load_cursor ||
        !cursor_file::bestEntry(file.data, file.size, target_size, entry)) {
      return 0;
    }

    std::vector<uint32_t> pixels = cursor_file::decode(entry);
    if (pixels.empty())
      return 0;

    XcursorImage* image = image_create(entry.width, entry.height);
    if (image == nullptr)
      return 0;

    for (size_t i = 0; i < pixels.size(); ++i) {
      uint32_t alpha = pixels[i] >> 24;
      uint32_t red = ((pixels[i] >> 16) & 0xff) * alpha / 255;
      uint32_t green = ((pixels[i] >> 8) & 0xff) * alpha / 255;
      uint32_t blue = (pixels[i] & 0xff) * alpha / 255;
      image->pixels[i] = (alpha << 24) | (red << 16) | (green << 8) | blue;
    }
    image->xhot = entry.hot_x;
    image->yhot = entry.hot_y;
    Cursor cursor = image_load_cursor(display, image);
    image_destroy(image);
    return cursor;
  }

  static Cursor handCursor(Display* display, const char* theme_name, const EmbeddedFile& file,
                           unsigned int fallback_shape) {
    using LoadCursorFunction = Cursor (*)(Display*, const char*);

    Cursor cursor = 0;
    if (void* xcursor = dlopen("libXcursor.so.1", RTLD_LAZY | RTLD_LOCAL)) {
      if (auto load_cursor =
              reinterpret_cast<LoadCursorFunction>(dlsym(xcursor, "XcursorLibraryLoadCursor")))
        cursor = load_cursor(display, theme_name);
      if (cursor == 0)
        cursor = embeddedCursor(xcursor, display, file);
      dlclose(xcursor);
    }
    return cursor ? cursor : XCreateFontCursor(display, fallback_shape);
  }

  X11Connection::Cursors::Cursors(Display* display) {
    if (display == nullptr)
      return;

    XLockDisplay(display);
    Pixmap blank;
    XColor dummy;
    char data[1] = { 0 };
    blank = XCreateBitmapFromData(display, DefaultRootWindow(display), data, 1, 1);
    no_cursor = XCreatePixmapCursor(display, blank, blank, &dummy, &dummy, 0, 0);
    XFreePixmap(display, blank);

    arrow_cursor = XCreateFontCursor(display, XC_left_ptr);
    ibeam_cursor = XCreateFontCursor(display, XC_xterm);
    crosshair_cursor = XCreateFontCursor(display, XC_crosshair);
    pointing_cursor = XCreateFontCursor(display, XC_hand2);
    drag_cursor = handCursor(display, "grab", cursors::hand_grab_cur, XC_hand1);
    dragging_cursor = handCursor(display, "grabbing", cursors::hand_grabbing_cur, XC_fleur);
    horizontal_resize_cursor = XCreateFontCursor(display, XC_sb_h_double_arrow);
    vertical_resize_cursor = XCreateFontCursor(display, XC_sb_v_double_arrow);
    top_left_resize_cursor = XCreateFontCursor(display, XC_top_left_corner);
    top_right_resize_cursor = XCreateFontCursor(display, XC_top_right_corner);
    bottom_left_resize_cursor = XCreateFontCursor(display, XC_bottom_left_corner);
    bottom_right_resize_cursor = XCreateFontCursor(display, XC_bottom_right_corner);
    multi_directional_resize_cursor = XCreateFontCursor(display, XC_fleur);
    XUnlockDisplay(display);
  }

  int displayFps() {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window)
      return std::round(window->monitorInfo().refresh_rate);
    return MonitorInfo::kDefaultRefreshRate;
  }

  void WindowX11::createWindow(IBounds bounds) {
    VISAGE_ASSERT(bounds.width() && bounds.height());

    ::Display* display = x11_->display();
    int screen = DefaultScreen(display);

    XSetWindowAttributes window_attributes;
    window_attributes.background_pixmap = None;
    window_attributes.border_pixel = BlackPixel(display, screen);
    window_attributes.backing_store = WhenMapped;
    window_attributes.bit_gravity = NorthWestGravity;
    window_attributes.win_gravity = NorthWestGravity;

    unsigned long attribute_mask = CWBackPixmap | CWBorderPixel | CWBackingStore | CWBitGravity |
                                   CWWinGravity;
    window_handle_ = XCreateWindow(display, x11_->rootWindow(), bounds.x(), bounds.y(),
                                   bounds.width(), bounds.height(), 0, CopyFromParent, InputOutput,
                                   CopyFromParent, attribute_mask, &window_attributes);
    setWindowName(x11_, window_handle_, VISAGE_APPLICATION_NAME);

    std::string class_name = VISAGE_APPLICATION_NAME;
    XClassHint class_hint = { class_name.data(), class_name.data() };
    XSetClassHint(display, window_handle_, &class_hint);

    unsigned char blank = 0;
    XChangeProperty(display, window_handle_, x11_->dndActionList(), XA_ATOM, 32, PropModeReplace,
                    x11_->dndActions(), X11Connection::kNumDndActions);
    XChangeProperty(display, window_handle_, x11_->dndActionDescription(), XA_STRING, 8,
                    PropModeReplace, &blank, 0);
    XChangeProperty(display, window_handle_, x11_->dndAware(), XA_ATOM, 32, PropModeReplace,
                    x11_->dndVersion(), 1);

    if (x11_->inputMethod()) {
      input_context_ = XCreateIC(x11_->inputMethod(), XNInputStyle,
                                 XIMPreeditNothing | XIMStatusNothing, XNClientWindow,
                                 window_handle_, XNFocusWindow, window_handle_, nullptr);
    }
  }

  struct ErrorHandler {
    int windows = 0;
    bool installed = false;
    XErrorHandler previous = nullptr;
  };

  static ErrorHandler& errorHandler() {
    static ErrorHandler handler;
    return handler;
  }

  static void acquireErrorHandler() {
    ErrorHandler& handler = errorHandler();
    if (handler.windows++)
      return;

    XErrorHandler previous = XSetErrorHandler(nullptr);
    XErrorHandler default_handler = XSetErrorHandler(previous);
    handler.installed = previous == default_handler;
    if (handler.installed) {
      handler.previous = previous;
      XSetErrorHandler(X11Connection::xErrorHandler);
    }
  }

  static void releaseErrorHandler() {
    ErrorHandler& handler = errorHandler();
    if (--handler.windows || !handler.installed)
      return;

    handler.installed = false;
    XErrorHandler current = XSetErrorHandler(handler.previous);
    if (current != X11Connection::xErrorHandler)
      XSetErrorHandler(current);
  }

  static long long refreshMicroseconds(const MonitorInfo& monitor_info) {
    double refresh_rate = monitor_info.refresh_rate > 0 ? monitor_info.refresh_rate :
                                                          MonitorInfo::kDefaultRefreshRate;
    return std::llround(1000000.0 / refresh_rate);
  }

  static void threadTimerCallback(WindowX11* window) {
    static constexpr int kReconnectMs = 100;
    static constexpr long long kStalledPresentUs = 100000;

    xcb_connection_t* connection = xcb_connect(nullptr, nullptr);
    while (xcb_connection_has_error(connection) && window->timerThreadRunning()) {
      xcb_disconnect(connection);
      Thread::sleep(kReconnectMs);
      connection = xcb_connect(nullptr, nullptr);
    }
    if (xcb_connection_has_error(connection)) {
      xcb_disconnect(connection);
      return;
    }

    xcb_window_t window_handle = (xcb_window_t)(uintptr_t)window->nativeHandle();
    const char* atom_name = "VISAGE_TIMER_EVENT";
    xcb_intern_atom_reply_t* atom_reply = xcb_intern_atom_reply(connection,
                                                                xcb_intern_atom(connection, False,
                                                                                std::strlen(atom_name),
                                                                                atom_name),
                                                                nullptr);
    xcb_atom_t timer_event = atom_reply ? atom_reply->atom : XCB_NONE;
    free(atom_reply);

    const xcb_query_extension_reply_t* present = xcb_get_extension_data(connection, &xcb_present_id);
    bool has_present = present && present->present;
    if (has_present)
      xcb_present_select_input(connection, xcb_generate_id(connection), window_handle,
                               XCB_PRESENT_EVENT_MASK_COMPLETE_NOTIFY);
    xcb_flush(connection);

    bool completion_driven = false;
    long long next_tick_us = time::microseconds();
    pollfd fd = { xcb_get_file_descriptor(connection), POLLIN, 0 };
    while (window->timerThreadRunning()) {
      long long period_us = window->timerMicroseconds();
      long long now_us = time::microseconds();
      int outstanding = window->presentsOutstanding();
      long long deadline_us = completion_driven && outstanding ?
                                  window->lastPresentMicroseconds() + kStalledPresentUs :
                                  next_tick_us;
      int timeout_ms = std::max(0LL, (deadline_us - now_us + 999) / 1000);
      poll(&fd, 1, std::min(timeout_ms, kReconnectMs));

      bool tick = false;
      while (xcb_generic_event_t* event = xcb_poll_for_event(connection)) {
        auto* complete = reinterpret_cast<xcb_present_complete_notify_event_t*>(event);
        if ((event->response_type & 0x7f) == XCB_GE_GENERIC && has_present &&
            complete->extension == present->major_opcode &&
            complete->event_type == XCB_PRESENT_EVENT_COMPLETE_NOTIFY &&
            complete->kind == XCB_PRESENT_COMPLETE_KIND_PIXMAP && complete->window == window_handle) {
          window->presentCompleted();
          completion_driven = true;
          tick = window->presentsOutstanding() == 0;
        }
        free(event);
      }
      if (xcb_connection_has_error(connection))
        break;

      now_us = time::microseconds();
      outstanding = window->presentsOutstanding();
      if (completion_driven && outstanding &&
          now_us >= window->lastPresentMicroseconds() + kStalledPresentUs) {
        completion_driven = false;
        window->resetPresents();
        outstanding = 0;
      }
      if (!tick && (!completion_driven || outstanding == 0) && now_us >= next_tick_us)
        tick = true;
      if (!tick)
        continue;

      if (completion_driven)
        next_tick_us = now_us + period_us;
      else
        next_tick_us = std::max(next_tick_us + period_us, now_us);
      if (!window->requestTick())
        continue;

      xcb_client_message_event_t message {};
      message.response_type = XCB_CLIENT_MESSAGE;
      message.format = 32;
      message.window = window_handle;
      message.type = timer_event;
      xcb_send_event(connection, False, window_handle, XCB_EVENT_MASK_NO_EVENT,
                     reinterpret_cast<const char*>(&message));
      xcb_flush(connection);
    }

    xcb_disconnect(connection);
  }

  WindowX11* WindowX11::last_active_window_ = nullptr;

  WindowX11::WindowX11(int x, int y, int width, int height, Decoration decoration) :
      Window(width, height), decoration_(decoration) {
    if (std::strcmp(setlocale(LC_CTYPE, nullptr), "C") == 0)
      setlocale(LC_CTYPE, "");
    acquireErrorHandler();
    x11_ = X11Connection::globalInstance();

    monitor_info_ = monitorInfoForPosition({ x + width / 2, y + height / 2 });
    X11Connection::DisplayLock lock(x11_);
    ::Display* display = x11_->display();
    IBounds bounds(x, y, width, height);
    createWindow(bounds);

    if (decoration == Decoration::Popup) {
      XSetWindowAttributes attributes;
      attributes.override_redirect = True;
      XChangeWindowAttributes(display, window_handle_, CWOverrideRedirect, &attributes);
    }
    else if (decoration == Decoration::Client)
      removeWindowDecorationButtons();

    XSizeHints* size_hints = XAllocSizeHints();
    size_hints->flags = USPosition | PMinSize;
    size_hints->x = x;
    size_hints->y = y;
    size_hints->min_width = kMinWidth;
    size_hints->min_height = kMinHeight;
    XSetWMNormalHints(display, window_handle_, size_hints);
    XFree(size_hints);

    XSelectInput(display, window_handle_, kEventMask);
    timer_microseconds_ = refreshMicroseconds(monitor_info_);
    timer_thread_running_ = true;
    timer_thread_ = std::make_unique<std::thread>(threadTimerCallback, this);
    start_draw_microseconds_ = time::microseconds();
    setDpiScale(monitor_info_.dpi / kDefaultDpi);
    XFlush(display);
    NativeWindowLookup::instance().addWindow(this);
  }

  WindowX11::WindowX11(int width, int height, void* parent_handle) : Window(width, height) {
    static constexpr long kEmbedVersion = 0;
    static constexpr long kEmbedMapped = 1;

    acquireErrorHandler();
    plugin_x11_ = std::make_unique<X11Connection>();
    x11_ = plugin_x11_.get();
    if (x11_->display() == nullptr)
      return;

    monitor_info_ = activeMonitorInfo();
    X11Connection::DisplayLock lock(x11_);
    ::Display* display = x11_->display();

    parent_handle_ = (::Window)parent_handle;
    XSelectInput(display, parent_handle_, StructureNotifyMask);

    createWindow({ 0, 0, width, height });
    Atom atom_embed_info = XInternAtom(display, "_XEMBED_INFO", False);
    long embed_info[2] = { kEmbedVersion, kEmbedMapped };

    XChangeProperty(display, window_handle_, atom_embed_info, atom_embed_info, 32, PropModeReplace,
                    reinterpret_cast<unsigned char*>(embed_info), 2);

    XChangeProperty(display, parent_handle_, x11_->dndAware(), XA_ATOM, 32, PropModeReplace,
                    x11_->dndVersion(), 1);
    XChangeProperty(display, parent_handle_, x11_->dndProxy(), XA_WINDOW, 32, PropModeReplace,
                    (unsigned char*)&window_handle_, 1);
    XChangeProperty(display, window_handle_, x11_->dndProxy(), XA_WINDOW, 32, PropModeReplace,
                    (unsigned char*)&window_handle_, 1);
    XReparentWindow(display, window_handle_, parent_handle_, 0, 0);

    XSelectInput(display, window_handle_, kEventMask);
    XFlush(display);

    timer_microseconds_ = refreshMicroseconds(monitor_info_);
    timer_thread_running_ = true;
    timer_thread_ = std::make_unique<std::thread>(threadTimerCallback, this);
    start_draw_microseconds_ = time::microseconds();
    setDpiScale(monitor_info_.dpi / kDefaultDpi);
    NativeWindowLookup::instance().addWindow(this);
  }

  WindowX11::~WindowX11() {
    NativeWindowLookup::instance().removeWindow(this);
    if (last_active_window_ == this)
      last_active_window_ = nullptr;

    timer_thread_running_ = false;
    if (timer_thread_ && timer_thread_->joinable())
      timer_thread_->join();

    timer_thread_.reset();

    X11Connection::DisplayLock lock(x11_);
    WindowX11* next_owner = isPluginWindow() ? NativeWindowLookup::instance().pluginWindow() : nullptr;
    if (next_owner && x11_->display()) {
      X11Connection* next_x11 = next_owner->x11Connection();
      X11Connection::DisplayLock next_lock(next_x11);
      for (Atom selection : { (Atom)XA_PRIMARY, x11_->clipboard() }) {
        if (XGetSelectionOwner(x11_->display(), selection) == window_handle_)
          XSetSelectionOwner(next_x11->display(), selection, (::Window)next_owner->nativeHandle(),
                             CurrentTime);
      }
      XSync(next_x11->display(), False);
    }

    if (parent_handle_ && dragDropProxy(parent_handle_) == window_handle_) {
      XDeleteProperty(x11_->display(), parent_handle_, x11_->dndAware());
      XDeleteProperty(x11_->display(), parent_handle_, x11_->dndProxy());
    }
    if (input_context_)
      XDestroyIC(input_context_);
    if (window_handle_)
      XDestroyWindow(x11_->display(), window_handle_);

    if (x11_->display())
      XSync(x11_->display(), False);
    releaseErrorHandler();
  }

  void WindowX11::framePresented() {
    last_present_us_ = time::microseconds();
    presents_outstanding_++;
  }

  double WindowX11::drawTime() const {
    return (time::microseconds() - start_draw_microseconds_) / 1000000.0;
  }

  int WindowX11::resizeOperationForPosition(int x, int y) const {
    if (decoration_ != Decoration::Client)
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

  void WindowX11::removeWindowDecorationButtons() {
    Atom mwm_hints = XInternAtom(x11_->display(), "_MOTIF_WM_HINTS", False);
    struct MwmHints {
      unsigned long flags = 0;
      unsigned long functions = 0;
      unsigned long decorations = 0;
      long input_mode = 0;
      unsigned long status = 0;
    };

    MwmHints hints;
    hints.flags = 2;
    XChangeProperty(x11_->display(), window_handle_, mwm_hints, mwm_hints, 32, PropModeReplace,
                    reinterpret_cast<unsigned char*>(&hints), 5);
  }

  void* WindowX11::initWindow() const {
    return (void*)SharedMessageWindow::handle();
  }

  template<typename F>
  void WindowX11::editSizeHints(F&& edit) {
    if (parent_handle_)
      return;

    X11Connection::DisplayLock lock(x11_);
    XSizeHints* size_hints = XAllocSizeHints();
    long supplied_return;
    XGetWMNormalHints(x11_->display(), window_handle_, size_hints, &supplied_return);
    edit(*size_hints);
    XSetWMNormalHints(x11_->display(), window_handle_, size_hints);
    XFree(size_hints);
  }

  void WindowX11::setFixedAspectRatio(bool fixed) {
    Window::setFixedAspectRatio(fixed);
    editSizeHints([this, fixed](XSizeHints& size_hints) {
      size_hints.flags = fixed ? (size_hints.flags | PAspect) : (size_hints.flags & ~PAspect);
      size_hints.min_aspect.x = clientWidth();
      size_hints.min_aspect.y = clientHeight();
      size_hints.max_aspect.x = clientWidth();
      size_hints.max_aspect.y = clientHeight();
    });
  }

  IPoint WindowX11::retrieveWindowDimensions() {
    X11Connection::DisplayLock lock(x11_);
    XWindowAttributes attributes;
    if (XGetWindowAttributes(x11_->display(), window_handle_, &attributes) == 0)
      return { clientWidth(), clientHeight() };

    return { attributes.width, attributes.height };
  }

  void WindowX11::updateMonitor(int width, int height) {
    if (parent_handle_)
      return;

    IPoint position;
    ::Window child;
    if (!XTranslateCoordinates(x11_->display(), window_handle_, x11_->rootWindow(), 0, 0,
                               &position.x, &position.y, &child)) {
      return;
    }

    IBounds window_bounds(position.x, position.y, width, height);
    if (window_bounds == last_window_bounds_)
      return;

    last_window_bounds_ = window_bounds;
    IPoint center = position + IPoint(width / 2, height / 2);
    if (monitor_info_.bounds.contains(center))
      return;

    MonitorInfo monitor_info = monitorForWindow(monitors(), monitor_info_, window_bounds);
    bool dpi_changed = monitor_info.dpi != monitor_info_.dpi;
    monitor_info_ = monitor_info;
    if (!dpi_changed)
      return;

    float scale = monitor_info_.dpi / kDefaultDpi;
    int scaled_width = std::round(width * scale / dpiScale());
    int scaled_height = std::round(height * scale / dpiScale());
    setDpiScale(scale);
    setMinimumSizeHints();
    handleAdjustResize(&scaled_width, &scaled_height, true, true);
    setNativeWindowSize(scaled_width, scaled_height);
  }

  void WindowX11::setMinimumSizeHints() {
    int min_width = kMinWidth;
    int min_height = kMinHeight;
    handleAdjustResize(&min_width, &min_height, true, true);
    editSizeHints([min_width, min_height](XSizeHints& size_hints) {
      size_hints.flags |= PMinSize;
      size_hints.min_width = min_width;
      size_hints.min_height = min_height;
    });
  }

  void WindowX11::passEventToParent(XEvent& event) {
    if (parent_handle_ == 0)
      return;

    X11Connection::DisplayLock lock(x11_);

    ::Window root = 0, parent = 0;
    ::Window* children = nullptr;
    unsigned int num_children = 0;
    Display* display = x11_->display();

    if (XQueryTree(display, window_handle_, &root, &parent, &children, &num_children) == 0)
      return;

    if (children)
      XFree(children);

    event.xany.window = parent;
    XSendEvent(display, parent, False, NoEventMask, &event);
    XFlush(display);
  }

  static int mouseButtonState(unsigned int state) {
    int result = 0;
    if (state & Button1Mask)
      result = result | kMouseButtonLeft;
    if (state & Button2Mask)
      result = result | kMouseButtonMiddle;
    if (state & Button3Mask)
      result = result | kMouseButtonRight;
    return result;
  }

  static int modifierState(unsigned int state) {
    int result = 0;
    if (state & ShiftMask)
      result = result | kModifierShift;
    if (state & ControlMask)
      result = result | kModifierRegCtrl;
    if (state & Mod1Mask)
      result = result | kModifierAlt;
    if (state & Mod4Mask)
      result = result | kModifierMeta;
    return result;
  }

  static MouseButton buttonFromEvent(XEvent& event) {
    if (event.xbutton.button == Button1)
      return kMouseButtonLeft;
    if (event.xbutton.button == Button2)
      return kMouseButtonMiddle;
    if (event.xbutton.button == Button3)
      return kMouseButtonRight;
    return kMouseButtonNone;
  }

  static KeyCode translateKeyCode(KeySym keysym) {
    switch (keysym) {
    case XK_a: return KeyCode::A;
    case XK_b: return KeyCode::B;
    case XK_c: return KeyCode::C;
    case XK_d: return KeyCode::D;
    case XK_e: return KeyCode::E;
    case XK_f: return KeyCode::F;
    case XK_g: return KeyCode::G;
    case XK_h: return KeyCode::H;
    case XK_i: return KeyCode::I;
    case XK_j: return KeyCode::J;
    case XK_k: return KeyCode::K;
    case XK_l: return KeyCode::L;
    case XK_m: return KeyCode::M;
    case XK_n: return KeyCode::N;
    case XK_o: return KeyCode::O;
    case XK_p: return KeyCode::P;
    case XK_q: return KeyCode::Q;
    case XK_r: return KeyCode::R;
    case XK_s: return KeyCode::S;
    case XK_t: return KeyCode::T;
    case XK_u: return KeyCode::U;
    case XK_v: return KeyCode::V;
    case XK_w: return KeyCode::W;
    case XK_x: return KeyCode::X;
    case XK_y: return KeyCode::Y;
    case XK_z: return KeyCode::Z;
    case XK_A: return KeyCode::A;
    case XK_B: return KeyCode::B;
    case XK_C: return KeyCode::C;
    case XK_D: return KeyCode::D;
    case XK_E: return KeyCode::E;
    case XK_F: return KeyCode::F;
    case XK_G: return KeyCode::G;
    case XK_H: return KeyCode::H;
    case XK_I: return KeyCode::I;
    case XK_J: return KeyCode::J;
    case XK_K: return KeyCode::K;
    case XK_L: return KeyCode::L;
    case XK_M: return KeyCode::M;
    case XK_N: return KeyCode::N;
    case XK_O: return KeyCode::O;
    case XK_P: return KeyCode::P;
    case XK_Q: return KeyCode::Q;
    case XK_R: return KeyCode::R;
    case XK_S: return KeyCode::S;
    case XK_T: return KeyCode::T;
    case XK_U: return KeyCode::U;
    case XK_V: return KeyCode::V;
    case XK_W: return KeyCode::W;
    case XK_X: return KeyCode::X;
    case XK_Y: return KeyCode::Y;
    case XK_Z: return KeyCode::Z;
    case XK_1: return KeyCode::Number1;
    case XK_2: return KeyCode::Number2;
    case XK_3: return KeyCode::Number3;
    case XK_4: return KeyCode::Number4;
    case XK_5: return KeyCode::Number5;
    case XK_6: return KeyCode::Number6;
    case XK_7: return KeyCode::Number7;
    case XK_8: return KeyCode::Number8;
    case XK_9: return KeyCode::Number9;
    case XK_0: return KeyCode::Number0;
    case XK_Return: return KeyCode::Return;
    case XK_Escape: return KeyCode::Escape;
    case XK_BackSpace: return KeyCode::Backspace;
    case XK_Tab: return KeyCode::Tab;
    case XK_space: return KeyCode::Space;
    case XK_minus: return KeyCode::Minus;
    case XK_equal: return KeyCode::Equals;
    case XK_bracketleft: return KeyCode::LeftBracket;
    case XK_bracketright: return KeyCode::RightBracket;
    case XK_backslash: return KeyCode::Backslash;
    case XK_semicolon: return KeyCode::Semicolon;
    case XK_apostrophe: return KeyCode::Apostrophe;
    case XK_grave: return KeyCode::Grave;
    case XK_comma: return KeyCode::Comma;
    case XK_period: return KeyCode::Period;
    case XK_slash: return KeyCode::Slash;
    case XK_Caps_Lock: return KeyCode::CapsLock;
    case XK_F1: return KeyCode::F1;
    case XK_F2: return KeyCode::F2;
    case XK_F3: return KeyCode::F3;
    case XK_F4: return KeyCode::F4;
    case XK_F5: return KeyCode::F5;
    case XK_F6: return KeyCode::F6;
    case XK_F7: return KeyCode::F7;
    case XK_F8: return KeyCode::F8;
    case XK_F9: return KeyCode::F9;
    case XK_F10: return KeyCode::F10;
    case XK_F11: return KeyCode::F11;
    case XK_F12: return KeyCode::F12;
    case XK_Print: return KeyCode::PrintScreen;
    case XK_Scroll_Lock: return KeyCode::ScrollLock;
    case XK_Pause: return KeyCode::Pause;
    case XK_Insert: return KeyCode::Insert;
    case XK_Home: return KeyCode::Home;
    case XK_Page_Up: return KeyCode::PageUp;
    case XK_Delete: return KeyCode::Delete;
    case XK_End: return KeyCode::End;
    case XK_Page_Down: return KeyCode::PageDown;
    case XK_Right: return KeyCode::Right;
    case XK_Left: return KeyCode::Left;
    case XK_Down: return KeyCode::Down;
    case XK_Up: return KeyCode::Up;
    case XK_Num_Lock: return KeyCode::NumLock;
    case XK_KP_Divide: return KeyCode::KPDivide;
    case XK_KP_Multiply: return KeyCode::KPMultiply;
    case XK_KP_Subtract: return KeyCode::KPMinus;
    case XK_KP_Add: return KeyCode::KPPlus;
    case XK_KP_Enter: return KeyCode::KPEnter;
    case XK_KP_1: return KeyCode::KP1;
    case XK_KP_2: return KeyCode::KP2;
    case XK_KP_3: return KeyCode::KP3;
    case XK_KP_4: return KeyCode::KP4;
    case XK_KP_5: return KeyCode::KP5;
    case XK_KP_6: return KeyCode::KP6;
    case XK_KP_7: return KeyCode::KP7;
    case XK_KP_8: return KeyCode::KP8;
    case XK_KP_9: return KeyCode::KP9;
    case XK_KP_0: return KeyCode::KP0;
    case XK_KP_Decimal: return KeyCode::KPPeriod;
    case XK_Shift_L: return KeyCode::LShift;
    case XK_Shift_R: return KeyCode::RShift;
    case XK_Control_L: return KeyCode::LCtrl;
    case XK_Control_R: return KeyCode::RCtrl;
    case XK_Alt_L: return KeyCode::LAlt;
    case XK_Alt_R: return KeyCode::RAlt;
    case XK_Super_L: return KeyCode::LGui;
    case XK_Super_R: return KeyCode::RGui;
    default: return KeyCode::Unknown;
    }
  }

  static int modifierForKeyCode(KeyCode key_code) {
    switch (key_code) {
    case KeyCode::LShift:
    case KeyCode::RShift: return kModifierShift;
    case KeyCode::LCtrl:
    case KeyCode::RCtrl: return kModifierRegCtrl;
    case KeyCode::LAlt:
    case KeyCode::RAlt: return kModifierAlt;
    case KeyCode::LGui:
    case KeyCode::RGui: return kModifierMeta;
    default: return 0;
    }
  }

  static std::string lookupText(XIC input_context, XKeyEvent* event) {
    static constexpr int kMaxCharacters = 32;
    std::string text(kMaxCharacters, '\0');
    if (input_context == nullptr) {
      text.resize(XLookupString(event, text.data(), text.size(), nullptr, nullptr));
      return text;
    }

    Status status;
    int length = Xutf8LookupString(input_context, event, text.data(), text.size(), nullptr, &status);
    if (status == XBufferOverflow) {
      text.resize(length);
      length = Xutf8LookupString(input_context, event, text.data(), text.size(), nullptr, &status);
    }
    text.resize(length);
    return text;
  }

  static KeySym lookupKeysym(XKeyEvent* event) {
    KeySym keysym = XLookupKeysym(event, 0);
    if (IsKeypadKey(keysym))
      XLookupString(event, nullptr, 0, &keysym, nullptr);
    return keysym;
  }

  static std::string fileUri(const std::string& path) {
    static constexpr char kHex[] = "0123456789ABCDEF";

    std::string result = "file://";
    for (unsigned char c : path) {
      if (std::isalnum(c) || c == '/' || c == '-' || c == '_' || c == '.' || c == '~')
        result += c;
      else {
        result += '%';
        result += kHex[c >> 4];
        result += kHex[c & 0xf];
      }
    }
    return result;
  }

  static std::string filePath(const std::string& uri) {
    static const std::string kFilePrefix = "file://";

    if (uri.compare(0, kFilePrefix.size(), kFilePrefix))
      return uri;

    std::string path = uri.substr(std::min(uri.find('/', kFilePrefix.size()), uri.size()));
    std::string result;
    for (int i = 0; i < path.size(); ++i) {
      if (path[i] == '%' && i + 2 < path.size() && std::isxdigit((unsigned char)path[i + 1]) &&
          std::isxdigit((unsigned char)path[i + 2])) {
        result += static_cast<char>(std::stoi(path.substr(i + 1, 2), nullptr, 16));
        i += 2;
      }
      else
        result += path[i];
    }
    return result;
  }

  ::Window WindowX11::dragDropTarget() const {
    ::Window window = x11_->rootWindow();
    while (window) {
      Atom actual_type;
      int actual_format = 0;
      unsigned long num_items = 0, bytes_after = 0;
      unsigned char* version = nullptr;
      XGetWindowProperty(x11_->display(), window, x11_->dndAware(), 0, 1, False, XA_ATOM,
                         &actual_type, &actual_format, &num_items, &bytes_after, &version);
      if (version)
        XFree(version);
      if (num_items)
        return window;

      ::Window root_return, child_return = None;
      int root_x, root_y, win_x, win_y;
      unsigned int mask_return;
      if (!XQueryPointer(x11_->display(), window, &root_return, &child_return, &root_x, &root_y,
                         &win_x, &win_y, &mask_return))
        return 0;
      window = child_return;
    }
    return 0;
  }

  ::Window WindowX11::dragDropReceiver(::Window target) const {
    ::Window proxy = dragDropProxy(target);
    return proxy ? proxy : target;
  }

  void WindowX11::sendDragDropEnter(::Window source, ::Window target) const {
    XClientMessageEvent message;
    memset(&message, 0, sizeof(message));
    message.type = ClientMessage;
    message.display = x11_->display();
    message.window = target;
    message.message_type = x11_->dndEnter();
    message.format = 32;
    message.data.l[0] = source;
    message.data.l[1] = 5 << 24;
    message.data.l[2] = x11_->dndUriList();
    message.data.l[3] = None;
    message.data.l[4] = None;

    XSendEvent(x11_->display(), dragDropReceiver(target), False, 0, reinterpret_cast<XEvent*>(&message));
    XFlush(x11_->display());
  }

  void WindowX11::sendDragDropLeave(::Window source, ::Window target) const {
    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_->display();
    message.xclient.window = target;
    message.xclient.message_type = x11_->dndLeave();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;

    XSendEvent(x11_->display(), dragDropReceiver(target), False, 0, &message);
    XFlush(x11_->display());
  }

  void WindowX11::sendDragDropPosition(::Window source, ::Window target, int x, int y,
                                       unsigned long time) const {
    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_->display();
    message.xclient.window = target;
    message.xclient.message_type = x11_->dndPosition();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    message.xclient.data.l[2] = ((x & 0xffff) << 16) | (y & 0xffff);
    message.xclient.data.l[3] = time;
    message.xclient.data.l[4] = x11_->dndActionCopy();

    XSendEvent(x11_->display(), dragDropReceiver(target), False, 0, &message);
    XFlush(x11_->display());
  }

  ::Window WindowX11::dragDropProxy(::Window window) const {
    Atom actual_type;
    int actual_format;
    unsigned long num_items = 0, bytes_after = 0;
    unsigned char* proxy_data = nullptr;
    XGetWindowProperty(x11_->display(), window, x11_->dndProxy(), 0, ~0, False, AnyPropertyType,
                       &actual_type, &actual_format, &num_items, &bytes_after, &proxy_data);
    ::Window result = 0;
    if (proxy_data && num_items * actual_format == 32)
      result = reinterpret_cast<long*>(proxy_data)[0];
    if (proxy_data)
      XFree(proxy_data);
    return result;
  }

  void WindowX11::sendDragDropStatus(::Window source, ::Window target, bool accept_drag) const {
    ::Window receiver = dragDropReceiver(target);

    XEvent message { 0 };
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_->display();
    message.xclient.window = receiver;
    message.xclient.message_type = x11_->dndStatus();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    message.xclient.data.l[2] = 1;
    message.xclient.data.l[3] = 0;
    if (accept_drag) {
      message.xclient.data.l[1] = 1;
      message.xclient.data.l[4] = x11_->dndActionCopy();
    }
    else {
      message.xclient.data.l[1] = 0;
      message.xclient.data.l[4] = x11_->dndActionNone();
    }

    XSendEvent(x11_->display(), receiver, False, NoEventMask, &message);
  }

  void WindowX11::sendDragDropDrop(::Window source, ::Window target) const {
    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_->display();
    message.xclient.window = target;
    message.xclient.message_type = x11_->dndDrop();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    message.xclient.data.l[2] = CurrentTime;

    XSendEvent(x11_->display(), dragDropReceiver(target), False, 0, &message);
  }

  void WindowX11::sendDragDropSelectionNotify(XSelectionRequestEvent* request) const {
    X11Connection::DisplayLock lock(x11_);

    XSelectionEvent result = { 0 };
    result.type = SelectionNotify;
    result.display = request->display;
    result.requestor = request->requestor;
    result.selection = request->selection;
    result.time = request->time;
    result.target = request->target;
    result.property = None;

    if (request->target == x11_->targets()) {
      Atom supported_types[] = { x11_->dndUriList() };
      XChangeProperty(request->display, request->requestor, request->property, XA_ATOM, 32,
                      PropModeReplace, (unsigned char*)supported_types, 1);
      result.property = request->property;
    }
    else if (request->target == x11_->dndUriList()) {
      std::string selection_data;
      for (const auto& drag_drop_file : drag_drop_files_)
        selection_data += fileUri(drag_drop_file) + "\r\n";

      XChangeProperty(request->display, request->requestor, request->property, request->target, 8,
                      PropModeReplace, (unsigned char*)selection_data.c_str(), selection_data.length());
      result.property = request->property;
    }

    XSendEvent(request->display, result.requestor, 0, NoEventMask, (XEvent*)&result);
  }

  void WindowX11::sendDragDropFinished(::Window source, ::Window target, bool accepted_drag) const {
    ::Window receiver = dragDropReceiver(target);

    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_->display();
    message.xclient.window = receiver;
    message.xclient.message_type = x11_->dndFinished();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    if (accepted_drag) {
      message.xclient.data.l[1] = 1;
      message.xclient.data.l[2] = x11_->dndActionCopy();
    }
    else {
      message.xclient.data.l[1] = 0;
      message.xclient.data.l[2] = x11_->dndActionNone();
    }

    XSendEvent(x11_->display(), receiver, False, NoEventMask, &message);
  }

  void WindowX11::processPluginFdEvents() {
    bool timer_fired = false;
    XEvent event;
    while (XPending(x11_->display())) {
      XNextEvent(x11_->display(), &event);
      if (XFilterEvent(&event, None))
        continue;

      if (event.xany.window == parent_handle_ && event.type == ConfigureNotify) {
        XWindowAttributes attributes;
        Status status;
        {
          X11Connection::DisplayLock lock(x11_);
          status = XGetWindowAttributes(x11_->display(), parent_handle_, &attributes);
        }
        if (status)
          setNativeWindowSize(attributes.width, attributes.height);
      }
      else if (event.xany.window == window_handle_ && event.type == ClientMessage &&
               event.xclient.message_type == x11_->timerEvent()) {
        clearTick();
        if (!timer_fired) {
          timer_fired = true;
          drawCallback(drawTime());
        }
      }
      else if (event.xany.window == window_handle_ || event.xany.window == parent_handle_)
        processEvent(event);
    }
  }

  void WindowX11::processMessageWindowEvent(X11Connection* x11, XEvent& event) {
    switch (event.type) {
    case SelectionRequest: {
      X11Connection::DisplayLock lock(x11);
      XSelectionRequestEvent* request = &event.xselectionrequest;

      XSelectionEvent result = { 0 };
      result.type = SelectionNotify;
      result.display = request->display;
      result.requestor = request->requestor;
      result.selection = request->selection;
      result.time = request->time;
      result.target = request->target;
      result.property = None;

      if (request->target == x11->targets()) {
        Atom supported_types[] = { x11->targets(), x11->utf8String(), XA_STRING };
        XChangeProperty(request->display, request->requestor, request->property, XA_ATOM, 32,
                        PropModeReplace, (unsigned char*)supported_types, 3);
        result.property = request->property;
      }
      else if (request->target == x11->utf8String() || request->target == XA_STRING) {
        std::string text = request->target == XA_STRING ? utf8ToLatin1(_clipboard_text) : _clipboard_text;
        XChangeProperty(request->display, request->requestor, request->property, request->target, 8,
                        PropModeReplace, (unsigned char*)text.c_str(), text.length());
        result.property = request->property;
      }

      XSendEvent(request->display, result.requestor, 0, NoEventMask, (XEvent*)&result);
      break;
    }
    }
  }

  void WindowX11::setDragDropFiles(std::string uri_list) {
    if (!uri_list.empty() && uri_list.back() == 0)
      uri_list.pop_back();

    std::stringstream stream(uri_list);
    std::string line;
    drag_drop_files_.clear();
    while (std::getline(stream, line)) {
      std::string trimmed = String(line).trim().toUtf8();
      if (!trimmed.empty() && trimmed[0] != '#')
        drag_drop_files_.push_back(filePath(trimmed));
    }
  }

  static bool changesFocus(const XFocusChangeEvent& event) {
    return event.mode != NotifyGrab && event.mode != NotifyUngrab && event.detail != NotifyPointer;
  }

  void WindowX11::processEvent(XEvent& event) {
    switch (event.type) {
    case ClientMessage: {
      if (event.xclient.message_type == x11_->dndEnter()) {
        X11Connection::DisplayLock lock(x11_);
        drag_drop_files_.clear();
        drag_drop_incremental_ = false;
        drag_drop_data_.clear();
        XConvertSelection(x11_->display(), x11_->dndSelection(), x11_->dndUriList(),
                          x11_->dndUriList(), window_handle_, CurrentTime);
        XFlush(x11_->display());
        sendDragDropStatus(event.xclient.window, event.xclient.data.l[0], false);
      }
      else if (event.xclient.message_type == x11_->dndLeave()) {
        handleFileDragLeave();
      }
      else if (event.xclient.message_type == x11_->dndDrop()) {
        bool success = handleFileDrop(drag_drop_target_x_, drag_drop_target_y_, drag_drop_files_);
        sendDragDropFinished(event.xclient.window, event.xclient.data.l[0], success);
      }
      else if (event.xclient.message_type == x11_->dndPosition()) {
        int win_x = 0;
        int win_y = 0;
        ::Window child_return;
        XTranslateCoordinates(x11_->display(), window_handle_, x11_->rootWindow(), 0, 0, &win_x,
                              &win_y, &child_return);

        drag_drop_target_x_ = static_cast<short>(event.xclient.data.l[2] >> 16) - win_x;
        drag_drop_target_y_ = static_cast<short>(event.xclient.data.l[2] & 0xffff) - win_y;
        bool accepts = handleFileDrag(drag_drop_target_x_, drag_drop_target_y_, drag_drop_files_);
        sendDragDropStatus(event.xclient.window, event.xclient.data.l[0], accepts);
      }
      else if (event.xclient.message_type == x11_->dndStatus()) {
        if (event.xclient.data.l[0] == drag_drop_out_state_.target)
          drag_drop_out_state_.accepted = event.xclient.data.l[1] & 1;
      }
      else if (event.xclient.message_type == x11_->dndFinished()) {
        cleanupDragDropSource();
        drag_drop_out_state_ = {};
        setCursorStyle(MouseCursor::Arrow);
      }
      break;
    }
    case SelectionRequest: {
      X11Connection::DisplayLock lock(x11_);
      XSelectionRequestEvent* request = &event.xselectionrequest;
      if (request->selection == x11_->dndSelection())
        sendDragDropSelectionNotify(request);
      else
        processMessageWindowEvent(x11_, event);
      break;
    }
    case SelectionNotify: {
      X11Connection::DisplayLock lock(x11_);
      if (event.xselection.selection == x11_->dndSelection() && event.xselection.property) {
        std::string uri_list;
        Atom type = takeWindowProperty(x11_->display(), event.xselection.requestor,
                                       event.xselection.property, uri_list);
        drag_drop_incremental_ = type == x11_->incr();
        if (!drag_drop_incremental_ && type != None)
          setDragDropFiles(uri_list);
      }
      break;
    }
    case PropertyNotify: {
      if (!drag_drop_incremental_ || event.xproperty.atom != x11_->dndUriList() ||
          event.xproperty.state != PropertyNewValue)
        break;

      X11Connection::DisplayLock lock(x11_);
      std::string chunk;
      if (takeWindowProperty(x11_->display(), window_handle_, event.xproperty.atom, chunk) == None)
        break;

      if (chunk.empty()) {
        drag_drop_incremental_ = false;
        setDragDropFiles(drag_drop_data_);
        drag_drop_data_.clear();
      }
      else
        drag_drop_data_ += chunk;
      break;
    }
    case MotionNotify: {
      last_active_window_ = this;

      if (window_operation_ & kMoveWindow) {
        X11Connection::DisplayLock lock(x11_);
        int x_offset = event.xmotion.x_root - dragging_window_position_.x;
        int y_offset = event.xmotion.y_root - dragging_window_position_.y;
        XMoveWindow(x11_->display(), window_handle_, x_offset, y_offset);
      }
      else if (window_operation_) {
        X11Connection::DisplayLock lock(x11_);
        ::Window root;
        int window_x, window_y;
        unsigned int window_width, window_height, border, depth;
        ::Window child;
        if (!XGetGeometry(x11_->display(), window_handle_, &root, &window_x, &window_y,
                          &window_width, &window_height, &border, &depth) ||
            !XTranslateCoordinates(x11_->display(), window_handle_, x11_->rootWindow(), 0, 0,
                                   &window_x, &window_y, &child)) {
          break;
        }
        int window_right = window_x + window_width;
        int window_bottom = window_y + window_height;

        if (window_operation_ & kResizeLeft)
          window_x = std::min(window_right - kMinWidth, event.xmotion.x_root);
        else if (window_operation_ & kResizeRight)
          window_right = std::max(window_x + kMinHeight, event.xmotion.x_root);
        if (window_operation_ & kResizeTop)
          window_y = std::min(window_bottom - kMinHeight, event.xmotion.y_root);
        else if (window_operation_ & kResizeBottom)
          window_bottom = std::max(window_y + kMinHeight, event.xmotion.y_root);

        XMoveResizeWindow(x11_->display(), window_handle_, window_x, window_y,
                          window_right - window_x, window_bottom - window_y);
      }

      int last_hover_operation = hover_window_operation_;
      hover_window_operation_ = resizeOperationForPosition(event.xmotion.x, event.xmotion.y);
      if (hover_window_operation_)
        setCursorStyle(windowResizeCursor(hover_window_operation_));
      else if (last_hover_operation)
        setCursorStyle(MouseCursor::Arrow);

      if (drag_drop_out_state_.dragging) {
        ::Window last_target = drag_drop_out_state_.target;
        if (event.xmotion.x >= 0 && event.xmotion.x < clientWidth() && event.xmotion.y >= 0 &&
            event.xmotion.y < clientHeight()) {
          drag_drop_out_state_.target = 0;
          handleFileDrag(event.xmotion.x, event.xmotion.y, drag_drop_files_);
        }
        else {
          if (drag_drop_out_state_.target == 0)
            handleFileDragLeave();

          drag_drop_out_state_.target = dragDropTarget();
        }
        if (last_target != drag_drop_out_state_.target) {
          drag_drop_out_state_.accepted = false;
          if (last_target)
            sendDragDropLeave(window_handle_, last_target);
          if (drag_drop_out_state_.target)
            sendDragDropEnter(window_handle_, drag_drop_out_state_.target);
        }
        if (drag_drop_out_state_.target)
          sendDragDropPosition(window_handle_, drag_drop_out_state_.target, event.xmotion.x_root,
                               event.xmotion.y_root, event.xmotion.time);
        break;
      }
      if (mouseRelativeMode() && mouse_down_position_ == IPoint(event.xmotion.x, event.xmotion.y))
        break;

      if (window_operation_ == 0) {
        handleMouseMove(event.xmotion.x, event.xmotion.y, mouseButtonState(event.xmotion.state),
                        modifierState(event.xmotion.state));
        if (mouseRelativeMode())
          setNativeCursorPosition(mouse_down_position_);
      }
      break;
    }
    case ButtonPress: {
      if (event.xbutton.button >= 4 && event.xbutton.button <= 7) {
        float y = (event.xbutton.button == 4 ? 1.0f : 0.0f) - (event.xbutton.button == 5 ? 1.0f : 0.0f);
        float x = (event.xbutton.button == 7 ? 1.0f : 0.0f) - (event.xbutton.button == 6 ? 1.0f : 0.0f);
        handleMouseWheel(x, y, event.xbutton.x, event.xbutton.y,
                         mouseButtonState(event.xbutton.state), modifierState(event.xbutton.state));
      }
      else {
        MouseButton button = buttonFromEvent(event);
        if (button == kMouseButtonNone)
          passEventToParent(event);
        else
          handleMouseDown(button, event.xbutton.x, event.xbutton.y,
                          mouseButtonState(event.xbutton.state) | button,
                          modifierState(event.xbutton.state));

        HitTestResult hit_test = handleHitTest(event.xbutton.x, event.xbutton.y);

        window_operation_ = resizeOperationForPosition(event.xbutton.x, event.xbutton.y);
        if (hit_test == HitTestResult::TitleBar && window_operation_ == 0) {
          ::Window child;
          int window_x, window_y;
          if (XTranslateCoordinates(x11_->display(), window_handle_, x11_->rootWindow(), 0, 0,
                                    &window_x, &window_y, &child)) {
            window_operation_ = kMoveWindow;
            dragging_window_position_ = { event.xbutton.x_root - window_x, event.xbutton.y_root - window_y };
          }
        }

        mouse_down_position_ = { event.xbutton.x, event.xbutton.y };

        drag_drop_out_state_.dragging = isDragDropSource();
        if (drag_drop_out_state_.dragging) {
          XSetSelectionOwner(x11_->display(), x11_->dndSelection(), window_handle_, event.xmotion.time);

          setCursorStyle(MouseCursor::MultiDirectionalResize);
          drag_drop_files_.clear();
          drag_drop_files_.push_back(startDragDropSource());
        }
      }
      break;
    }
    case ButtonRelease: {
      window_operation_ = 0;
      MouseButton button = buttonFromEvent(event);
      HitTestResult hit_test = currentHitTest();
      if (drag_drop_out_state_.dragging && button == kMouseButtonLeft) {
        bool inside = event.xbutton.x >= 0 && event.xbutton.x < clientWidth() &&
                      event.xbutton.y >= 0 && event.xbutton.y < clientHeight();
        if (!inside && drag_drop_out_state_.target && drag_drop_out_state_.accepted) {
          sendDragDropDrop(window_handle_, drag_drop_out_state_.target);
          drag_drop_out_state_.dragging = false;
        }
        else {
          if (drag_drop_out_state_.target)
            sendDragDropLeave(window_handle_, drag_drop_out_state_.target);
          if (inside)
            handleFileDrop(event.xbutton.x, event.xbutton.y, drag_drop_files_);
          cleanupDragDropSource();
          drag_drop_out_state_ = {};
          setCursorStyle(MouseCursor::Arrow);
        }
      }
      if (button == kMouseButtonNone)
        passEventToParent(event);
      else
        handleMouseUp(button, event.xbutton.x, event.xbutton.y,
                      mouseButtonState(event.xbutton.state) & ~button,
                      modifierState(event.xbutton.state));

      if (button == kMouseButtonLeft) {
        if (hit_test == HitTestResult::CloseButton &&
            handleHitTest(event.xbutton.x, event.xbutton.y) == HitTestResult::CloseButton) {
          close();
        }
        else if (hit_test == HitTestResult::MaximizeButton &&
                 handleHitTest(event.xbutton.x, event.xbutton.y) == HitTestResult::MaximizeButton) {
          sendMaximizeState(x11_, window_handle_, kNetWmStateToggle);
        }
        else if (hit_test == HitTestResult::MinimizeButton &&
                 handleHitTest(event.xbutton.x, event.xbutton.y) == HitTestResult::MinimizeButton) {
          Atom wm_change_state = XInternAtom(x11_->display(), "WM_CHANGE_STATE", False);
          XEvent min_event = {};
          min_event.xclient.type = ClientMessage;
          min_event.xclient.message_type = wm_change_state;
          min_event.xclient.display = x11_->display();
          min_event.xclient.window = window_handle_;
          min_event.xclient.format = 32;
          min_event.xclient.data.l[0] = IconicState;

          XSendEvent(x11_->display(), x11_->rootWindow(), False,
                     SubstructureRedirectMask | SubstructureNotifyMask, &min_event);
          XFlush(x11_->display());
        }
      }
      break;
    }
    case FocusIn: {
      if (input_context_)
        XSetICFocus(input_context_);
      if (changesFocus(event.xfocus))
        handleFocusGained();
      break;
    }
    case FocusOut: {
      if (input_context_)
        XUnsetICFocus(input_context_);
      if (changesFocus(event.xfocus))
        handleFocusLost();
      break;
    }
    case EnterNotify: {
      handleMouseEnter(event.xcrossing.x, event.xcrossing.y);
      break;
    }
    case LeaveNotify: {
      handleMouseLeave(mouseButtonState(event.xcrossing.state), modifierState(event.xcrossing.state));
      break;
    }
    case KeyPress: {
      int modifier_state = modifierState(event.xkey.state);
      std::string text = lookupText(input_context_, &event.xkey);
      KeySym keysym = lookupKeysym(&event.xkey);
      bool repeat = pressed_[keysym];
      pressed_[keysym] = true;
      KeyCode key_code = translateKeyCode(keysym);
      bool used = false;
      if (key_code != KeyCode::Unknown)
        used = handleKeyDown(key_code, modifier_state | modifierForKeyCode(key_code), repeat);

      bool shortcut = modifier_state & (kModifierRegCtrl | kModifierAlt | kModifierMeta);
      if (!shortcut && !text.empty() && text[0] < 127)
        handleTextInput(text);

      if (!used)
        passEventToParent(event);
      break;
    }
    case KeyRelease: {
      KeySym keysym = lookupKeysym(&event.xkey);
      pressed_[keysym] = false;
      KeyCode key_code = translateKeyCode(keysym);
      bool used = false;
      if (key_code != KeyCode::Unknown)
        used = handleKeyUp(key_code, modifierState(event.xkey.state) & ~modifierForKeyCode(key_code));

      if (!used)
        passEventToParent(event);
      break;
    }
    case ConfigureNotify: {
      updateMonitor(event.xconfigure.width, event.xconfigure.height);
      IPoint dimensions = retrieveWindowDimensions();
      if (dimensions.x != clientWidth() || dimensions.y != clientHeight()) {
        handleResized(dimensions.x, dimensions.y);
        drawCallback(drawTime());
      }
      break;
    }
    case VisibilityNotify: {
      setVisible(event.xvisibility.state != VisibilityFullyObscured);
      break;
    }
    case MapNotify:
    case UnmapNotify: {
      X11Connection::DisplayLock lock(x11_);
      XWindowAttributes attributes;
      setVisible(XGetWindowAttributes(x11_->display(), window_handle_, &attributes) &&
                 attributes.map_state == IsViewable);
      break;
    }
    }
  }

  void WindowX11::runEventLoop() {
    X11Connection* x11 = x11_;
    Display* display = x11->display();

    fd_set read_fds;
    unsigned int fd = ConnectionNumber(display);

    XEvent event;
    bool running = true;
    while (running) {
      FD_ZERO(&read_fds);
      FD_SET(fd, &read_fds);

      int result = 0;
      if (!XPending(display))
        result = select(fd + 1, &read_fds, nullptr, nullptr, nullptr);

      if (result == -1 && errno != EINTR)
        running = false;

      while (running && XPending(display)) {
        XNextEvent(display, &event);
        if (XFilterEvent(&event, None))
          continue;

        WindowX11* window = NativeWindowLookup::instance().findWindow(event.xany.window);
        if (window == nullptr) {
          if (event.type == SelectionRequest)
            processMessageWindowEvent(x11, event);
          continue;
        }

        if (event.type == Expose) {
          int height = window->clientHeight();
          if (window->decoration_ != Decoration::Native)
            window->handleResized(window->clientWidth(), height + 1);
          window->handleResized(window->clientWidth(), height);
        }

        if (event.type == ClientMessage && event.xclient.message_type == x11->timerEvent()) {
          window->clearTick();
          window->drawCallback(window->drawTime());
          continue;
        }

        if (event.type == DestroyNotify ||
            (event.type == ClientMessage && event.xclient.data.l[0] == x11->deleteMessage())) {
          if (window->handleCloseRequested()) {
            NativeWindowLookup::instance().removeWindow(window);
            window->hide();
            if (!NativeWindowLookup::instance().anyWindowOpen())
              running = false;
          }
        }
        else
          window->processEvent(event);
      }
    }
  }

  void WindowX11::windowContentsResized(int width, int height) {
    X11Connection::DisplayLock lock(x11_);
    XResizeWindow(x11_->display(), window_handle_, width, height);
    XSync(x11_->display(), False);
  }

  void WindowX11::show() {
    setMinimumSizeHints();

    {
      X11Connection::DisplayLock lock(x11_);
      XMapWindow(x11_->display(), window_handle_);
      XSetWMProtocols(x11_->display(), window_handle_, x11_->deleteMessageRef(), 1);
      XFlush(x11_->display());
    }
    handleWindowShown();
  }

  void WindowX11::showMaximized() {
    show();
    sendMaximizeState(x11_, window_handle_, kNetWmStateAdd);

    for (int retries = 0; retries < 10; ++retries) {
      Thread::sleep(10);
      XEvent e;
      if (XCheckTypedWindowEvent(x11_->display(), window_handle_, ConfigureNotify, &e)) {
        visage::IPoint dimensions = retrieveWindowDimensions();
        handleResized(dimensions.x, dimensions.y);
        return;
      }
    }
  }

  void WindowX11::hide() {
    {
      X11Connection::DisplayLock lock(x11_);
      ::Display* display = x11_->display();
      XUnmapWindow(display, window_handle_);
      XFlush(display);
    }
    handleWindowHidden();
  }

  void WindowX11::close() {
    XEvent close_event = {};
    close_event.xclient.type = ClientMessage;
    close_event.xclient.window = window_handle_;
    close_event.xclient.message_type = XInternAtom(x11_->display(), "WM_PROTOCOLS", True);
    close_event.xclient.format = 32;
    close_event.xclient.data.l[0] = x11_->deleteMessage();
    close_event.xclient.data.l[1] = CurrentTime;

    XSendEvent(x11_->display(), window_handle_, False, NoEventMask, &close_event);
    XFlush(x11_->display());
  }

  bool WindowX11::isShowing() const {
    X11Connection::DisplayLock lock(x11_);
    XWindowAttributes attributes;
    if (XGetWindowAttributes(x11_->display(), window_handle_, &attributes) == 0)
      return false;
    if (attributes.map_state == IsViewable)
      return true;

    Atom wm_state = XInternAtom(x11_->display(), "WM_STATE", False);
    Atom actual_type;
    int actual_format = 0;
    unsigned long num_items = 0, bytes_after = 0;
    unsigned char* state = nullptr;
    XGetWindowProperty(x11_->display(), window_handle_, wm_state, 0, 1, False, wm_state,
                       &actual_type, &actual_format, &num_items, &bytes_after, &state);
    bool iconic = state && num_items && reinterpret_cast<long*>(state)[0] == IconicState;
    if (state)
      XFree(state);
    return iconic;
  }

  bool WindowX11::isClosed() const {
    return NativeWindowLookup::instance().findWindow(window_handle_) != this;
  }

  void WindowX11::setWindowTitle(const std::string& title) {
    X11Connection::DisplayLock lock(x11_);
    setWindowName(x11_, window_handle_, title);
  }

  void WindowX11::setAlwaysOnTop(bool on_top) {
    if (parent_handle_)
      return;

    X11Connection::DisplayLock lock(x11_);
    Display* display = x11_->display();
    Atom wm_state = XInternAtom(display, "_NET_WM_STATE", False);
    Atom wm_state_above = XInternAtom(display, "_NET_WM_STATE_ABOVE", False);

    if (isShowing()) {
      XEvent event = {};
      event.type = ClientMessage;
      event.xclient.window = window_handle_;
      event.xclient.message_type = wm_state;
      event.xclient.format = 32;
      event.xclient.data.l[0] = on_top ? 1 : 0;
      event.xclient.data.l[1] = wm_state_above;
      event.xclient.data.l[3] = 1;
      XSendEvent(display, x11_->rootWindow(), False,
                 SubstructureRedirectMask | SubstructureNotifyMask, &event);
    }
    else {
      std::vector<Atom> states;
      Atom type = None;
      int format = 0;
      unsigned long count = 0, remaining = 0;
      unsigned char* data = nullptr;
      if (XGetWindowProperty(display, window_handle_, wm_state, 0, 64, False, XA_ATOM, &type,
                             &format, &count, &remaining, &data) == Success &&
          data) {
        Atom* atoms = reinterpret_cast<Atom*>(data);
        for (unsigned long i = 0; i < count; ++i) {
          if (atoms[i] != wm_state_above)
            states.push_back(atoms[i]);
        }
        XFree(data);
      }
      if (on_top)
        states.push_back(wm_state_above);
      XChangeProperty(display, window_handle_, wm_state, XA_ATOM, 32, PropModeReplace,
                      (unsigned char*)states.data(), states.size());
    }

    XFlush(display);
  }

  IPoint WindowX11::maxWindowDimensions() const {
    MonitorInfo monitor_info = activeMonitorInfo();
    return { monitor_info.bounds.width(), monitor_info.bounds.height() };
  }
}
#endif
