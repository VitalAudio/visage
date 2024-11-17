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

#if VISAGE_LINUX
#include "windowing_x11.h"

#include "visage_utils/thread_utils.h"

#include <cstring>
#include <sstream>
#include <X11/cursorfont.h>
#include <X11/extensions/Xrandr.h>
#include <X11/Xutil.h>

namespace visage {
  static std::string _clipboard_text;

  class DummyWindow {
  public:
    static ::Window handle() {
      static DummyWindow dummy;
      return dummy.window_handle_;
    }

  private:
    DummyWindow() {
      X11Connection& x11 = X11Connection::globalInstance();
      ::Display* display = x11.display();
      window_handle_ = XCreateSimpleWindow(display, x11.rootWindow(), -100, -100, 1, 1, 0, 0, 0);
      XSelectInput(display, window_handle_, StructureNotifyMask);
      XFlush(display);
    }

    ~DummyWindow() { XDestroyWindow(X11Connection::globalInstance().display(), window_handle_); }

    ::Window window_handle_ = 0;
  };

  std::string readClipboardText() {
    static constexpr int kSleepWait = 5;
    static constexpr int kMaxWaitTime = 250;
    static constexpr int kTries = kMaxWaitTime / kSleepWait;
    X11Connection& x11 = X11Connection::globalInstance();
    X11Connection::DisplayLock lock(x11);

    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return "";

    Display* display = x11.display();

    ::Window selection_owner = XGetSelectionOwner(display, x11.clipboard());
    if (selection_owner == DummyWindow::handle())
      return _clipboard_text;

    Atom selection_property = XInternAtom(display, "VISAGE_SELECT", False);
    XConvertSelection(display, x11.clipboard(), x11.utf8String(), selection_property,
                      DummyWindow::handle(), CurrentTime);

    XEvent event;
    for (int i = 0; i < kTries; ++i) {
      if (XCheckTypedWindowEvent(display, DummyWindow::handle(), SelectionNotify, &event)) {
        if (event.xselection.property == selection_property) {
          Atom actual_type;
          int actual_format = 0;
          unsigned long num_items = 0, bytes_after = 0;
          unsigned char* property = nullptr;
          XGetWindowProperty(display, DummyWindow::handle(), selection_property, 0, ~0, False,
                             AnyPropertyType, &actual_type, &actual_format, &num_items,
                             &bytes_after, &property);

          if (actual_type == x11.utf8String()) {
            if (num_items && property[num_items - 1] == 0)
              num_items--;

            std::string result(reinterpret_cast<char*>(property), num_items);
            XFree(property);
            return result;
          }
          XFree(property);
        }
      }
      Thread::sleep(kSleepWait);
    }

    return "";
  }

  void setClipboardText(const std::string& text) {
    _clipboard_text = text;

    X11Connection& x11 = X11Connection::globalInstance();
    XSetSelectionOwner(x11.display(), XA_PRIMARY, DummyWindow::handle(), CurrentTime);
    XSetSelectionOwner(x11.display(), x11.clipboard(), DummyWindow::handle(), CurrentTime);
  }

  void setCursorStyle(MouseCursor style) {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return;

    X11Connection& x11 = window->x11Connection();
    X11Connection::DisplayLock lock(x11);

    Cursor cursor;
    switch (style) {
    case MouseCursor::Invisible: cursor = x11.cursors().no_cursor; break;
    case MouseCursor::Arrow: cursor = x11.cursors().arrow_cursor; break;
    case MouseCursor::IBeam: cursor = x11.cursors().ibeam_cursor; break;
    case MouseCursor::Crosshair: cursor = x11.cursors().crosshair_cursor; break;
    case MouseCursor::Pointing: cursor = x11.cursors().pointing_cursor; break;
    case MouseCursor::HorizontalResize: cursor = x11.cursors().horizontal_resize_cursor; break;
    case MouseCursor::VerticalResize: cursor = x11.cursors().vertical_resize_cursor; break;
    case MouseCursor::Dragging:
    case MouseCursor::MultiDirectionalResize:
      cursor = x11.cursors().multi_directional_resize_cursor;
      break;
    default: return;
    }

    XDefineCursor(x11.display(), (::Window)window->nativeHandle(), cursor);
    XFlush(x11.display());
  }

  void setCursorVisible(bool visible) {
    if (visible)
      setCursorStyle(MouseCursor::Arrow);
    else
      setCursorStyle(MouseCursor::Invisible);
  }

  Point cursorPosition(::Window window_handle) {
    X11Connection& x11 = X11Connection::globalInstance();
    X11Connection::DisplayLock lock(x11);
    ::Window root_return, child_return;
    int root_x = 0, root_y = 0;
    int win_x = 0, win_y = 0;
    unsigned int mask_return = 0;

    XQueryPointer(x11.display(), window_handle, &root_return, &child_return, &root_x, &root_y,
                  &win_x, &win_y, &mask_return);
    return { win_x, win_y };
  }

  Point cursorPosition() {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return { 0, 0 };

    return cursorPosition((::Window)window->nativeHandle());
  }

  void setCursorPosition(Point window_position) {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window == nullptr)
      return;

    X11Connection& x11 = window->x11Connection();
    X11Connection::DisplayLock lock(x11);
    XWarpPointer(x11.display(), None, (::Window)window->nativeHandle(), 0, 0, 0, 0,
                 window_position.x, window_position.y);
    XFlush(x11.display());
  }

  void setCursorScreenPosition(Point window_position) {
    X11Connection& x11 = X11Connection::globalInstance();
    X11Connection::DisplayLock lock(x11);

    XWarpPointer(x11.display(), None, x11.rootWindow(), 0, 0, 0, 0, window_position.x,
                 window_position.y);
    XFlush(x11.display());
  }

  Point cursorScreenPosition() {
    X11Connection& x11 = X11Connection::globalInstance();
    X11Connection::DisplayLock lock(x11);

    ::Window root_return, child_return;
    int root_x = 0, root_y = 0;
    int win_x = 0, win_y = 0;
    unsigned int mask_return = 0;

    XQueryPointer(x11.display(), x11.rootWindow(), &root_return, &child_return, &root_x, &root_y,
                  &win_x, &win_y, &mask_return);
    return { root_x, root_y };
  }

  bool shouldWindowDraw(void* window_handle) {
    return true;
  }

  float windowPixelScale() {
    return 1.0f;
  }

  double refreshRate(XRRScreenResources* screen_resources, XRRCrtcInfo* info) {
    for (int i = 0; i < screen_resources->nmode; ++i) {
      if (screen_resources->modes[i].id == info->mode)
        return screen_resources->modes[i].dotClock * 1.0 /
               (screen_resources->modes[i].hTotal * screen_resources->modes[i].vTotal);
    }
    return MonitorInfo::kDefaultRefreshRate;
  }

  MonitorInfo monitorInfoForPosition(Point point) {
    static constexpr float kInchToMm = 25.4;

    X11Connection& x11 = X11Connection::globalInstance();
    X11Connection::DisplayLock lock(x11);
    Display* display = x11.display();

    Bounds default_bounds(0, 0, DisplayWidth(display, DefaultScreen(display)),
                          DisplayWidth(display, DefaultScreen(display)));
    MonitorInfo result;
    result.bounds = default_bounds;

    XRRScreenResources* screen_resources = XRRGetScreenResources(display, x11.rootWindow());
    if (screen_resources == nullptr)
      return result;

    for (int i = 0; i < screen_resources->noutput; ++i) {
      XRROutputInfo* output_info = XRRGetOutputInfo(display, screen_resources,
                                                    screen_resources->outputs[i]);
      if (output_info == nullptr)
        continue;

      if (output_info->crtc && output_info->connection == RR_Connected) {
        XRRCrtcInfo* info = XRRGetCrtcInfo(display, screen_resources, output_info->crtc);

        Bounds bounds(info->x, info->y, info->width, info->height);
        if (result.bounds.width() == 0 || bounds.contains(point)) {
          result.bounds = bounds;
          result.dpi = bounds.height() * kInchToMm / output_info->mm_height;
          result.refresh_rate = refreshRate(screen_resources, info);
        }
        XRRFreeCrtcInfo(info);
      }
      XRRFreeOutputInfo(output_info);
    }

    XRRFreeScreenResources(screen_resources);
    return result;
  }

  MonitorInfo activeMonitorInfo() {
    return monitorInfoForPosition(cursorScreenPosition());
  }

  Bounds boundsInDisplay(MonitorInfo monitor_info, float aspect_ratio, float display_scale) {
    int scale_width = monitor_info.bounds.width() * display_scale;
    int height = std::min(monitor_info.bounds.height() * display_scale, scale_width / aspect_ratio);
    int width = height * aspect_ratio + 0.5f;
    int x = (monitor_info.bounds.width() - width) / 2;
    int y = (monitor_info.bounds.height() - height) / 2;
    return { monitor_info.bounds.x() + x, monitor_info.bounds.y() + y, width, height };
  }

  void drawMessageBox(int width, int height, Display* display, ::Window window, GC gc,
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

    XUnloadFont(display, font->fid);
  }

  void showMessageBox(std::string title, std::string message) {
    constexpr float kAspectRatio = 1.5f;
    constexpr float kDisplayScale = 0.2f;

    MonitorInfo monitor_info = activeMonitorInfo();
    Bounds bounds = boundsInDisplay(monitor_info, kAspectRatio, kDisplayScale);

    X11Connection& x11 = X11Connection::globalInstance();
    X11Connection::DisplayLock lock(x11);
    Display* display = x11.display();
    int screen = DefaultScreen(display);

    ::Window message_window = XCreateSimpleWindow(display, x11.rootWindow(), bounds.x(), bounds.y(),
                                                  bounds.width(), bounds.height(), 0,
                                                  BlackPixel(display, screen),
                                                  WhitePixel(display, screen));
    XStoreName(display, message_window, title.c_str());
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
    XEvent event;
    while (true) {
      XNextEvent(display, &event);
      if (event.xany.window != message_window)
        continue;

      if (event.type == Expose) {
        drawMessageBox(bounds.width(), bounds.height(), display, message_window, gc, message);
      }
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
      else if (event.type == KeyPress || event.type == DestroyNotify) {
        break;
      }
      else if (event.type == ClientMessage) {
        if (event.xclient.data.l[0] == wm_delete_message)
          break;
      }
    }

    XFreeGC(display, gc);
    XDestroyWindow(display, message_window);
  }

  std::unique_ptr<Window> createWindow(int x, int y, int width, int height, bool popup) {
    return std::make_unique<WindowX11>(x, y, width, height, popup);
  }

  std::unique_ptr<Window> createPluginWindow(int width, int height, void* parent_handle) {
    return std::make_unique<WindowX11>(width, height, parent_handle);
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
    horizontal_resize_cursor = XCreateFontCursor(display, XC_sb_h_double_arrow);
    vertical_resize_cursor = XCreateFontCursor(display, XC_sb_v_double_arrow);
    multi_directional_resize_cursor = XCreateFontCursor(display, XC_fleur);
    XUnlockDisplay(display);
  }

  int displayFps() {
    WindowX11* window = WindowX11::lastActiveWindow();
    if (window)
      return std::round(window->monitorInfo().refresh_rate);
    return MonitorInfo::kDefaultRefreshRate;
  }

  void WindowX11::createWindow(Bounds bounds) {
    VISAGE_ASSERT(bounds.width() && bounds.height());

    ::Display* display = x11_.display();
    int screen = DefaultScreen(display);
    window_handle_ = XCreateSimpleWindow(display, x11_.rootWindow(), bounds.x(), bounds.y(),
                                         bounds.width(), bounds.height(), 0,
                                         BlackPixel(display, screen), BlackPixel(display, screen));
    XStoreName(display, window_handle_, VISAGE_APPLICATION_NAME);

    unsigned char blank = 0;
    XChangeProperty(display, window_handle_, x11_.dndTypeList(), XA_ATOM, 32, PropModeReplace,
                    x11_.dndTypes(), X11Connection::kNumDndTypes);
    XChangeProperty(display, window_handle_, x11_.dndActionList(), XA_ATOM, 32, PropModeReplace,
                    x11_.dndActions(), X11Connection::kNumDndActions);
    XChangeProperty(display, window_handle_, x11_.dndActionDescription(), XA_STRING, 8,
                    PropModeReplace, &blank, 0);
    XChangeProperty(display, window_handle_, x11_.dndAware(), XA_ATOM, 32, PropModeReplace,
                    x11_.dndVersion(), 1);
  }

  WindowX11* WindowX11::last_active_window_ = nullptr;

  Bounds scaledWindowBounds(float aspect_ratio, float display_scale, int x, int y) {
    return boundsInDisplay(activeMonitorInfo(), aspect_ratio, display_scale);
  }

  WindowX11::WindowX11(int x, int y, int width, int height, bool popup) : Window(width, height) {
    monitor_info_ = activeMonitorInfo();
    X11Connection::DisplayLock lock(x11_);
    ::Display* display = x11_.display();
    Bounds bounds(x, y, width, height);
    createWindow(bounds);

    XSizeHints* size_hints = XAllocSizeHints();
    size_hints->flags = USPosition;
    if (x == kNotSet || y == kNotSet) {
      size_hints->x = bounds.x();
      size_hints->y = bounds.y();
    }
    else {
      size_hints->x = x;
      size_hints->y = y;
    }
    XSetWMNormalHints(display, window_handle_, size_hints);
    XFree(size_hints);

    if (popup) {
      XSetWindowAttributes attributes;
      attributes.override_redirect = True;
      XChangeWindowAttributes(display, window_handle_, CWOverrideRedirect, &attributes);
    }

    XSelectInput(display, window_handle_, kEventMask);
    XFlush(display);
    start_draw_microseconds_ = time::microseconds();
    setDpiScale(monitor_info_.dpi / kDefaultDpi);
  }

  static void threadTimerCallback(WindowX11* window) {
    while (window->timerThreadRunning()) {
      Thread::sleep(window->timerMs());

      X11Connection& x11 = window->x11Connection();
      X11Connection::DisplayLock lock(x11);
      ::Window window_handle = (::Window)window->nativeHandle();

      XEvent event;
      memset(&event, 0, sizeof(event));
      event.type = ClientMessage;
      event.xclient.window = window_handle;
      event.xclient.message_type = x11.timerEvent();
      event.xclient.format = 32;
      event.xclient.data.l[0] = 0;

      XSendEvent(x11.display(), window_handle, False, NoEventMask, &event);
      XFlush(x11.display());
    }
  }

  WindowX11::WindowX11(int width, int height, void* parent_handle) : Window(width, height) {
    static constexpr long kEmbedVersion = 0;
    static constexpr long kEmbedMapped = 1;

    monitor_info_ = activeMonitorInfo();
    X11Connection::DisplayLock lock(x11_);
    ::Display* display = x11_.display();

    parent_handle_ = (::Window)parent_handle;
    XSelectInput(display, parent_handle_, StructureNotifyMask);

    createWindow({ 0, 0, width, height });
    Atom atom_embed_info = XInternAtom(display, "_XEMBED_INFO", False);
    long embed_info[2] = { kEmbedVersion, kEmbedMapped };

    XChangeProperty(display, window_handle_, atom_embed_info, atom_embed_info, 32, PropModeReplace,
                    reinterpret_cast<unsigned char*>(embed_info), 2);

    XChangeProperty(display, parent_handle_, x11_.dndAware(), XA_ATOM, 32, PropModeReplace,
                    x11_.dndVersion(), 1);
    XChangeProperty(display, parent_handle_, x11_.dndProxy(), XA_WINDOW, 32, PropModeReplace,
                    (unsigned char*)&window_handle_, 1);
    XReparentWindow(display, window_handle_, parent_handle_, 0, 0);

    XSelectInput(display, window_handle_, kEventMask);
    XFlush(display);

    timer_thread_running_ = true;
    timer_thread_ = std::make_unique<std::thread>(threadTimerCallback, this);
    start_draw_microseconds_ = time::microseconds();
    setDpiScale(monitor_info_.dpi / kDefaultDpi);
  }

  WindowX11::~WindowX11() {
    timer_thread_running_ = false;
    if (timer_thread_ && timer_thread_->joinable())
      timer_thread_->join();

    timer_thread_.reset();

    X11Connection::DisplayLock lock(x11_);
    if (window_handle_)
      XDestroyWindow(x11_.display(), window_handle_);
  }

  void* WindowX11::initWindow() const {
    return (void*)DummyWindow::handle();
  }

  void WindowX11::setFixedAspectRatio(bool fixed) {
    Window::setFixedAspectRatio(fixed);
    XSizeHints* size_hints = XAllocSizeHints();
    size_hints->flags = fixed ? PAspect : 0;

    ::Window handle = parent_handle_ ? parent_handle_ : window_handle_;

    long supplied_return;
    XGetWMNormalHints(x11_.display(), handle, size_hints, &supplied_return);
    size_hints->flags = fixed ? (size_hints->flags | PAspect) : (size_hints->flags & ~PAspect);
    size_hints->min_aspect.x = clientWidth();
    size_hints->min_aspect.y = clientHeight();
    size_hints->max_aspect.x = clientWidth();
    size_hints->max_aspect.y = clientHeight();
    XSetWMNormalHints(x11_.display(), handle, size_hints);
    XFree(size_hints);
  }

  visage::Point WindowX11::retrieveWindowDimensions() {
    X11Connection::DisplayLock lock(x11_);
    XWindowAttributes attributes;
    XGetWindowAttributes(x11_.display(), window_handle_, &attributes);

    return { attributes.width, attributes.height };
  }

  void WindowX11::passEventToParent(XEvent& event) {
    X11Connection::DisplayLock lock(x11_);

    ::Window root = 0, parent = 0;
    ::Window* children = nullptr;
    unsigned int num_children = 0;
    Display* display = x11_.display();

    if (XQueryTree(display, window_handle_, &root, &parent, &children, &num_children) == 0)
      return;

    if (children)
      XFree(children);

    event.xany.window = parent;
    XSendEvent(display, parent, False, NoEventMask, &event);
    XFlush(display);
  }

  int WindowX11::mouseButtonState() const {
    X11Connection::DisplayLock lock(x11_);

    ::Window root_return = 0, child_return = 0;
    int root_x = 0, root_y = 0, win_x = 0, win_y = 0;
    unsigned int mask_return = 0;

    XQueryPointer(x11_.display(), window_handle_, &root_return, &child_return, &root_x, &root_y,
                  &win_x, &win_y, &mask_return);
    int result = 0;
    if (mask_return & Button1Mask)
      result = result | kMouseButtonLeft;
    if (mask_return & Button2Mask)
      result = result | kMouseButtonMiddle;
    if (mask_return & Button3Mask)
      result = result | kMouseButtonRight;
    return result;
  }

  int WindowX11::modifierState() const {
    X11Connection::DisplayLock lock(x11_);

    ::Window root_return, child_return;
    int root_x, root_y, win_x, win_y;
    unsigned int mask_return;

    XQueryPointer(x11_.display(), window_handle_, &root_return, &child_return, &root_x, &root_y,
                  &win_x, &win_y, &mask_return);
    int result = 0;
    if (mask_return & ShiftMask)
      result = result | kModifierShift;
    if (mask_return & ControlMask)
      result = result | kModifierRegCtrl;
    if (mask_return & Mod1Mask)
      result = result | kModifierAlt;
    if (mask_return & Mod4Mask)
      result = result | kModifierMeta;
    return result;
  }

  MouseButton buttonFromEvent(XEvent& event) {
    if (event.xbutton.button == Button1)
      return kMouseButtonLeft;
    if (event.xbutton.button == Button2)
      return kMouseButtonMiddle;
    if (event.xbutton.button == Button3)
      return kMouseButtonRight;
    return kMouseButtonNone;
  }

  KeyCode translateKeyCode(KeySym keysym) {
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
    default: return KeyCode::Unknown;
    }
  }

  ::Window WindowX11::windowUnderCursor(::Window inside) {
    ::Window root_return, child_return;
    int root_x, root_y, win_x, win_y;
    unsigned int mask_return;
    if (XQueryPointer(x11_.display(), inside, &root_return, &child_return, &root_x, &root_y, &win_x,
                      &win_y, &mask_return)) {
      if (child_return != None)
        return windowUnderCursor(child_return);
    }

    return inside;
  }

  ::Window WindowX11::windowUnderCursor() {
    return windowUnderCursor(x11_.rootWindow());
  }

  void WindowX11::sendDragDropEnter(::Window source, ::Window target) const {
    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_.display();
    message.xclient.window = target;
    message.xclient.message_type = x11_.dndEnter();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    message.xclient.data.l[1] = X11Connection::kDndVersion << 24;
    message.xclient.data.l[2] = x11_.dndType(0);
    message.xclient.data.l[3] = None;
    message.xclient.data.l[4] = None;

    XSendEvent(x11_.display(), target, False, 0, &message);
  }

  void WindowX11::sendDragDropLeave(::Window source, ::Window target) const {
    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_.display();
    message.xclient.window = target;
    message.xclient.message_type = x11_.dndLeave();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;

    XSendEvent(x11_.display(), target, False, 0, &message);
  }

  void WindowX11::sendDragDropPosition(::Window source, ::Window target, int x, int y,
                                       unsigned long time) const {
    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_.display();
    message.xclient.window = target;
    message.xclient.message_type = x11_.dndPosition();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    message.xclient.data.l[2] = x << 16 | y;
    message.xclient.data.l[3] = time;
    message.xclient.data.l[4] = x11_.dndActionCopy();

    XSendEvent(x11_.display(), drag_drop_out_state_.target, False, 0, &message);
  }

  ::Window WindowX11::dragDropProxy(::Window window) const {
    Atom actual_type;
    int actual_format;
    unsigned long num_items = 0, bytes_after = 0;
    unsigned char* proxy_data = nullptr;
    XGetWindowProperty(x11_.display(), window, x11_.dndProxy(), 0, ~0, False, AnyPropertyType,
                       &actual_type, &actual_format, &num_items, &bytes_after, &proxy_data);
    if (proxy_data && num_items * actual_format == 32)
      return ((long*)proxy_data)[0];
    return 0;
  }

  void WindowX11::sendDragDropStatus(::Window source, ::Window target, bool accept_drag) const {
    ::Window proxy = dragDropProxy(target);
    ::Window receiver = proxy ? proxy : target;

    XEvent message { 0 };
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_.display();
    message.xclient.window = receiver;
    message.xclient.message_type = x11_.dndStatus();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    message.xclient.data.l[2] = 0;
    message.xclient.data.l[3] = 0;
    if (accept_drag) {
      message.xclient.data.l[1] = 1;
      message.xclient.data.l[4] = x11_.dndActionCopy();
    }
    else {
      message.xclient.data.l[1] = 0;
      message.xclient.data.l[4] = x11_.dndActionNone();
    }

    XSendEvent(x11_.display(), receiver, False, NoEventMask, &message);
  }

  void WindowX11::sendDragDropDrop(::Window source, ::Window target) const {
    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_.display();
    message.xclient.window = target;
    message.xclient.message_type = x11_.dndDrop();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    message.xclient.data.l[2] = CurrentTime;

    XSendEvent(x11_.display(), target, False, 0, &message);
  }

  void WindowX11::sendDragDropSelectionNotify(XSelectionRequestEvent* request) const {
    std::string selection_data;
    for (int i = 0; i < drag_drop_files_.size(); ++i) {
      selection_data += "file://" + drag_drop_files_[i];
      if (i < drag_drop_files_.size() - 1)
        selection_data += "\n";
    }

    XChangeProperty(x11_.display(), request->requestor, request->property, x11_.dndType(0), 8,
                    PropModeReplace, (unsigned char*)selection_data.c_str(), selection_data.size() + 1);

    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xselection.type = SelectionNotify;
    message.xselection.display = x11_.display();
    message.xselection.requestor = request->requestor;
    message.xselection.selection = request->selection;
    message.xselection.target = request->target;
    message.xselection.property = request->property;
    message.xselection.time = request->time;

    XSendEvent(x11_.display(), request->requestor, False, 0, &message);
  }

  void WindowX11::sendDragDropFinished(::Window source, ::Window target, bool accepted_drag) const {
    ::Window proxy = dragDropProxy(target);
    ::Window receiver = proxy ? proxy : target;

    XEvent message;
    memset(&message, 0, sizeof(message));
    message.xclient.type = ClientMessage;
    message.xclient.display = x11_.display();
    message.xclient.window = receiver;
    message.xclient.message_type = x11_.dndFinished();
    message.xclient.format = 32;
    message.xclient.data.l[0] = source;
    if (accepted_drag) {
      message.xclient.data.l[1] = 1;
      message.xclient.data.l[2] = x11_.dndActionCopy();
    }
    else {
      message.xclient.data.l[1] = 0;
      message.xclient.data.l[2] = x11_.dndActionNone();
    }

    XSendEvent(x11_.display(), receiver, False, NoEventMask, &message);
  }

  void WindowX11::processPluginFdEvents() {
    bool timer_fired = false;
    XEvent event;
    while (XPending(x11_.display())) {
      XNextEvent(x11_.display(), &event);

      if (event.xany.window == parent_handle_ && event.type == ConfigureNotify) {
        X11Connection::DisplayLock lock(x11_);
        XWindowAttributes attributes;
        XGetWindowAttributes(x11_.display(), parent_handle_, &attributes);
        setWindowSize(attributes.width, attributes.height);
      }
      else if (event.xany.window == window_handle_ && event.type == ClientMessage &&
               event.xclient.message_type == x11_.timerEvent()) {
        if (!timer_fired) {
          timer_fired = true;
          long long microseconds = time::microseconds() - start_draw_microseconds_;
          drawCallback(microseconds / 1000000.0);
        }
      }
      else if (event.xany.window == window_handle_ || event.xany.window == parent_handle_)
        processEvent(event);
    }
  }

  void WindowX11::processMessageWindowEvent(XEvent& event) {
    switch (event.type) {
    case SelectionRequest: {
      X11Connection& x11 = X11Connection::globalInstance();
      X11Connection::DisplayLock lock(x11);
      XSelectionRequestEvent* request = &event.xselectionrequest;
      if (request->selection == x11.dndSelection()) {
        sendDragDropSelectionNotify(request);
        break;
      }

      XSelectionEvent result = { 0 };
      result.type = SelectionNotify;
      result.display = request->display;
      result.requestor = request->requestor;
      result.selection = request->selection;
      result.time = request->time;
      result.target = request->target;
      result.property = None;

      if (request->target == x11.targets()) {
        Atom supported_types[] = { x11.utf8String(), XA_STRING };
        XChangeProperty(request->display, request->requestor, request->property, XA_ATOM, 32,
                        PropModeReplace, (unsigned char*)supported_types, 2);
        result.property = request->property;
      }
      else if (request->target == x11.utf8String() || request->target == XA_STRING) {
        XChangeProperty(request->display, request->requestor, request->property, request->target, 8,
                        PropModeReplace, (unsigned char*)_clipboard_text.c_str(),
                        _clipboard_text.length());
        result.property = request->property;
      }

      if (result.property != None)
        XSendEvent(request->display, result.requestor, 0, NoEventMask, (XEvent*)&result);
      break;
    }
    }
  }

  void WindowX11::processEvent(XEvent& event) {
    switch (event.type) {
    case ClientMessage: {
      X11Connection::DisplayLock lock(x11_);
      if (event.xclient.message_type == x11_.dndEnter()) {
        drag_drop_files_.clear();
        XConvertSelection(x11_.display(), x11_.dndSelection(), XA_STRING, x11_.dndSelection(),
                          window_handle_, CurrentTime);
        sendDragDropStatus(event.xclient.window, event.xclient.data.l[0], false);
      }
      else if (event.xclient.message_type == x11_.dndLeave()) {
        handleFileDragLeave();
      }
      else if (event.xclient.message_type == x11_.dndDrop()) {
        Point position = cursorPosition(window_handle_);
        bool accepted = handleFileDrop(position.x, position.y, drag_drop_files_);
        sendDragDropFinished(event.xclient.window, event.xclient.data.l[0], accepted);
      }
      else if (event.xclient.message_type == x11_.dndPosition()) {
        Point position = cursorPosition(event.xclient.window);
        bool accepts = handleFileDrag(position.x, position.y, drag_drop_files_);
        sendDragDropStatus(event.xclient.window, event.xclient.data.l[0], accepts);
      }
      else if (event.xclient.message_type == x11_.dndStatus()) {
      }
      else if (event.xclient.message_type == x11_.dndFinished()) {
        drag_drop_out_state_.target = 0;
        drag_drop_out_state_.dragging = false;
        setCursorStyle(MouseCursor::Arrow);
      }
      break;
    }
    case SelectionNotify: {
      X11Connection::DisplayLock lock(x11_);

      if (event.xselection.selection == x11_.dndSelection()) {
        if (event.xselection.property) {
          const std::string kFilePrefix = "file://";

          Atom actual_type;
          int actual_format;
          unsigned long num_items = 0, bytes_after = 0;
          unsigned char* files_string = nullptr;

          XGetWindowProperty(x11_.display(), event.xany.window, x11_.dndSelection(), 0, ~0, False,
                             AnyPropertyType, &actual_type, &actual_format, &num_items,
                             &bytes_after, &files_string);

          if (files_string) {
            std::vector<std::string> result;
            if (num_items && files_string[num_items - 1] == 0)
              num_items--;

            std::stringstream stream(std::string((char*)files_string, num_items));
            std::string line;

            drag_drop_files_.clear();
            while (std::getline(stream, line)) {
              if (line.substr(0, kFilePrefix.size()) == kFilePrefix)
                drag_drop_files_.push_back(line.substr(kFilePrefix.size()));
              else
                drag_drop_files_.push_back(line);
            }

            XFree(files_string);
          }
        }
      }
      break;
    }
    case MotionNotify: {
      X11Connection::DisplayLock lock(x11_);
      last_active_window_ = this;

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

          drag_drop_out_state_.target = windowUnderCursor();
        }
        if (last_target != drag_drop_out_state_.target) {
          XSetSelectionOwner(x11_.display(), x11_.dndSelection(), window_handle_, event.xmotion.time);
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
      if (mouseRelativeMode() && mouse_down_position_ == Point(event.xmotion.x, event.xmotion.y))
        break;

      handleMouseMove(event.xmotion.x, event.xmotion.y, mouseButtonState(), modifierState());
      if (mouseRelativeMode())
        setCursorPosition(mouse_down_position_);
      break;
    }
    case ButtonPress: {
      if (event.xbutton.button >= 4 && event.xbutton.button <= 7) {
        float y = (event.xbutton.button == 4 ? 1.0f : 0.0f) - (event.xbutton.button == 5 ? 1.0f : 0.0f);
        float x = (event.xbutton.button == 7 ? 1.0f : 0.0f) - (event.xbutton.button == 6 ? 1.0f : 0.0f);
        handleMouseWheel(x, y, event.xbutton.x, event.xbutton.y, mouseButtonState(), modifierState());
      }
      else {
        MouseButton button = buttonFromEvent(event);
        if (button == kMouseButtonNone)
          passEventToParent(event);
        else
          handleMouseDown(button, event.xbutton.x, event.xbutton.y, mouseButtonState(), modifierState());
        mouse_down_position_ = { event.xbutton.x, event.xbutton.y };

        drag_drop_out_state_.dragging = isDragDropSource();
        if (drag_drop_out_state_.dragging) {
          setCursorStyle(MouseCursor::MultiDirectionalResize);
          drag_drop_files_.clear();
          drag_drop_files_.push_back(startDragDropSource());
        }
      }
      break;
    }
    case ButtonRelease: {
      MouseButton button = buttonFromEvent(event);
      if (drag_drop_out_state_.dragging && button == kMouseButtonLeft) {
        if (event.xbutton.x >= 0 && event.xbutton.x < clientWidth() && event.xbutton.y >= 0 &&
            event.xbutton.y < clientHeight()) {
          if (drag_drop_out_state_.target)
            sendDragDropLeave(window_handle_, drag_drop_out_state_.target);
          handleFileDrop(event.xbutton.x, event.xbutton.y, drag_drop_files_);
        }
        else if (drag_drop_out_state_.target)
          sendDragDropDrop(window_handle_, drag_drop_out_state_.target);
      }
      if (button == kMouseButtonNone)
        passEventToParent(event);
      else
        handleMouseUp(buttonFromEvent(event), event.xbutton.x, event.xbutton.y, mouseButtonState(),
                      modifierState());
      break;
    }
    case EnterNotify: {
      handleMouseEnter(event.xcrossing.x, event.xcrossing.y);
      break;
    }
    case LeaveNotify: {
      if (event.xbutton.x < 0 || event.xbutton.x >= clientWidth() || event.xbutton.y < 0 ||
          event.xbutton.y >= clientHeight())
        handleMouseLeave(mouseButtonState(), modifierState());
      break;
    }
    case KeyPress: {
      static constexpr int kMaxCharacters = 32;
      int modifier_state = modifierState();
      char buffer[kMaxCharacters] {};
      KeySym keysym;
      int length = XLookupString(&event.xkey, buffer, sizeof(buffer), &keysym, nullptr);
      if ((modifier_state & kModifierAlt) == 0 && length && buffer[0] < 127)
        handleTextInput(std::string(buffer, length));

      bool repeat = pressed_[keysym];
      pressed_[keysym] = true;
      KeyCode key_code = translateKeyCode(keysym);
      bool used = false;
      if (key_code != KeyCode::Unknown)
        used = handleKeyDown(key_code, modifier_state, repeat);

      if (!used)
        passEventToParent(event);
      break;
    }
    case KeyRelease: {
      KeySym keysym = XLookupKeysym(&event.xkey, 0);
      pressed_[keysym] = false;
      KeyCode key_code = translateKeyCode(keysym);
      bool used = false;
      if (key_code != KeyCode::Unknown)
        used = handleKeyUp(key_code, modifierState());

      if (!used)
        passEventToParent(event);
      break;
    }
    case ConfigureNotify: {
      visage::Point dimensions = retrieveWindowDimensions();
      handleResized(dimensions.x, dimensions.y);
      break;
    }
    }
  }

  void WindowX11::runEventLoop() {
    Display* display = x11_.display();
    Atom wm_delete_message = XInternAtom(display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(x11_.display(), window_handle_, &wm_delete_message, 1);

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    fd_set read_fds;
    unsigned int fd = ConnectionNumber(display);

    long long start_timer_microseconds = time::microseconds();
    long long last_timer_microseconds = start_timer_microseconds;

    XEvent event;
    bool running = true;
    while (running) {
      FD_ZERO(&read_fds);
      FD_SET(fd, &read_fds);

      timeout.tv_sec = 0;
      long long us_to_timer = timer_microseconds_ - (time::microseconds() - last_timer_microseconds);
      int result = 0;
      if (us_to_timer > 0) {
        timeout.tv_usec = us_to_timer;
        result = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
      }
      if (result == -1)
        running = false;
      else if (result == 0) {
        last_timer_microseconds = time::microseconds();
        long long us_time = last_timer_microseconds - start_timer_microseconds;
        drawCallback(us_time / 1000000.0);

        while (XPending(X11Connection::globalInstance().display())) {
          XNextEvent(X11Connection::globalInstance().display(), &event);
          processMessageWindowEvent(event);
        }
      }
      else if (FD_ISSET(fd, &read_fds)) {
        while (running && XPending(x11_.display())) {
          XNextEvent(x11_.display(), &event);
          if (event.type == DestroyNotify ||
              (event.type == ClientMessage && event.xclient.data.l[0] == wm_delete_message))
            running = false;
          else
            processEvent(event);
        }
      }
    }
  }

  void WindowX11::windowContentsResized(int width, int height) {
    setFixedAspectRatio(isFixedAspectRatio());
    XResizeWindow(x11_.display(), window_handle_, width, height);
  }

  void WindowX11::show() {
    X11Connection::DisplayLock lock(x11_);
    ::Display* display = x11_.display();
    XMapWindow(display, window_handle_);
    XFlush(display);
  }

  void WindowX11::hide() {
    X11Connection::DisplayLock lock(x11_);
    ::Display* display = x11_.display();
    XUnmapWindow(display, window_handle_);
    XFlush(display);
  }

  void WindowX11::setWindowTitle(const std::string& title) {
    X11Connection::DisplayLock lock(x11_);
    XStoreName(x11_.display(), window_handle_, title.c_str());
  }

  Point WindowX11::maxWindowDimensions() const {
    MonitorInfo monitor_info = activeMonitorInfo();

    int display_width = monitor_info.bounds.width();
    int display_height = monitor_info.bounds.height();
    float aspect_ratio = aspectRatio();
    return { std::min<int>(display_width, display_height * aspect_ratio),
             std::min<int>(display_height, display_width / aspect_ratio) };
  }

  Point WindowX11::minWindowDimensions() const {
    MonitorInfo monitor_info = activeMonitorInfo();
    float minimum_scale = minimumWindowScale();

    int min_display_width = minimum_scale * monitor_info.bounds.width();
    int min_display_height = minimum_scale * monitor_info.bounds.height();
    float aspect_ratio = aspectRatio();

    return { std::max<int>(min_display_width, min_display_height * aspect_ratio),
             std::max<int>(min_display_height, min_display_width / aspect_ratio) };
  }
}
#endif
