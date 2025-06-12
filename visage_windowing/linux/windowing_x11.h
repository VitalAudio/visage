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

#if VISAGE_LINUX
#include "visage_utils/string_utils.h"
#include "windowing.h"

#include <atomic>
#include <map>
#include <thread>
#include <X11/Xatom.h>
#include <X11/Xlib.h>

namespace visage {
  enum DecoratorOperation {
    kMoveWindow = 1 << 0,
    kResizeLeft = 1 << 1,
    kResizeTop = 1 << 2,
    kResizeRight = 1 << 3,
    kResizeBottom = 1 << 4,
  };

  class X11Connection {
  public:
    static constexpr int kDndVersion = 5;
    static constexpr int kNumDndActions = 2;
    static constexpr int kNumDndTypes = 1;

    static int xErrorHandler(Display* display, XErrorEvent* error_event) {
      VISAGE_LOG(String("X11 Error: ") + static_cast<int>(error_event->error_code));
      return 0;
    }

    static X11Connection* globalInstance() {
      static X11Connection connection;
      return &connection;
    }

    class DisplayLock {
    public:
      explicit DisplayLock(const X11Connection* x11) : x11_(x11) { XLockDisplay(x11_->display()); }

      ~DisplayLock() { XUnlockDisplay(x11_->display()); }

    private:
      const X11Connection* x11_;
    };

    struct Cursors {
      explicit Cursors(Display* display);

      Cursor no_cursor;
      Cursor arrow_cursor;
      Cursor ibeam_cursor;
      Cursor crosshair_cursor;
      Cursor pointing_cursor;
      Cursor horizontal_resize_cursor;
      Cursor vertical_resize_cursor;
      Cursor top_left_resize_cursor;
      Cursor top_right_resize_cursor;
      Cursor bottom_left_resize_cursor;
      Cursor bottom_right_resize_cursor;
      Cursor multi_directional_resize_cursor;
    };

    X11Connection() {
      display_ = XOpenDisplay(nullptr);
      XSetErrorHandler(xErrorHandler);
      if (display_ == nullptr)
        return;

      fd_ = ConnectionNumber(display_);
      root_ = DefaultRootWindow(display_);
      XSetWindowAttributes attributes;
      attributes.event_mask = NoEventMask;
      XSync(display_, false);
      clipboard_ = XInternAtom(display_, "CLIPBOARD", False);
      utf8_string_ = XInternAtom(display_, "UTF8_STRING", False);
      targets_ = XInternAtom(display_, "TARGETS", False);
      timer_event_ = XInternAtom(display_, "VISAGE_TIMER_EVENT", False);
      delete_message_ = XInternAtom(display_, "WM_DELETE_WINDOW", False);
      dnd_aware_ = XInternAtom(display_, "XdndAware", False);
      dnd_proxy_ = XInternAtom(display_, "XdndProxy", False);
      dnd_enter_ = XInternAtom(display_, "XdndEnter", False);
      dnd_leave_ = XInternAtom(display_, "XdndLeave", False);
      dnd_drop_ = XInternAtom(display_, "XdndDrop", False);
      dnd_selection_ = XInternAtom(display_, "XdndSelection", False);
      dnd_uri_list_ = XInternAtom(display_, "text/uri-list", False);
      dnd_position_ = XInternAtom(display_, "XdndPosition", False);
      dnd_finished_ = XInternAtom(display_, "XdndFinished", False);
      dnd_status_ = XInternAtom(display_, "XdndStatus", False);
      dnd_action_none_ = XInternAtom(display_, "XdndActionNone", False);
      dnd_action_copy_ = XInternAtom(display_, "XdndActionCopy", False);
      dnd_action_list_ = XInternAtom(display_, "XdndActionList", False);
      dnd_action_description_ = XInternAtom(display_, "XdndActionDescription", False);
      dnd_type_list_ = XInternAtom(display_, "XdndTypeList", False);
      dnd_types_[0] = dnd_uri_list_;
      dnd_actions_[0] = dnd_action_copy_;
      dnd_actions_[1] = dnd_action_none_;
      cursors_ = std::make_unique<Cursors>(display_);
    }

    X11Connection(const X11Connection& copy) = delete;

    ~X11Connection() { XCloseDisplay(display_); }

    ::Display* display() const { return display_; }
    ::Window rootWindow() const { return root_; }
    Atom clipboard() const { return clipboard_; }
    Atom utf8String() const { return utf8_string_; }
    Atom targets() const { return targets_; }
    Atom timerEvent() const { return timer_event_; }
    Atom* deleteMessageRef() { return &delete_message_; }
    Atom deleteMessage() const { return delete_message_; }
    Atom dndAware() const { return dnd_aware_; }
    Atom dndProxy() const { return dnd_proxy_; }
    Atom dndEnter() const { return dnd_enter_; }
    Atom dndLeave() const { return dnd_leave_; }
    Atom dndDrop() const { return dnd_drop_; }
    Atom dndPosition() const { return dnd_position_; }
    Atom dndSelection() const { return dnd_selection_; }
    Atom dndUriList() const { return dnd_uri_list_; }
    Atom dndFinished() const { return dnd_finished_; }
    Atom dndStatus() const { return dnd_status_; }
    Atom dndActionNone() const { return dnd_action_none_; }
    Atom dndActionCopy() const { return dnd_action_copy_; }
    Atom dndActionList() const { return dnd_action_list_; }
    Atom dndActionDescription() const { return dnd_action_description_; }
    const unsigned char* dndActions() const { return (unsigned char*)dnd_actions_; }
    Atom dndTypeList() const { return dnd_type_list_; }
    const unsigned char* dndTypes() const { return (unsigned char*)dnd_types_; }
    Atom dndType(int index) const { return dnd_types_[index]; }
    const unsigned char* dndVersion() { return (unsigned char*)&dnd_version_; }

    const Cursors& cursors() const { return *cursors_; }
    int fd() const { return fd_; }

  private:
    ::Display* display_ = nullptr;
    int fd_ = 0;
    ::Window root_ = 0;
    Atom clipboard_ = 0;
    Atom utf8_string_ = 0;
    Atom targets_ = 0;
    Atom timer_event_ = 0;
    Atom delete_message_ = 0;
    Atom dnd_aware_ = 0;
    Atom dnd_proxy_ = 0;
    Atom dnd_enter_ = 0;
    Atom dnd_leave_ = 0;
    Atom dnd_drop_ = 0;
    Atom dnd_selection_ = 0;
    Atom dnd_uri_list_ = 0;
    Atom dnd_position_ = 0;
    Atom dnd_finished_ = 0;
    Atom dnd_status_ = 0;
    Atom dnd_action_none_ = 0;
    Atom dnd_action_copy_ = 0;
    Atom dnd_action_list_ = 0;
    Atom dnd_action_description_ = 0;
    Atom dnd_version_ = kDndVersion;
    Atom dnd_actions_[kNumDndActions] {};
    Atom dnd_type_list_ = 0;
    Atom dnd_types_[kNumDndTypes] {};
    std::unique_ptr<Cursors> cursors_;
  };

  struct MonitorInfo {
    static constexpr int kDefaultRefreshRate = 60;

    IBounds bounds;
    double refresh_rate = kDefaultRefreshRate;
    float dpi = Window::kDefaultDpi;
  };

  class WindowX11 : public Window {
  public:
    static constexpr long kEventMask = ExposureMask | KeyPressMask | KeyReleaseMask |
                                       ButtonPressMask | ButtonReleaseMask | StructureNotifyMask |
                                       EnterWindowMask | LeaveWindowMask | PointerMotionMask |
                                       Button1MotionMask | Button2MotionMask | Button3MotionMask |
                                       VisibilityChangeMask | FocusChangeMask;

    static constexpr int kMinWidth = 80;
    static constexpr int kMinHeight = 80;
    static constexpr int kClientResizeBorder = 8;

    static WindowX11* lastActiveWindow() { return last_active_window_; }

    WindowX11(int x, int y, int width, int height, Decoration decoration);
    WindowX11(int width, int height, void* parent_handle);

    ~WindowX11() override;

    void runEventLoop() override;
    void processPluginFdEvents() override;
    void processMessageWindowEvent(XEvent& event);
    void processEvent(XEvent& event);

    void* nativeHandle() const override { return (void*)window_handle_; }

    int resizeOperationForPosition(int x, int y) const;
    void removeWindowDecorationButtons();
    void* initWindow() const override;
    void* globalDisplay() const override { return X11Connection::globalInstance()->display(); }
    int posixFd() const override { return x11_->fd(); }

    void setFixedAspectRatio(bool fixed) override;

    void windowContentsResized(int width, int height) override;
    void show() override;
    void showMaximized() override;
    void hide() override;
    void close() override;
    bool isShowing() const override;

    void setWindowTitle(const std::string& title) override;
    IPoint maxWindowDimensions() const override;
    IPoint minWindowDimensions() const override;
    MonitorInfo monitorInfo() { return monitor_info_; }
    X11Connection* x11Connection() { return x11_; }
    bool timerThreadRunning() { return timer_thread_running_.load(); }
    int timerMs() const { return timer_microseconds_.load() / 1000; }

  private:
    static WindowX11* last_active_window_;

    struct DragDropOutState {
      bool dragging = false;
      ::Window target = 0;
    };

    ::Window parentHandle() const { return parent_handle_; }

    void createWindow(IBounds bounds);
    IPoint retrieveWindowDimensions();
    void passEventToParent(XEvent& event);
    int mouseButtonState() const;
    int modifierState() const;

    ::Window windowUnderCursor(::Window inside);
    ::Window windowUnderCursor();
    ::Window dragDropProxy(::Window window) const;

    void sendDragDropEnter(::Window source, ::Window target) const;
    void sendDragDropLeave(::Window source, ::Window target) const;
    void sendDragDropPosition(::Window source, ::Window target, int x, int y, unsigned long time) const;
    void sendDragDropStatus(::Window source, ::Window target, bool accept_drag) const;
    void sendDragDropDrop(::Window source, ::Window target) const;
    void sendDragDropSelectionNotify(XSelectionRequestEvent* request) const;
    void sendDragDropFinished(::Window source, ::Window target, bool accepted_drag) const;

    X11Connection* x11_ = nullptr;
    std::unique_ptr<X11Connection> plugin_x11_;
    DragDropOutState drag_drop_out_state_;
    std::vector<std::string> drag_drop_files_;
    int drag_drop_target_x_ = 0;
    int drag_drop_target_y_ = 0;
    int hover_window_operation_ = 0;
    int window_operation_ = 0;
    IPoint dragging_window_position_;

    long long start_draw_microseconds_ = 0;
    IPoint mouse_down_position_;
    Decoration decoration_ = Decoration::Native;
    MonitorInfo monitor_info_;
    ::Window window_handle_ = 0;
    ::Window parent_handle_ = 0;
    std::map<KeySym, bool> pressed_;
    long long start_microseconds_ = 0;
    std::atomic<long long> timer_microseconds_ = 16667;
    std::atomic<bool> timer_thread_running_ = false;
    std::unique_ptr<std::thread> timer_thread_;
  };
}

#endif
