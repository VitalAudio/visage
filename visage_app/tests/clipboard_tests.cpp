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

#include "visage/app.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdlib>

#if VISAGE_LINUX
#include <atomic>
#include <thread>
#include <visage/ui.h>
#include <X11/Xatom.h>
#include <X11/Xlib.h>

namespace {
  class FakeSelectionOwner {
  public:
    FakeSelectionOwner(const char* selection, const char* target, std::string data, size_t chunk_size) :
        data_(std::move(data)), chunk_size_(chunk_size) {
      display_ = XOpenDisplay(nullptr);
      window_ = XCreateSimpleWindow(display_, DefaultRootWindow(display_), 0, 0, 1, 1, 0, 0, 0);
      selection_ = XInternAtom(display_, selection, False);
      target_ = XInternAtom(display_, target, False);
      XSetSelectionOwner(display_, selection_, window_, CurrentTime);
      XSync(display_, False);
    }

    ~FakeSelectionOwner() {
      XDestroyWindow(display_, window_);
      XCloseDisplay(display_);
    }

    void pump() {
      while (XPending(display_)) {
        XEvent event;
        XNextEvent(display_, &event);
        if (event.type == SelectionRequest)
          reply(event.xselectionrequest);
        else if (event.type == PropertyNotify && event.xproperty.window == requestor_ &&
                 event.xproperty.atom == property_ && event.xproperty.state == PropertyDelete)
          sendChunk();
      }
    }

    Display* display() const { return display_; }
    ::Window window() const { return window_; }
    bool done() const { return done_; }
    int chunks() const { return chunks_; }

  private:
    void reply(const XSelectionRequestEvent& request) {
      XSelectionEvent notify = {};
      notify.type = SelectionNotify;
      notify.requestor = request.requestor;
      notify.selection = request.selection;
      notify.target = request.target;
      notify.time = request.time;
      notify.property = None;

      if (request.target == target_) {
        if (data_.size() > chunk_size_) {
          requestor_ = request.requestor;
          property_ = request.property;
          XSelectInput(display_, requestor_, PropertyChangeMask);
          long size = data_.size();
          XChangeProperty(display_, requestor_, property_, XInternAtom(display_, "INCR", False), 32,
                          PropModeReplace, reinterpret_cast<unsigned char*>(&size), 1);
        }
        else {
          XChangeProperty(display_, request.requestor, request.property, target_, 8, PropModeReplace,
                          reinterpret_cast<const unsigned char*>(data_.data()), data_.size());
          done_ = true;
        }
        notify.property = request.property;
      }

      XSendEvent(display_, request.requestor, False, NoEventMask, reinterpret_cast<XEvent*>(&notify));
      XFlush(display_);
    }

    void sendChunk() {
      if (done_)
        return;

      size_t length = std::min(chunk_size_, data_.size() - sent_);
      XChangeProperty(display_, requestor_, property_, target_, 8, PropModeReplace,
                      reinterpret_cast<const unsigned char*>(data_.data() + sent_), length);
      XFlush(display_);
      sent_ += length;
      chunks_++;
      done_ = length == 0;
    }

    Display* display_ = nullptr;
    ::Window window_ = 0;
    Atom selection_ = None;
    Atom target_ = None;
    std::string data_;
    size_t chunk_size_ = 0;
    ::Window requestor_ = 0;
    Atom property_ = None;
    size_t sent_ = 0;
    int chunks_ = 0;
    bool done_ = false;
  };

  std::string readClipboardFrom(FakeSelectionOwner& owner) {
    std::atomic<bool> stop = false;
    std::thread thread([&] {
      while (!stop) {
        owner.pump();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    std::string text = visage::readClipboardText();
    stop = true;
    thread.join();
    return text;
  }

  bool hasDisplay() {
    const char* display_name = std::getenv("DISPLAY");
    return display_name && display_name[0];
  }

  struct SelectionReply {
    bool replied = false;
    Atom type = None;
    std::string data;
  };

  SelectionReply requestSelection(Display* display, ::Window requestor,
                                  visage::Window* plugin_window, Atom target) {
    Atom clipboard = XInternAtom(display, "CLIPBOARD", False);
    Atom property = XInternAtom(display, "VISAGE_TEST_SELECTION", False);
    XConvertSelection(display, clipboard, target, property, requestor, CurrentTime);
    XFlush(display);

    SelectionReply reply;
    XEvent event;
    for (int i = 0; i < 400 && !reply.replied; ++i) {
      plugin_window->processPluginFdEvents();
      reply.replied = XCheckTypedWindowEvent(display, requestor, SelectionNotify, &event);
      if (!reply.replied)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (!reply.replied || event.xselection.property == None)
      return reply;

    int format = 0;
    unsigned long num_items = 0, bytes_after = 0;
    unsigned char* data = nullptr;
    XGetWindowProperty(display, requestor, property, 0, ~0, True, AnyPropertyType, &reply.type,
                       &format, &num_items, &bytes_after, &data);
    if (data) {
      int item_size = format == 32 ? sizeof(long) : format / 8;
      reply.data = std::string(reinterpret_cast<char*>(data), num_items * item_size);
      XFree(data);
    }
    return reply;
  }
}

TEST_CASE("X11 plugin windows serve clipboard requests", "[integration]") {
  const char* display_name = std::getenv("DISPLAY");
  if (display_name == nullptr || display_name[0] == 0) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window root = DefaultRootWindow(host);
  ::Window parent = XCreateSimpleWindow(host, root, 0, 0, 200, 200, 0, 0, 0);
  ::Window requestor = XCreateSimpleWindow(host, root, 0, 0, 1, 1, 0, 0, 0);
  XMapWindow(host, parent);
  XFlush(host);

  std::unique_ptr<visage::Window> plugin_window =
      visage::createPluginWindow(visage::Dimension::logicalPixels(100),
                                 visage::Dimension::logicalPixels(100), (void*)parent);
  REQUIRE(plugin_window);

  visage::setClipboardText("h\xc3\xa9llo \xe2\x82\xac");
  REQUIRE(visage::readClipboardText() == "h\xc3\xa9llo \xe2\x82\xac");

  Atom utf8_string = XInternAtom(host, "UTF8_STRING", False);
  SelectionReply utf8 = requestSelection(host, requestor, plugin_window.get(), utf8_string);
  REQUIRE(utf8.replied);
  REQUIRE(utf8.type == utf8_string);
  REQUIRE(utf8.data == "h\xc3\xa9llo \xe2\x82\xac");

  SelectionReply latin1 = requestSelection(host, requestor, plugin_window.get(), XA_STRING);
  REQUIRE(latin1.replied);
  REQUIRE(latin1.type == XA_STRING);
  REQUIRE(latin1.data == "h\xe9llo ?");

  Atom targets = XInternAtom(host, "TARGETS", False);
  SelectionReply supported = requestSelection(host, requestor, plugin_window.get(), targets);
  REQUIRE(supported.replied);
  REQUIRE(supported.type == XA_ATOM);
  REQUIRE(supported.data.size() == 3 * sizeof(long));

  plugin_window.reset();
  REQUIRE(XGetSelectionOwner(host, XInternAtom(host, "CLIPBOARD", False)) == None);

  XDestroyWindow(host, requestor);
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("X11 clipboard moves to another plugin window when its owner closes", "[integration]") {
  const char* display_name = std::getenv("DISPLAY");
  if (display_name == nullptr || display_name[0] == 0) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window root = DefaultRootWindow(host);
  ::Window parent = XCreateSimpleWindow(host, root, 0, 0, 200, 200, 0, 0, 0);
  ::Window requestor = XCreateSimpleWindow(host, root, 0, 0, 1, 1, 0, 0, 0);
  XMapWindow(host, parent);
  XFlush(host);

  std::unique_ptr<visage::Window> windows[] = {
    visage::createPluginWindow(visage::Dimension::logicalPixels(100),
                               visage::Dimension::logicalPixels(100), (void*)parent),
    visage::createPluginWindow(visage::Dimension::logicalPixels(100),
                               visage::Dimension::logicalPixels(100), (void*)parent)
  };
  REQUIRE(windows[0]);
  REQUIRE(windows[1]);

  visage::setClipboardText("copied");
  Atom clipboard = XInternAtom(host, "CLIPBOARD", False);
  ::Window owner = XGetSelectionOwner(host, clipboard);
  int owner_index = owner == (::Window)windows[0]->nativeHandle() ? 0 : 1;
  REQUIRE(owner == (::Window)windows[owner_index]->nativeHandle());

  XSetSelectionOwner(host, XA_PRIMARY, requestor, CurrentTime);
  XSync(host, False);

  windows[owner_index].reset();
  REQUIRE(XGetSelectionOwner(host, XA_PRIMARY) == requestor);
  visage::Window* remaining = windows[1 - owner_index].get();
  REQUIRE(XGetSelectionOwner(host, clipboard) == (::Window)remaining->nativeHandle());
  REQUIRE(visage::readClipboardText() == "copied");

  SelectionReply reply = requestSelection(host, requestor, remaining,
                                          XInternAtom(host, "UTF8_STRING", False));
  REQUIRE(reply.replied);
  REQUIRE(reply.data == "copied");

  windows[1 - owner_index].reset();
  REQUIRE(XGetSelectionOwner(host, clipboard) == None);

  XDestroyWindow(host, requestor);
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("X11 clipboard reads text sent in INCR chunks", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  std::string text;
  for (int i = 0; text.size() < 300000; ++i)
    text += "line " + std::to_string(i) + " caf\xc3\xa9\n";

  FakeSelectionOwner owner("CLIPBOARD", "UTF8_STRING", text, 65536);
  std::string result = readClipboardFrom(owner);
  REQUIRE(owner.done());
  REQUIRE(owner.chunks() > 1);
  REQUIRE(result.size() == text.size());
  REQUIRE((result == text));
}

TEST_CASE("X11 clipboard falls back to Latin-1 STRING", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  FakeSelectionOwner owner("CLIPBOARD", "STRING", "caf\xe9", 65536);
  REQUIRE(readClipboardFrom(owner) == "caf\xc3\xa9");
}

TEST_CASE("X11 drops read file lists sent in INCR chunks", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  class DropFrame : public visage::Frame {
  public:
    bool receivesDragDropFiles() override { return true; }
    bool receivesMultipleDragDropFiles() override { return true; }
    void dropFiles(const std::vector<std::string>& paths) override { dropped = paths; }

    std::vector<std::string> dropped;
  };

  static constexpr int kNumFiles = 6000;
  std::string uri_list;
  for (int i = 0; i < kNumFiles; ++i)
    uri_list += "file:///tmp/visage%20incr/file%20" + std::to_string(i) + ".wav\r\n";

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 100, 100, 0, 0, 0);
  XMapWindow(host, parent);
  XSync(host, False);

  std::unique_ptr<visage::Window> window = visage::createPluginWindow(visage::Dimension::logicalPixels(100),
                                                                      visage::Dimension::logicalPixels(100),
                                                                      (void*)parent);
  REQUIRE(window);

  visage::ApplicationEditor editor;
  DropFrame drop_frame;
  editor.addChild(drop_frame);
  editor.setBounds(0, 0, 100, 100);
  drop_frame.setBounds(0, 0, 100, 100);
  editor.addToWindow(window.get());

  FakeSelectionOwner source("XdndSelection", "text/uri-list", uri_list, 65536);
  auto send_dnd = [&](const char* type, long data1, long data2, long data4) {
    XEvent event = {};
    event.xclient.type = ClientMessage;
    event.xclient.window = (::Window)window->nativeHandle();
    event.xclient.message_type = XInternAtom(host, type, False);
    event.xclient.format = 32;
    event.xclient.data.l[0] = source.window();
    event.xclient.data.l[1] = data1;
    event.xclient.data.l[2] = data2;
    event.xclient.data.l[3] = CurrentTime;
    event.xclient.data.l[4] = data4;
    XSendEvent(host, (::Window)window->nativeHandle(), False, NoEventMask, &event);
    XFlush(host);
  };
  auto pump = [&](int iterations) {
    for (int i = 0; i < iterations; ++i) {
      window->processPluginFdEvents();
      source.pump();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };

  send_dnd("XdndEnter", 5 << 24, XInternAtom(host, "text/uri-list", False), 0);
  for (int i = 0; i < 2000 && !source.done(); ++i)
    pump(1);
  pump(20);
  REQUIRE(source.done());
  REQUIRE(source.chunks() > 1);

  send_dnd("XdndPosition", 0, (50 << 16) | 50, XInternAtom(host, "XdndActionCopy", False));
  pump(20);
  send_dnd("XdndDrop", 0, CurrentTime, 0);
  for (int i = 0; i < 1000 && drop_frame.dropped.empty(); ++i)
    pump(1);

  REQUIRE(drop_frame.dropped.size() == kNumFiles);
  REQUIRE(drop_frame.dropped.front() == "/tmp/visage incr/file 0.wav");
  REQUIRE(drop_frame.dropped.back() == "/tmp/visage incr/file " + std::to_string(kNumFiles - 1) + ".wav");

  editor.removeFromWindow();
  window.reset();
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}
#endif
