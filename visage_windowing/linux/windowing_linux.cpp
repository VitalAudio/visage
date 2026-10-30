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

#if VISAGE_WAYLAND
#include "windowing_wayland.h"

#include <link.h>
#include <strings.h>
#endif

namespace visage {
  static bool _plugin_windows = false;

#if VISAGE_WAYLAND
  static bool loadedInMainProgram() {
    struct Search {
      uintptr_t address = 0;
      bool first = true;
      bool main_program = false;
    };

    Search search;
    search.address = reinterpret_cast<uintptr_t>(&loadedInMainProgram);
    dl_iterate_phdr(
        [](dl_phdr_info* info, size_t, void* data) {
          auto search = static_cast<Search*>(data);
          bool first = search->first;
          search->first = false;
          for (int i = 0; i < info->dlpi_phnum; ++i) {
            const ElfW(Phdr)& header = info->dlpi_phdr[i];
            uintptr_t start = info->dlpi_addr + header.p_vaddr;
            if (header.p_type == PT_LOAD && search->address >= start &&
                search->address < start + header.p_memsz) {
              search->main_program = first;
              return 1;
            }
          }
          return 0;
        },
        &search);
    return search.main_program;
  }
#endif

  static bool useWayland() {
#if VISAGE_WAYLAND
    static const bool wayland = [] {
      const char* setting = std::getenv("VISAGE_WINDOWING");
      bool forced = setting && strcasecmp(setting, "wayland") == 0;
      if (setting && strcasecmp(setting, "x11") == 0)
        return false;

      const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
      if (!forced && (wayland_display == nullptr || wayland_display[0] == '\0' || !loadedInMainProgram()))
        return false;
      return waylandAvailable();
    }();
    return wayland && !_plugin_windows;
#else
    return false;
#endif
  }

#if VISAGE_WAYLAND
#define VISAGE_LINUX_WINDOWING(function, ...) \
  (useWayland() ? function##Wayland(__VA_ARGS__) : function##X11(__VA_ARGS__))
#else
#define VISAGE_LINUX_WINDOWING(function, ...) function##X11(__VA_ARGS__)
#endif

  std::string readClipboardText() {
    return VISAGE_LINUX_WINDOWING(readClipboardText);
  }

  void setClipboardText(const std::string& text) {
    VISAGE_LINUX_WINDOWING(setClipboardText, text);
  }

  void setCursorStyle(MouseCursor style) {
    VISAGE_LINUX_WINDOWING(setCursorStyle, style);
  }

  void setCursorVisible(bool visible) {
    VISAGE_LINUX_WINDOWING(setCursorVisible, visible);
  }

  Point cursorPosition() {
    return VISAGE_LINUX_WINDOWING(cursorPosition);
  }

  void setCursorPosition(Point window_position) {
    VISAGE_LINUX_WINDOWING(setCursorPosition, window_position);
  }

  void setCursorScreenPosition(Point window_position) {
    VISAGE_LINUX_WINDOWING(setCursorScreenPosition, window_position);
  }

  bool isMobileDevice() {
    return false;
  }

  void showMessageBox(std::string title, std::string message) {
    VISAGE_LINUX_WINDOWING(showMessageBox, std::move(title), std::move(message));
  }

  float defaultDpiScale() {
    return VISAGE_LINUX_WINDOWING(defaultDpiScale);
  }

  IBounds computeWindowBounds(const Dimension& x, const Dimension& y, const Dimension& width,
                              const Dimension& height) {
    return VISAGE_LINUX_WINDOWING(computeWindowBounds, x, y, width, height);
  }

  std::unique_ptr<Window> createWindow(const Dimension& x, const Dimension& y, const Dimension& width,
                                       const Dimension& height, Window::Decoration decoration) {
    return VISAGE_LINUX_WINDOWING(createWindow, x, y, width, height, decoration);
  }

  std::unique_ptr<Window> createPluginWindow(const Dimension& width, const Dimension& height,
                                             void* parent_handle) {
    _plugin_windows = true;
    return createPluginWindowX11(width, height, parent_handle);
  }

  void* headlessWindowHandle() {
    return nullptr;
  }

  void closeApplication() {
    VISAGE_LINUX_WINDOWING(closeApplication);
  }

  int displayFps() {
    return VISAGE_LINUX_WINDOWING(displayFps);
  }
}

#endif
