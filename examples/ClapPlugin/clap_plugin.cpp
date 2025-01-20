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

#include "clap_plugin.h"

#include <clap/helpers/host-proxy.hh>
#include <clap/helpers/host-proxy.hxx>
#include <clap/helpers/plugin.hh>
#include <clap/helpers/plugin.hxx>
#include <visage_windowing/windowing.h>

using namespace visage::dimension;

static const char* kClapFeatures[] = { CLAP_PLUGIN_FEATURE_INSTRUMENT, nullptr };

clap_plugin_descriptor ClapPlugin::descriptor = { CLAP_VERSION,          "dev.visage.example",
                                                  "Example Clap Plugin", "Visage",
                                                  "visage.dev",          "visage.dev",
                                                  "visage.dev",          "0.0.1",
                                                  "Example Clap Plugin", kClapFeatures };

ClapPlugin::ClapPlugin(const clap_host* host) : ClapPluginBase(&descriptor, host) { }

ClapPlugin::~ClapPlugin() = default;

#ifdef __linux__
void ClapPlugin::onPosixFd(int fd, clap_posix_fd_flags_t flags) noexcept {
  if (window_)
    window_->processPluginFdEvents();
}
#endif

bool ClapPlugin::guiIsApiSupported(const char* api, bool is_floating) noexcept {
  if (is_floating)
    return false;

#ifdef _WIN32
  if (strcmp(api, CLAP_WINDOW_API_WIN32) == 0)
    return true;
#elif __APPLE__
  if (strcmp(api, CLAP_WINDOW_API_COCOA) == 0)
    return true;
#elif __linux__
  if (strcmp(api, CLAP_WINDOW_API_X11) == 0)
    return true;
#endif

  return false;
}

bool ClapPlugin::guiCreate(const char* api, bool is_floating) noexcept {
  if (is_floating)
    return false;

  if (editor_.get())
    return true;

  editor_ = std::make_unique<visage::ApplicationEditor>();
  
  editor_->onDraw() = [this](visage::Canvas& canvas) {
    canvas.setColor(0xff000066);
    canvas.fill(0, 0, editor_->width(), editor_->height());

    float circle_radius = editor_->height() * 0.1f;
    float x = editor_->width() * 0.5f - circle_radius;
    float y = editor_->height() * 0.5f - circle_radius;
    canvas.setColor(0xff00ffff);
    canvas.circle(x, y, 2.0f * circle_radius);
  };

  visage::Bounds bounds = visage::computeWindowBounds(80_vmin, 60_vmin);
  editor_->setBounds(0, 0, bounds.width(), bounds.height());

  editor_->onResize() += [this]() {
    _host.guiResizeHintsChanged();
    _host.guiRequestResize(editor_->logicalWidth(), editor_->logicalHeight());
  };

  return true;
}

void ClapPlugin::guiDestroy() noexcept {
#if VA_LINUX
  if (window_)
    _host.posixFdSupportUnregister(window_->getPosixFd());
#endif
  editor_->removeFromWindow();
  window_.reset();
}

bool ClapPlugin::guiSetParent(const clap_window* window) noexcept {
  if (editor_ == nullptr)
    return false;

  window_ = visage::createPluginWindow(editor_->width(), editor_->height(), window->ptr);
  window_->setFixedAspectRatio(editor_->isFixedAspectRatio());
  editor_->addToWindow(window_.get());
  window_->show();

#if VA_LINUX
  return _host.posixFdSupportRegister(window_->getPosixFd(),
                                      CLAP_POSIX_FD_READ | CLAP_POSIX_FD_WRITE | CLAP_POSIX_FD_ERROR);
#else
  return true;
#endif
}

bool ClapPlugin::guiGetResizeHints(clap_gui_resize_hints_t* hints) noexcept {
  if (editor_ == nullptr)
    return false;

  bool fixed_aspect_ratio = editor_->isFixedAspectRatio();
  hints->can_resize_horizontally = !fixed_aspect_ratio;
  hints->can_resize_vertically = !fixed_aspect_ratio;
  hints->preserve_aspect_ratio = fixed_aspect_ratio;

  if (fixed_aspect_ratio) {
    hints->aspect_ratio_width = editor_->height() * editor_->aspectRatio();
    hints->aspect_ratio_height = editor_->width();
  }
  return true;
}

bool ClapPlugin::guiAdjustSize(uint32_t* width, uint32_t* height) noexcept {
  if (editor_ == nullptr)
    return false;

  if (!editor_->isFixedAspectRatio())
    return true;

  visage::Point max_dimensions = { INT_MAX, INT_MAX };
  visage::Point min_dimensions = { 0, 0 };
  if (window_) {
    max_dimensions = window_->maxWindowDimensions();
    min_dimensions = window_->minWindowDimensions();
  }

  visage::Point point(static_cast<int>(*width), static_cast<int>(*height));
  visage::Point adjusted_dimensions = adjustBoundsForAspectRatio(point, min_dimensions, max_dimensions,
                                                                 editor_->aspectRatio(), true, true);
  *width = adjusted_dimensions.x;
  *height = adjusted_dimensions.y;

  return true;
}

bool ClapPlugin::guiSetSize(uint32_t width, uint32_t height) noexcept {
  if (editor_ == nullptr)
    return false;

  if (window_)
    window_->setWindowSize(width, height);
  else
    editor_->setLogicalDimensions(width, height);

  return true;
}

bool ClapPlugin::guiGetSize(uint32_t* width, uint32_t* height) noexcept {
  if (editor_ == nullptr)
    return false;

  *width = std::round(editor_->logicalWidth());
  *height = std::round(editor_->logicalHeight());
  return true;
}
