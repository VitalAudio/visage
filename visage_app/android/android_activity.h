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

#if VISAGE_ANDROID || defined(__ANDROID__)

#include <android/configuration.h>
#include <android/input.h>
#include <android/log.h>
#include <android/native_activity.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <android_native_app_glue.h>
#include <atomic>
#include <chrono>
#include <jni.h>
#include <memory>
#include <mutex>
#include <string>
#include <visage_graphics/canvas.h>
#include <visage_graphics/renderer.h>
#include <visage_ui/frame.h>
#include <visage_windowing/android/windowing_android.h>

namespace visage {

  struct AndroidInsets {
    float statusBarHeightDp = 0.0f;
    float navigationBarHeightDp = 0.0f;
    int statusBarHeightPx = 0;
    int navigationBarHeightPx = 0;
  };

  class Activity {
  public:
    Activity();
    virtual ~Activity();

    Activity(const Activity&) = delete;
    Activity& operator=(const Activity&) = delete;

    // Lifecycle callbacks to override in client applications
    virtual void onInit() { }
    virtual void onDestroy() { }
    virtual void onStart() { }
    virtual void onStop() { }
    virtual void onPause() { }
    virtual void onResume() { }
    virtual void onWindowCreated() { }
    virtual void onWindowDestroyed() { }
    virtual void onResize(float width_dp, float height_dp) { }
    virtual void onDraw(visage::Canvas& canvas) { }
    virtual bool onTouchEvent(int action, float x_dp, float y_dp) { return false; }

    // Control APIs
    void requestRedraw(int frames = 2);
    void setVsync(bool vsync);
    [[nodiscard]] bool vsync() const { return vsync_; }
    [[nodiscard]] FrameEventHandler* frameEventHandler() { return &frame_event_handler_; }

    // Geometry & System APIs
    [[nodiscard]] float dpiScale() const { return dpi_scale_; }
    [[nodiscard]] int windowWidth() const { return width_; }
    [[nodiscard]] int windowHeight() const { return height_; }
    [[nodiscard]] float logicalWidth() const {
      return dpi_scale_ > 0.0f ? static_cast<float>(width_) / dpi_scale_ : static_cast<float>(width_);
    }
    [[nodiscard]] float logicalHeight() const {
      return dpi_scale_ > 0.0f ? static_cast<float>(height_) / dpi_scale_ : static_cast<float>(height_);
    }
    [[nodiscard]] const AndroidInsets& insets() const { return insets_; }
    [[nodiscard]] struct android_app* androidApp() const { return app_; }
    [[nodiscard]] ANativeWindow* nativeWindow() const { return window_; }
    [[nodiscard]] visage::Canvas* canvas() const { return canvas_.get(); }
    [[nodiscard]] bool hasWindow() const;
    [[nodiscard]] bool needsRedraw() const;

    // Run entry point called from android_main
    static void run(struct android_app* state, std::unique_ptr<Activity> activity);

    template<typename T, typename... Args>
    static void run(struct android_app* state, Args&&... args) {
      run(state, std::make_unique<T>(std::forward<Args>(args)...));
    }

  private:
    void setApp(struct android_app* app);
    void setDpiScale(float scale);
    void setWindow(ANativeWindow* window);
    void destroyWindow();
    void resize(int width, int height);
    void renderFrame();
    void resolveInsets();
    void handleTouch(int action, float x_px, float y_px);

    struct android_app* app_ = nullptr;
    ANativeWindow* window_ = nullptr;
    std::unique_ptr<visage::WindowAndroid> window_android_;
    std::unique_ptr<visage::Canvas> canvas_;
    int width_ = 1080;
    int height_ = 2400;
    float dpi_scale_ = 2.75f;
    AndroidInsets insets_ {};
    std::chrono::steady_clock::time_point start_time_;
    uint32_t frame_count_ = 0;
    std::atomic<int> redraw_frames_ { 4 };
    std::atomic<bool> is_paused_ { false };
    bool vsync_ = true;
    FrameEventHandler frame_event_handler_;
    mutable std::mutex render_mutex_;

    friend void OnVisageAppCmd(struct android_app* app, int32_t cmd);
    friend int32_t OnVisageInputEvent(struct android_app* app, AInputEvent* event);
  };

  using AndroidActivity = Activity;

}  // namespace visage

#endif  // VISAGE_ANDROID || defined(__ANDROID__)
