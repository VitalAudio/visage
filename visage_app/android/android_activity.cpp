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

#include "android_activity.h"

#if VISAGE_ANDROID || defined(__ANDROID__)

namespace visage {

  namespace {

    constexpr float kDefaultStatusBarDp = 32.0f;

  }  // namespace

  void OnVisageAppCmd(struct android_app* app, int32_t cmd) {
    auto* activity = static_cast<Activity*>(app->userData);
    if (activity == nullptr) {
      return;
    }

    switch (cmd) {
    case APP_CMD_INIT_WINDOW:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_INIT_WINDOW");
      activity->setWindow(app->window);
      activity->requestRedraw(4);
      activity->renderFrame();
      break;
    case APP_CMD_TERM_WINDOW:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_TERM_WINDOW");
      activity->destroyWindow();
      break;
    case APP_CMD_WINDOW_REDRAW_NEEDED:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_WINDOW_REDRAW_NEEDED");
      activity->requestRedraw(4);
      activity->renderFrame();
      break;
    case APP_CMD_GAINED_FOCUS:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_GAINED_FOCUS");
      activity->requestRedraw(2);
      break;
    case APP_CMD_LOST_FOCUS: VISAGE_LOG("OnVisageAppCmd: APP_CMD_LOST_FOCUS"); break;
    case APP_CMD_WINDOW_RESIZED:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_WINDOW_RESIZED");
      if (app->window != nullptr) {
        activity->resize(ANativeWindow_getWidth(app->window), ANativeWindow_getHeight(app->window));
      }
      break;
    case APP_CMD_CONFIG_CHANGED: VISAGE_LOG("OnVisageAppCmd: APP_CMD_CONFIG_CHANGED"); break;
    case APP_CMD_LOW_MEMORY: VISAGE_LOG("OnVisageAppCmd: APP_CMD_LOW_MEMORY"); break;
    case APP_CMD_START:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_START");
      activity->onStart();
      break;
    case APP_CMD_RESUME:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_RESUME");
      activity->is_paused_.store(false, std::memory_order_release);
      activity->onResume();
      activity->requestRedraw(4);
      break;
    case APP_CMD_PAUSE:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_PAUSE");
      activity->is_paused_.store(true, std::memory_order_release);
      activity->onPause();
      break;
    case APP_CMD_STOP:
      VISAGE_LOG("OnVisageAppCmd: APP_CMD_STOP");
      activity->is_paused_.store(true, std::memory_order_release);
      activity->onStop();
      activity->destroyWindow();
      break;
    case APP_CMD_DESTROY: VISAGE_LOG("OnVisageAppCmd: APP_CMD_DESTROY"); break;
    default: VISAGE_LOG("OnVisageAppCmd: unhandled cmd=%d", cmd); break;
    }
  }

  int32_t OnVisageInputEvent(struct android_app* app, AInputEvent* event) {
    auto* activity = static_cast<Activity*>(app->userData);
    if (activity == nullptr || event == nullptr) {
      return 0;
    }

    if (AInputEvent_getType(event) == AINPUT_EVENT_TYPE_MOTION) {
      const int32_t action = AMotionEvent_getAction(event);
      const float x = AMotionEvent_getX(event, 0);
      const float y = AMotionEvent_getY(event, 0);
      activity->handleTouch(action, x, y);
      return 1;
    }
    return 0;
  }

  Activity::Activity() : start_time_(std::chrono::steady_clock::now()) {
    frame_event_handler_.request_redraw = [this](Frame* /*frame*/) { requestRedraw(2); };
  }

  Activity::~Activity() {
    destroyWindow();
  }

  void Activity::requestRedraw(int frames) {
    int current = redraw_frames_.load(std::memory_order_relaxed);
    while (current < frames && !redraw_frames_.compare_exchange_weak(current, frames, std::memory_order_release,
                                                                     std::memory_order_relaxed)) {
    }
    if (app_ != nullptr && app_->looper != nullptr) {
      ALooper_wake(app_->looper);
    }
  }

  void Activity::setVsync(bool vsync) {
    vsync_ = vsync;
    if (window_ != nullptr) {
      visage::Renderer::setNativeWindow(window_, width_, height_, vsync_);
    }
  }

  bool Activity::hasWindow() const {
    return window_ != nullptr && canvas_ != nullptr && !is_paused_.load(std::memory_order_acquire);
  }

  bool Activity::needsRedraw() const {
    return !is_paused_.load(std::memory_order_acquire) &&
           (redraw_frames_.load(std::memory_order_acquire) > 0);
  }

  void Activity::setApp(struct android_app* app) {
    app_ = app;
  }

  void Activity::setDpiScale(float scale) {
    if (scale > 0.0f) {
      dpi_scale_ = scale;
    }
  }

  void Activity::setWindow(ANativeWindow* window) {
    std::lock_guard<std::mutex> lock(render_mutex_);
    if (window_ == window && window != nullptr) {
      return;
    }

    if (window_ != nullptr) {
      onWindowDestroyed();
      if (canvas_ != nullptr) {
        canvas_->removeFromWindow();
        canvas_.reset();
      }
      window_android_.reset();
      visage::WindowAndroid::SetGlobalNativeWindow(nullptr);
      visage::Renderer::setNativeWindow(nullptr, 0, 0, vsync_);
      ANativeWindow_release(window_);
      window_ = nullptr;
    }

    if (window != nullptr) {
      ANativeWindow_acquire(window);
      window_ = window;

      visage::WindowAndroid::SetGlobalNativeWindow(window_);
      visage::WindowAndroid::SetGlobalDpiScale(dpi_scale_);

      int width = ANativeWindow_getWidth(window_);
      int height = ANativeWindow_getHeight(window_);
      if (width <= 0) {
        width = 1080;
      }
      if (height <= 0) {
        height = 2400;
      }
      width_ = width;
      height_ = height;

      if (!visage::Renderer::instance().initialized()) {
        visage::Renderer::instance().initialize(window_, nullptr);
      }
      else {
        visage::Renderer::setNativeWindow(window_, width_, height_, vsync_);
      }

      VISAGE_LOG("setWindow: %p (width=%d, height=%d, dpi_scale=%.2f)", static_cast<void*>(window),
                 width, height, dpi_scale_);

      window_android_ = std::make_unique<visage::WindowAndroid>(width_, height_);
      window_android_->setDpiScale(dpi_scale_);
      canvas_ = std::make_unique<visage::Canvas>();
      canvas_->pairToWindow(window_, width_, height_);
      canvas_->setDpiScale(dpi_scale_);
      canvas_->setLogicalPixelScale();

      const float logical_w = logicalWidth();
      const float logical_h = logicalHeight();
      onResize(logical_w, logical_h);
      onWindowCreated();
      requestRedraw(4);
    }
  }

  void Activity::destroyWindow() {
    setWindow(nullptr);
  }

  void Activity::resize(int width, int height) {
    if (width <= 0 || height <= 0) {
      return;
    }

    std::lock_guard<std::mutex> lock(render_mutex_);
    const bool size_changed = (width != width_ || height != height_);
    width_ = width;
    height_ = height;

    if (window_android_ != nullptr) {
      window_android_->windowContentsResized(width, height);
    }
    if (canvas_ != nullptr && window_ != nullptr) {
      canvas_->pairToWindow(window_, width, height);
      canvas_->setDpiScale(dpi_scale_);
      canvas_->setLogicalPixelScale();
      if (size_changed) {
        visage::Renderer::resetResolution(width_, height_, vsync_);
      }
    }

    const float logical_w = logicalWidth();
    const float logical_h = logicalHeight();
    onResize(logical_w, logical_h);
    requestRedraw(2);
  }

  void Activity::renderFrame() {
    std::lock_guard<std::mutex> lock(render_mutex_);
    if (window_ == nullptr || canvas_ == nullptr || is_paused_.load(std::memory_order_acquire)) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double time_sec = std::chrono::duration<double>(now - start_time_).count();

    canvas_->clearDrawnShapes();
    canvas_->updateTime(time_sec);
    canvas_->setDpiScale(dpi_scale_);
    canvas_->setLogicalPixelScale();

    onDraw(*canvas_);
    canvas_->submit();

    frame_count_++;
    if (frame_count_ <= 5 || frame_count_ % 300 == 0) {
      VISAGE_LOG("Frame #%u submitted successfully", frame_count_);
    }

    int frames = redraw_frames_.load(std::memory_order_relaxed);
    if (frames > 0) {
      redraw_frames_.fetch_sub(1, std::memory_order_relaxed);
    }
  }

  void Activity::resolveInsets() {
    const float safe_dpi = dpi_scale_ > 0.0f ? dpi_scale_ : 1.0f;

    if (app_ == nullptr || app_->activity == nullptr || app_->activity->vm == nullptr ||
        app_->activity->clazz == nullptr) {
      insets_.statusBarHeightDp = kDefaultStatusBarDp;
      insets_.statusBarHeightPx = static_cast<int>(std::round(kDefaultStatusBarDp * safe_dpi));
      insets_.navigationBarHeightDp = 0.0f;
      insets_.navigationBarHeightPx = 0;
      return;
    }

    ANativeActivity* activity = app_->activity;
    JNIEnv* env = nullptr;
    jint get_env_result = activity->vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    bool did_attach = false;

    if (get_env_result == JNI_EDETACHED) {
      if (activity->vm->AttachCurrentThread(&env, nullptr) == JNI_OK) {
        did_attach = true;
      }
      else {
        env = nullptr;
      }
    }

    if (env == nullptr) {
      insets_.statusBarHeightDp = kDefaultStatusBarDp;
      insets_.statusBarHeightPx = static_cast<int>(std::round(kDefaultStatusBarDp * safe_dpi));
      insets_.navigationBarHeightDp = 0.0f;
      insets_.navigationBarHeightPx = 0;
      return;
    }

    auto cleanup = [&]() {
      if (env->ExceptionCheck()) {
        env->ExceptionClear();
      }
      if (did_attach) {
        activity->vm->DetachCurrentThread();
      }
    };

    jclass activity_class = env->GetObjectClass(activity->clazz);
    if (activity_class == nullptr) {
      cleanup();
      insets_.statusBarHeightDp = kDefaultStatusBarDp;
      insets_.statusBarHeightPx = static_cast<int>(std::round(kDefaultStatusBarDp * safe_dpi));
      return;
    }

    jmethodID get_resources_mid = env->GetMethodID(activity_class, "getResources",
                                                   "()Landroid/content/res/Resources;");
    env->DeleteLocalRef(activity_class);

    if (get_resources_mid == nullptr) {
      cleanup();
      insets_.statusBarHeightDp = kDefaultStatusBarDp;
      insets_.statusBarHeightPx = static_cast<int>(std::round(kDefaultStatusBarDp * safe_dpi));
      return;
    }

    jobject resources_obj = env->CallObjectMethod(activity->clazz, get_resources_mid);
    if (resources_obj == nullptr) {
      cleanup();
      insets_.statusBarHeightDp = kDefaultStatusBarDp;
      insets_.statusBarHeightPx = static_cast<int>(std::round(kDefaultStatusBarDp * safe_dpi));
      return;
    }

    jclass resources_class = env->GetObjectClass(resources_obj);
    if (resources_class == nullptr) {
      env->DeleteLocalRef(resources_obj);
      cleanup();
      insets_.statusBarHeightDp = kDefaultStatusBarDp;
      insets_.statusBarHeightPx = static_cast<int>(std::round(kDefaultStatusBarDp * safe_dpi));
      return;
    }

    jmethodID get_identifier_mid = env->GetMethodID(resources_class,
                                                    "getIdentifier", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)I");
    jmethodID get_dimen_px_mid = env->GetMethodID(resources_class, "getDimensionPixelSize", "(I)I");

    if (get_identifier_mid != nullptr && get_dimen_px_mid != nullptr) {
      jstring def_type = env->NewStringUTF("dimen");
      jstring def_package = env->NewStringUTF("android");

      jstring status_bar_name = env->NewStringUTF("status_bar_height");
      jint status_bar_res_id = env->CallIntMethod(resources_obj, get_identifier_mid,
                                                  status_bar_name, def_type, def_package);
      env->DeleteLocalRef(status_bar_name);

      if (status_bar_res_id > 0) {
        insets_.statusBarHeightPx = env->CallIntMethod(resources_obj, get_dimen_px_mid, status_bar_res_id);
        insets_.statusBarHeightDp = static_cast<float>(insets_.statusBarHeightPx) / safe_dpi;
      }

      jstring nav_bar_name = env->NewStringUTF("navigation_bar_height");
      jint nav_bar_res_id = env->CallIntMethod(resources_obj, get_identifier_mid, nav_bar_name,
                                               def_type, def_package);
      env->DeleteLocalRef(nav_bar_name);

      if (nav_bar_res_id > 0) {
        insets_.navigationBarHeightPx = env->CallIntMethod(resources_obj, get_dimen_px_mid, nav_bar_res_id);
        insets_.navigationBarHeightDp = static_cast<float>(insets_.navigationBarHeightPx) / safe_dpi;
      }

      env->DeleteLocalRef(def_type);
      env->DeleteLocalRef(def_package);
    }

    env->DeleteLocalRef(resources_class);
    env->DeleteLocalRef(resources_obj);
    cleanup();

    if (insets_.statusBarHeightDp <= 0.0f) {
      insets_.statusBarHeightDp = kDefaultStatusBarDp;
      insets_.statusBarHeightPx = static_cast<int>(std::round(kDefaultStatusBarDp * safe_dpi));
    }
  }

  void Activity::handleTouch(int action, float x_px, float y_px) {
    const float x_dp = dpi_scale_ > 0.0f ? x_px / dpi_scale_ : x_px;
    const float y_dp = dpi_scale_ > 0.0f ? y_px / dpi_scale_ : y_px;

    std::lock_guard<std::mutex> lock(render_mutex_);
    if (window_android_ != nullptr) {
      window_android_->handleTouch(action, x_dp, y_dp);
    }
    onTouchEvent(action, x_dp, y_dp);
    requestRedraw(2);
  }

  void Activity::run(struct android_app* state, std::unique_ptr<Activity> activity) {
    if (state == nullptr || activity == nullptr) {
      return;
    }

    VISAGE_LOG("Activity::run: starting loop");

    state->userData = activity.get();
    state->onAppCmd = OnVisageAppCmd;
    state->onInputEvent = OnVisageInputEvent;

    float dpi_scale = 2.75f;
    if (state->config != nullptr) {
      int32_t density = AConfiguration_getDensity(state->config);
      if (density > 0 && density != ACONFIGURATION_DENSITY_NONE && density != ACONFIGURATION_DENSITY_ANY) {
        dpi_scale = static_cast<float>(density) / 160.0f;
      }
    }
    if (dpi_scale < 1.0f) {
      dpi_scale = 2.75f;
    }

    VISAGE_LOG("Android Display DPI scale resolved: %.2f", dpi_scale);
    activity->setDpiScale(dpi_scale);
    activity->setApp(state);
    activity->resolveInsets();
    activity->onInit();

    while (true) {
      int events = 0;
      struct android_poll_source* source = nullptr;

      int timeout_ms = -1;
      if (activity->hasWindow()) {
        timeout_ms = activity->needsRedraw() ? 0 : -1;
      }

      while (ALooper_pollOnce(timeout_ms, nullptr, &events, reinterpret_cast<void**>(&source)) >= 0) {
        if (source != nullptr) {
          source->process(state, source);
        }
        if (state->destroyRequested != 0) {
          VISAGE_LOG("Activity::run: destroy requested, terminating looper");
          activity->onDestroy();
          activity->destroyWindow();
          return;
        }
        timeout_ms = 0;
      }

      if (activity->hasWindow() && activity->needsRedraw()) {
        activity->renderFrame();
      }
    }
  }

}  // namespace visage

#endif  // VISAGE_ANDROID || defined(__ANDROID__)
