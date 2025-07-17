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

#include "events.h"

#include "frame.h"
#include "visage_utils/time_utils.h"

namespace visage {

  EventTimer::~EventTimer() {
    if (isRunning())
      stopTimer();
  }

  void EventTimer::startTimer(int ms) {
    VISAGE_ASSERT(ms > 0);

    if (ms > 0) {
      if (!isRunning())
        EventManager::instance().addTimer(this);

      last_run_time_ = time::milliseconds();
      ms_ = ms;
    }
  }

  void EventTimer::stopTimer() {
    if (isRunning()) {
      EventManager::instance().removeTimer(this);
      ms_ = 0;
    }
  }

  inline bool EventTimer::checkTimer(long long current_time) {
    VISAGE_ASSERT(isRunning());

    if (current_time - last_run_time_ >= ms_) {
      last_run_time_ = current_time;
      notifyTimerCallback();
      return true;
    }

    return false;
  }

  void EventManager::addTimer(EventTimer* timer) {
    timers_.push_back(timer);
  }

  void EventManager::removeTimer(const EventTimer* timer) {
    auto it = std::find(timers_.begin(), timers_.end(), timer);
    if (it != timers_.end())
      timers_.erase(it);
  }

  void EventManager::addCallback(std::function<void()> callback) {
    callbacks_.push_back(std::move(callback));
  }

  void EventManager::checkEventTimers() {
    long long current_time = time::milliseconds();
    std::vector<EventTimer*> timers = timers_;
    std::vector<std::function<void()>> callbacks = std::move(callbacks_);

    for (auto timer : timers)
      timer->checkTimer(current_time);

    for (auto& callback : callbacks)
      callback();
  }

  MouseEvent MouseEvent::relativeTo(const Frame* new_frame) const {
    MouseEvent copy = *this;
    copy.position = copy.window_position - new_frame->positionInWindow();
    copy.frame = new_frame;
    return copy;
  }
}
