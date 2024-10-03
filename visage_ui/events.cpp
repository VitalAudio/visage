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

#include "events.h"

#include "ui_frame.h"
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
        EventManager::getInstance().addTimer(this);

      last_run_time_ = time::getMilliseconds();
      ms_ = ms;
    }
  }

  void EventTimer::stopTimer() {
    if (isRunning()) {
      EventManager::getInstance().removeTimer(this);
      ms_ = 0;
    }
  }

  inline bool EventTimer::checkTimer(long long current_time) {
    VISAGE_ASSERT(isRunning());

    if (current_time - last_run_time_ >= ms_) {
      timerCallback();
      last_run_time_ = current_time;
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
    long long current_time = time::getMilliseconds();
    std::vector<EventTimer*> timers = timers_;
    std::vector<std::function<void()>> callbacks = std::move(callbacks_);

    for (auto timer : timers)
      timer->checkTimer(current_time);

    for (auto& callback : callbacks)
      callback();
  }

  MouseEvent MouseEvent::relativeTo(const UiFrame* new_frame) const {
    MouseEvent copy = *this;
    copy.position = copy.window_position - new_frame->getPositionInWindow();
    copy.frame = new_frame;
    return copy;
  }
}
