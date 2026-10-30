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

#include <algorithm>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

namespace visage {
  class UndoableAction {
  public:
    virtual void undo() = 0;
    virtual void redo() = 0;

    virtual ~UndoableAction() = default;

    void setup() const {
      if (setup_)
        setup_();
    }

    void setSetupFunction(std::function<void()> setup) { setup_ = std::move(setup); }

  private:
    std::function<void()> setup_ = nullptr;
  };

  class LambdaAction : public UndoableAction {
  public:
    LambdaAction(std::function<void()> undo_action, std::function<void()> redo_action) :
        undo_action_(std::move(undo_action)), redo_action_(std::move(redo_action)) { }

    void undo() override { undo_action_(); }
    void redo() override { redo_action_(); }

  private:
    std::function<void()> undo_action_;
    std::function<void()> redo_action_;
  };

  class UndoHistory {
  public:
    static constexpr int kMaxUndoHistory = 1000;

    class Listener {
    public:
      virtual ~Listener() = default;
      virtual void undoPerformed() = 0;
      virtual void redoPerformed() = 0;
      virtual void undoActionAdded() = 0;
      virtual void undoHistoryCleared() { }
    };

    UndoHistory() = default;

    void push(std::unique_ptr<UndoableAction> action);
    void undo();
    void redo();
    bool canUndo() const { return !actions_.empty(); }
    bool canRedo() const { return !undone_actions_.empty(); }
    void clearUndoHistory() {
      actions_.clear();
      undone_actions_.clear();
      notify(&Listener::undoHistoryCleared);
    }

    UndoableAction* peekUndo() const {
      if (actions_.empty())
        return nullptr;

      return actions_.back().get();
    }

    void addListener(Listener* listener) { listeners_.push_back(listener); }
    void removeListener(Listener* listener) {
      listeners_.erase(std::remove(listeners_.begin(), listeners_.end(), listener), listeners_.end());
    }

  private:
    void notify(void (Listener::*function)()) {
      std::vector<Listener*> listeners = listeners_;
      for (Listener* listener : listeners) {
        if (std::find(listeners_.begin(), listeners_.end(), listener) != listeners_.end())
          (listener->*function)();
      }
    }

    std::deque<std::unique_ptr<UndoableAction>> actions_;
    std::deque<std::unique_ptr<UndoableAction>> undone_actions_;
    std::vector<Listener*> listeners_;
    bool performing_action_ = false;
  };
}
