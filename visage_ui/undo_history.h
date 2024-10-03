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

#pragma once

#include <deque>
#include <functional>
#include <memory>

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
    }

    UndoableAction* peekUndo() const {
      if (actions_.empty())
        return nullptr;

      return actions_.back().get();
    }

    void addListener(Listener* listener) { listeners_.push_back(listener); }

  private:
    std::deque<std::unique_ptr<UndoableAction>> actions_;
    std::deque<std::unique_ptr<UndoableAction>> undone_actions_;
    std::vector<Listener*> listeners_;
  };
}
