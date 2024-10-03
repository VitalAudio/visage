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

#include "undo_history.h"

namespace visage {
  void UndoHistory::push(std::unique_ptr<UndoableAction> action) {
    undone_actions_.clear();
    if (actions_.size() >= kMaxUndoHistory)
      actions_.pop_front();

    actions_.push_back(std::move(action));

    for (Listener* listener : listeners_)
      listener->undoActionAdded();
  }

  void UndoHistory::undo() {
    if (!canUndo())
      return;

    std::unique_ptr<UndoableAction> action = std::move(actions_.back());
    actions_.pop_back();
    action->setup();
    action->undo();
    undone_actions_.push_back(std::move(action));

    for (Listener* listener : listeners_)
      listener->undoPerformed();
  }

  void UndoHistory::redo() {
    if (!canRedo())
      return;

    std::unique_ptr<UndoableAction> action = std::move(undone_actions_.back());
    undone_actions_.pop_back();
    action->setup();
    action->redo();
    actions_.push_back(std::move(action));

    for (Listener* listener : listeners_)
      listener->redoPerformed();
  }
}
