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
