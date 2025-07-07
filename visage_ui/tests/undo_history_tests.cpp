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

#include "visage_ui/undo_history.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

class TestAction : public UndoableAction {
public:
  TestAction(int& value, int undo_val, int redo_val) :
      value_(value), undo_value_(undo_val), redo_value_(redo_val) { }

  void undo() override {
    undo_count_++;
    value_ = undo_value_;
  }

  void redo() override {
    redo_count_++;
    value_ = redo_value_;
  }

  int undo_count_ = 0;
  int redo_count_ = 0;

private:
  int& value_;
  int undo_value_;
  int redo_value_;
};

class TestListener : public UndoHistory::Listener {
public:
  void undoPerformed() override { undo_performed_count++; }
  void redoPerformed() override { redo_performed_count++; }
  void undoActionAdded() override { action_added_count++; }

  int undo_performed_count = 0;
  int redo_performed_count = 0;
  int action_added_count = 0;
};

TEST_CASE("UndoableAction base functionality", "[ui]") {
  int value = 0;
  TestAction action(value, 10, 20);

  SECTION("Undo operation") {
    value = 5;
    action.undo();
    REQUIRE(value == 10);
    REQUIRE(action.undo_count_ == 1);
  }

  SECTION("Redo operation") {
    value = 5;
    action.redo();
    REQUIRE(value == 20);
    REQUIRE(action.redo_count_ == 1);
  }

  SECTION("Setup function") {
    bool setup_called = false;
    action.setSetupFunction([&setup_called]() { setup_called = true; });

    action.setup();
    REQUIRE(setup_called);
  }

  SECTION("Setup function not set") {
    action.setup();
  }
}

TEST_CASE("LambdaAction functionality", "[ui]") {
  int value = 0;
  bool undo_called = false;
  bool redo_called = false;

  auto undo_func = [&]() {
    value = 10;
    undo_called = true;
  };
  auto redo_func = [&]() {
    value = 20;
    redo_called = true;
  };

  LambdaAction action(undo_func, redo_func);

  SECTION("Lambda undo") {
    action.undo();
    REQUIRE(value == 10);
    REQUIRE(undo_called);
  }

  SECTION("Lambda redo") {
    action.redo();
    REQUIRE(value == 20);
    REQUIRE(redo_called);
  }
}

TEST_CASE("UndoHistory basic operations", "[ui]") {
  UndoHistory history;

  SECTION("Initial state") {
    REQUIRE_FALSE(history.canUndo());
    REQUIRE_FALSE(history.canRedo());
    REQUIRE(history.peekUndo() == nullptr);
  }

  SECTION("Adding actions") {
    int value = 0;
    auto action = std::make_unique<TestAction>(value, 5, 10);

    history.push(std::move(action));
    REQUIRE(history.canUndo());
    REQUIRE_FALSE(history.canRedo());
    REQUIRE(history.peekUndo() != nullptr);
  }

  SECTION("Single undo/redo cycle") {
    int value = 0;
    auto action = std::make_unique<TestAction>(value, 5, 10);
    TestAction* action_ptr = action.get();

    history.push(std::move(action));

    value = 3;
    history.undo();
    REQUIRE(value == 5);
    REQUIRE(action_ptr->undo_count_ == 1);
    REQUIRE_FALSE(history.canUndo());
    REQUIRE(history.canRedo());

    history.redo();
    REQUIRE(value == 10);
    REQUIRE(action_ptr->redo_count_ == 1);
    REQUIRE(history.canUndo());
    REQUIRE_FALSE(history.canRedo());
  }

  SECTION("Multiple actions") {
    int value1 = 0, value2 = 0;
    auto action1 = std::make_unique<TestAction>(value1, 10, 20);
    auto action2 = std::make_unique<TestAction>(value2, 30, 40);

    history.push(std::move(action1));
    history.push(std::move(action2));

    REQUIRE(history.canUndo());
    REQUIRE_FALSE(history.canRedo());

    history.undo();
    REQUIRE(value2 == 30);
    REQUIRE(value1 == 0);

    history.undo();
    REQUIRE(value1 == 10);
    REQUIRE_FALSE(history.canUndo());
    REQUIRE(history.canRedo());
  }

  SECTION("Redo invalidation") {
    int value = 0;
    auto action1 = std::make_unique<TestAction>(value, 10, 20);
    auto action2 = std::make_unique<TestAction>(value, 30, 40);

    history.push(std::move(action1));
    history.undo();
    REQUIRE(history.canRedo());

    history.push(std::move(action2));
    REQUIRE_FALSE(history.canRedo());
  }

  SECTION("Clear history") {
    int value = 0;
    auto action = std::make_unique<TestAction>(value, 5, 10);

    history.push(std::move(action));
    history.undo();

    REQUIRE_FALSE(history.canUndo());
    REQUIRE(history.canRedo());

    history.clearUndoHistory();
    REQUIRE_FALSE(history.canUndo());
    REQUIRE_FALSE(history.canRedo());
    REQUIRE(history.peekUndo() == nullptr);
  }
}

TEST_CASE("UndoHistory listener notifications", "[ui]") {
  UndoHistory history;
  TestListener listener;
  history.addListener(&listener);

  SECTION("Action added notification") {
    int value = 0;
    auto action = std::make_unique<TestAction>(value, 5, 10);

    history.push(std::move(action));
    REQUIRE(listener.action_added_count == 1);
  }

  SECTION("Undo performed notification") {
    int value = 0;
    auto action = std::make_unique<TestAction>(value, 5, 10);

    history.push(std::move(action));
    history.undo();
    REQUIRE(listener.undo_performed_count == 1);
  }

  SECTION("Redo performed notification") {
    int value = 0;
    auto action = std::make_unique<TestAction>(value, 5, 10);

    history.push(std::move(action));
    history.undo();
    history.redo();
    REQUIRE(listener.redo_performed_count == 1);
  }

  SECTION("Multiple listeners") {
    TestListener listener2;
    history.addListener(&listener2);

    int value = 0;
    auto action = std::make_unique<TestAction>(value, 5, 10);

    history.push(std::move(action));
    REQUIRE(listener.action_added_count == 1);
    REQUIRE(listener2.action_added_count == 1);

    history.undo();
    REQUIRE(listener.undo_performed_count == 1);
    REQUIRE(listener2.undo_performed_count == 1);
  }
}

TEST_CASE("UndoHistory edge cases", "[ui]") {
  UndoHistory history;

  SECTION("Undo with no actions") {
    history.undo();
    REQUIRE_FALSE(history.canUndo());
    REQUIRE_FALSE(history.canRedo());
  }

  SECTION("Redo with no undone actions") {
    history.redo();
    REQUIRE_FALSE(history.canUndo());
    REQUIRE_FALSE(history.canRedo());
  }

  SECTION("Peek undo with empty history") {
    REQUIRE(history.peekUndo() == nullptr);
  }

  SECTION("Peek undo after clear") {
    int value = 0;
    auto action = std::make_unique<TestAction>(value, 5, 10);

    history.push(std::move(action));
    REQUIRE(history.peekUndo() != nullptr);

    history.clearUndoHistory();
    REQUIRE(history.peekUndo() == nullptr);
  }
}

TEST_CASE("UndoHistory complex scenarios", "[ui]") {
  UndoHistory history;
  int value = 0;

  SECTION("Multiple undo/redo operations") {
    auto action1 = std::make_unique<TestAction>(value, 10, 20);
    auto action2 = std::make_unique<TestAction>(value, 30, 40);
    auto action3 = std::make_unique<TestAction>(value, 50, 60);

    history.push(std::move(action1));
    history.push(std::move(action2));
    history.push(std::move(action3));

    history.undo();
    REQUIRE(value == 50);
    history.undo();
    REQUIRE(value == 30);

    history.redo();
    REQUIRE(value == 40);
    history.redo();
    REQUIRE(value == 60);

    REQUIRE(history.canUndo());
    REQUIRE_FALSE(history.canRedo());
  }

  SECTION("Interleaved operations") {
    auto action1 = std::make_unique<TestAction>(value, 10, 20);
    auto action2 = std::make_unique<TestAction>(value, 30, 40);
    auto action3 = std::make_unique<TestAction>(value, 50, 60);

    history.push(std::move(action1));
    history.undo();
    REQUIRE(value == 10);

    history.push(std::move(action2));
    REQUIRE(value == 10);
    REQUIRE_FALSE(history.canRedo());

    history.undo();
    REQUIRE(value == 30);

    history.redo();
    REQUIRE(value == 40);
  }
}

TEST_CASE("UndoHistory setup function integration", "[ui]") {
  UndoHistory history;
  int value = 0;
  bool setup_called = false;

  auto action = std::make_unique<TestAction>(value, 10, 20);
  action->setSetupFunction([&setup_called]() { setup_called = true; });

  history.push(std::move(action));

  SECTION("Setup called on undo") {
    history.undo();
    REQUIRE(setup_called);
  }

  SECTION("Setup called on redo") {
    history.undo();
    setup_called = false;

    history.redo();
    REQUIRE(setup_called);
  }
}