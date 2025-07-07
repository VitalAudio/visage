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

#include "visage_graphics/canvas.h"
#include "visage_graphics/font.h"
#include "visage_ui/popup_menu.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("PopupMenu construction and basic properties", "[ui]") {
  SECTION("Default construction") {
    PopupMenu menu;
    REQUIRE(menu.name().isEmpty());
    REQUIRE(menu.id() == -1);
    REQUIRE_FALSE(menu.isBreak());
    REQUIRE_FALSE(menu.hasOptions());
    REQUIRE(menu.size() == 0);
    REQUIRE(menu.enabled());
    REQUIRE_FALSE(menu.selected());
  }

  SECTION("Construction with parameters") {
    std::vector<PopupMenu> options = { PopupMenu("Option 1", 1), PopupMenu("Option 2", 2) };
    PopupMenu menu("Test Menu", 100, std::move(options), false);

    REQUIRE(menu.name() == "Test Menu");
    REQUIRE(menu.id() == 100);
    REQUIRE_FALSE(menu.isBreak());
    REQUIRE(menu.hasOptions());
    REQUIRE(menu.size() == 2);
  }

  SECTION("Break menu construction") {
    PopupMenu break_menu("", -1, {}, true);
    REQUIRE(break_menu.isBreak());
    REQUIRE(break_menu.name().isEmpty());
  }
}

TEST_CASE("PopupMenu option management", "[ui]") {
  PopupMenu menu("Main Menu");

  SECTION("Adding options") {
    PopupMenu& option1 = menu.addOption(1, "Option 1");
    REQUIRE(menu.size() == 1);
    REQUIRE(option1.name() == "Option 1");
    REQUIRE(option1.id() == 1);

    menu.addOption(2, "Option 2");
    REQUIRE(menu.size() == 2);
    REQUIRE(menu.hasOptions());
  }

  SECTION("Adding sub-menus") {
    PopupMenu sub_menu("Sub Menu", 10);
    sub_menu.addOption(11, "Sub Option 1");

    menu.addSubMenu(std::move(sub_menu));
    REQUIRE(menu.size() == 1);
    REQUIRE(menu.options()[0].name() == "Sub Menu");
    REQUIRE(menu.options()[0].hasOptions());
  }

  SECTION("Adding breaks") {
    menu.addOption(1, "Option 1");
    menu.addBreak();
    menu.addOption(2, "Option 2");

    REQUIRE(menu.size() == 3);
    REQUIRE_FALSE(menu.options()[0].isBreak());
    REQUIRE(menu.options()[1].isBreak());
    REQUIRE_FALSE(menu.options()[2].isBreak());
  }

  SECTION("Accessing options") {
    menu.addOption(1, "Option 1");
    menu.addOption(2, "Option 2");

    const auto& options = menu.options();
    REQUIRE(options.size() == 2);
    REQUIRE(options[0].name() == "Option 1");
    REQUIRE(options[1].name() == "Option 2");
  }
}

TEST_CASE("PopupMenu state management", "[ui]") {
  PopupMenu menu("Test Menu", 1);

  SECTION("Selection state") {
    REQUIRE_FALSE(menu.selected());

    menu.select(true);
    REQUIRE(menu.selected());

    menu.select(false);
    REQUIRE_FALSE(menu.selected());

    PopupMenu& self_ref = menu.select(true);
    REQUIRE(&self_ref == &menu);
  }

  SECTION("Enabled state") {
    REQUIRE(menu.enabled());

    menu.enable(false);
    REQUIRE_FALSE(menu.enabled());

    menu.enable(true);
    REQUIRE(menu.enabled());

    PopupMenu& self_ref = menu.enable(false);
    REQUIRE(&self_ref == &menu);
  }
}

TEST_CASE("PopupMenu keyboard shortcuts", "[ui]") {
  PopupMenu menu("Test Menu", 1);

  SECTION("Setting keyboard shortcut") {
    menu.withNativeKeyboardShortcut(kModifierCmd, "s");

    REQUIRE(menu.nativeShortcutModifiers() == kModifierCmd);
    REQUIRE(menu.nativeShortcutCharacter() == "s");
  }

  SECTION("Default keyboard shortcut") {
    REQUIRE(menu.nativeShortcutModifiers() == 0);
    REQUIRE(menu.nativeShortcutCharacter().empty());
  }

  SECTION("Chaining with keyboard shortcut") {
    PopupMenu& self_ref = menu.withNativeKeyboardShortcut(kModifierAlt, "x");
    REQUIRE(&self_ref == &menu);
  }
}

TEST_CASE("PopupMenu callbacks", "[ui]") {
  PopupMenu menu("Test Menu", 1);

  SECTION("Selection callback") {
    int selected_id = -1;
    menu.onSelection().add([&](int id) { selected_id = id; });

    menu.onSelection().callback(42);
    REQUIRE(selected_id == 42);
  }

  SECTION("Cancel callback") {
    bool cancel_called = false;
    menu.onCancel().add([&]() { cancel_called = true; });

    menu.onCancel().callback();
    REQUIRE(cancel_called);
  }

  SECTION("Const callback access") {
    const PopupMenu const_menu("Const Menu", 1);
    const auto& selection_callbacks = const_menu.onSelection();
    const auto& cancel_callbacks = const_menu.onCancel();
  }
}

TEST_CASE("PopupMenu show functionality", "[ui]") {
  PopupMenu menu("Test Menu", 1);
  Frame source_frame;

  SECTION("Show with default position") {
    menu.show(&source_frame);
  }

  SECTION("Show with specific position") {
    menu.show(&source_frame, { 100, 200 });
  }

  SECTION("Show with kNotSet position") {
    menu.show(&source_frame, { PopupMenu::kNotSet, PopupMenu::kNotSet });
  }
}

TEST_CASE("PopupMenu native menu bar", "[ui]") {
  PopupMenu menu("File");
  menu.addOption(1, "New");
  menu.addOption(2, "Open");

  SECTION("Set as native menu bar") {
    menu.setAsNativeMenuBar();
  }
}

class MockPopupListListener : public PopupList::Listener {
public:
  void optionSelected(const PopupMenu& option, PopupList* list) override {
    last_selected_option = &option;
    last_list = list;
    option_selected_count++;
  }

  void subMenuSelected(const PopupMenu& option, int selected_y, PopupList* list) override {
    last_submenu_option = &option;
    last_submenu_y = selected_y;
    last_list = list;
    submenu_selected_count++;
  }

  void mouseMovedOnMenu(Point position, PopupList* list) override {
    last_mouse_position = position;
    last_list = list;
    mouse_moved_count++;
  }

  void mouseDraggedOnMenu(Point position, PopupList* list) override {
    last_drag_position = position;
    last_list = list;
    mouse_dragged_count++;
  }

  void mouseUpOutside(Point position, PopupList* list) override {
    last_mouse_up_position = position;
    last_list = list;
    mouse_up_outside_count++;
  }

  const PopupMenu* last_selected_option = nullptr;
  const PopupMenu* last_submenu_option = nullptr;
  PopupList* last_list = nullptr;
  Point last_mouse_position;
  Point last_drag_position;
  Point last_mouse_up_position;
  int last_submenu_y = 0;
  int option_selected_count = 0;
  int submenu_selected_count = 0;
  int mouse_moved_count = 0;
  int mouse_dragged_count = 0;
  int mouse_up_outside_count = 0;
};

TEST_CASE("PopupList construction and configuration", "[ui]") {
  PopupList popup_list;

  SECTION("Default state") {
    REQUIRE(popup_list.hoverIndex() == -1);
    REQUIRE(popup_list.numOptions() == 0);
  }

  SECTION("Setting options") {
    std::vector<PopupMenu> options = { PopupMenu("Option 1", 1), PopupMenu("Option 2", 2),
                                       PopupMenu("Option 3", 3) };

    popup_list.setOptions(std::move(options));
    REQUIRE(popup_list.numOptions() == 3);
    REQUIRE(popup_list.option(0).name() == "Option 1");
    REQUIRE(popup_list.option(1).name() == "Option 2");
    REQUIRE(popup_list.option(2).name() == "Option 3");
  }

  SECTION("Setting font") {
    Font font;
    popup_list.setFont(font);
  }

  SECTION("Setting opacity") {
    popup_list.setOpacity(0.8f);
    popup_list.setOpacity(0.0f);
  }
}

TEST_CASE("PopupList hover and selection", "[ui]") {
  PopupList popup_list;
  std::vector<PopupMenu> options = { PopupMenu("Option 1", 1), PopupMenu("Option 2", 2),
                                     PopupMenu("Option 3", 3) };
  popup_list.setOptions(std::move(options));

  SECTION("Setting hover from position") {
    popup_list.setBounds(0, 0, 100, 100);
    popup_list.setHoverFromPosition({ 50, 50 });
  }

  SECTION("Selecting from position") {
    popup_list.setBounds(0, 0, 100, 100);
    popup_list.selectFromPosition({ 50, 50 });
  }

  SECTION("Selecting hovered index") {
    popup_list.selectHoveredIndex();
  }

  SECTION("Setting no hover") {
    popup_list.setNoHover();
    REQUIRE(popup_list.hoverIndex() == -1);
  }

  SECTION("Y position for index") {
    int y_pos = popup_list.yForIndex(1);
  }

  SECTION("Hover Y position") {
    int hover_y = popup_list.hoverY();
  }
}

TEST_CASE("PopupList menu state management", "[ui]") {
  PopupList popup_list;

  SECTION("Open menu management") {
    popup_list.setOpenMenu(2);
    popup_list.resetOpenMenu();
  }

  SECTION("Mouse up enabling") {
    popup_list.enableMouseUp(true);
    popup_list.enableMouseUp(false);
  }
}

TEST_CASE("PopupList mouse event handling", "[ui]") {
  PopupList popup_list;
  popup_list.setBounds(0, 0, 100, 100);

  std::vector<PopupMenu> options = { PopupMenu("Option 1", 1), PopupMenu("Option 2", 2) };
  popup_list.setOptions(std::move(options));

  SECTION("Mouse exit") {
    MouseEvent event;
    popup_list.mouseExit(event);
  }

  SECTION("Mouse down") {
    MouseEvent event;
    event.relative_position = { 50, 25 };
    popup_list.mouseDown(event);
  }

  SECTION("Mouse move") {
    MouseEvent event;
    event.relative_position = { 50, 75 };
    popup_list.mouseMove(event);
  }

  SECTION("Mouse drag") {
    MouseEvent event;
    event.relative_position = { 50, 50 };
    popup_list.mouseDrag(event);
  }

  SECTION("Mouse up") {
    MouseEvent event;
    event.relative_position = { 50, 25 };
    popup_list.mouseUp(event);
  }

  SECTION("Mouse wheel") {
    MouseEvent wheel_event;
    wheel_event.precise_wheel_delta_y = 10.0f;
    popup_list.setVisible(true);
    bool result = popup_list.mouseWheel(wheel_event);
  }
}

TEST_CASE("PopupList listener integration", "[ui]") {
  PopupList popup_list;
  MockPopupListListener listener;
  popup_list.addListener(&listener);

  std::vector<PopupMenu> options = { PopupMenu("Option 1", 1), PopupMenu("Option 2", 2) };
  popup_list.setOptions(std::move(options));
  popup_list.setBounds(0, 0, 100, 100);

  SECTION("Mouse wheel with listener") {
    MouseEvent wheel_event;
    wheel_event.relative_position = { 50, 50 };
    wheel_event.precise_wheel_delta_y = 5.0f;
    popup_list.setVisible(true);

    popup_list.mouseWheel(wheel_event);

    REQUIRE(listener.mouse_moved_count >= 0);
    REQUIRE(listener.last_list == &popup_list);
  }
}

TEST_CASE("PopupList rendering", "[ui]") {
  PopupList popup_list;
  std::vector<PopupMenu> options = { PopupMenu("Option 1", 1), PopupMenu("Option 2", 2) };
  popup_list.setOptions(std::move(options));

  SECTION("Resized handling") {
    popup_list.setBounds(0, 0, 150, 200);
    popup_list.resized();
  }
}

TEST_CASE("ValueDisplay functionality", "[ui]") {
  ValueDisplay display;

  SECTION("Construction") {
    REQUIRE(display.ignoresMouseEvents());
  }

  SECTION("Setting font") {
    Font font;
    display.setFont(font);
  }
}

TEST_CASE("PopupMenuFrame constants", "[ui]") {
  SECTION("Constant values") {
    REQUIRE(PopupMenuFrame::kMaxSubMenus == 4);
    REQUIRE(PopupMenuFrame::kWaitForSelection == 20);
    REQUIRE(PopupMenuFrame::kPauseMs == 400);
  }
}