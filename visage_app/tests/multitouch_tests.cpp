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

#include "visage/app.h"

#include <catch2/catch_test_macros.hpp>
#include <visage/ui.h>

using namespace visage;

TEST_CASE("Multi-pointer: two touches dispatch to different frames", "[multitouch]") {
  ApplicationEditor editor;

  Frame left_frame;
  Frame right_frame;

  left_frame.setBounds(0, 0, 50, 100);
  right_frame.setBounds(50, 0, 50, 100);

  int left_down_count = 0;
  int right_down_count = 0;
  int left_drag_count = 0;
  int right_drag_count = 0;
  int left_up_count = 0;
  int right_up_count = 0;
  int left_last_pointer_id = -1;
  int right_last_pointer_id = -1;

  left_frame.onMouseDown() += [&](const MouseEvent& e) {
    left_down_count++;
    left_last_pointer_id = e.pointer_id;
  };
  right_frame.onMouseDown() += [&](const MouseEvent& e) {
    right_down_count++;
    right_last_pointer_id = e.pointer_id;
  };
  left_frame.onMouseDrag() += [&](const MouseEvent& e) {
    left_drag_count++;
  };
  right_frame.onMouseDrag() += [&](const MouseEvent& e) {
    right_drag_count++;
  };
  left_frame.onMouseUp() += [&](const MouseEvent& e) {
    left_up_count++;
  };
  right_frame.onMouseUp() += [&](const MouseEvent& e) {
    right_up_count++;
  };

  editor.addChild(&left_frame);
  editor.addChild(&right_frame);
  editor.setWindowless(100, 100);

  Window* window = editor.window();
  REQUIRE(window != nullptr);

  // Touch 0 on left frame (at pixel 25, 50)
  window->handleMouseDown(kMouseButtonLeft, 25, 50, kMouseButtonLeft, 0, 0);
  REQUIRE(left_down_count == 1);
  REQUIRE(right_down_count == 0);
  REQUIRE(left_last_pointer_id == 0);

  // Touch 1 on right frame (at pixel 75, 50)
  window->handleMouseDown(kMouseButtonLeft, 75, 50, kMouseButtonLeft, 0, 1);
  REQUIRE(left_down_count == 1);
  REQUIRE(right_down_count == 1);
  REQUIRE(right_last_pointer_id == 1);

  // Drag touch 0 (stays on left)
  window->handleMouseMove(26, 51, kMouseButtonLeft, 0, 0);
  REQUIRE(left_drag_count == 1);
  REQUIRE(right_drag_count == 0);

  // Drag touch 1 (stays on right)
  window->handleMouseMove(76, 51, kMouseButtonLeft, 0, 1);
  REQUIRE(left_drag_count == 1);
  REQUIRE(right_drag_count == 1);

  // Release touch 0
  window->handleMouseUp(kMouseButtonLeft, 26, 51, 0, 0, 0);
  REQUIRE(left_up_count == 1);
  REQUIRE(right_up_count == 0);

  // Release touch 1
  window->handleMouseUp(kMouseButtonLeft, 76, 51, 0, 0, 1);
  REQUIRE(left_up_count == 1);
  REQUIRE(right_up_count == 1);
}

TEST_CASE("Multi-pointer: drag is locked to the frame where touch began", "[multitouch]") {
  ApplicationEditor editor;

  Frame frame_a;
  Frame frame_b;

  frame_a.setBounds(0, 0, 50, 100);
  frame_b.setBounds(50, 0, 50, 100);

  int a_drag_count = 0;
  int b_drag_count = 0;

  frame_a.onMouseDrag() += [&](const MouseEvent& e) { a_drag_count++; };
  frame_b.onMouseDrag() += [&](const MouseEvent& e) { b_drag_count++; };

  editor.addChild(&frame_a);
  editor.addChild(&frame_b);
  editor.setWindowless(100, 100);

  Window* window = editor.window();

  // Touch starts on frame_a
  window->handleMouseDown(kMouseButtonLeft, 25, 50, kMouseButtonLeft, 0, 0);

  // Drag crosses into frame_b territory — should still dispatch to frame_a
  window->handleMouseMove(75, 50, kMouseButtonLeft, 0, 0);
  REQUIRE(a_drag_count == 1);
  REQUIRE(b_drag_count == 0);

  window->handleMouseUp(kMouseButtonLeft, 75, 50, 0, 0, 0);
}

TEST_CASE("Multi-pointer: single touch works identically to mouse", "[multitouch]") {
  ApplicationEditor editor;

  Frame target;
  target.setBounds(0, 0, 100, 100);

  int down_count = 0;
  int drag_count = 0;
  int up_count = 0;
  int last_pointer_id = -1;

  target.onMouseDown() += [&](const MouseEvent& e) {
    down_count++;
    last_pointer_id = e.pointer_id;
  };
  target.onMouseDrag() += [&](const MouseEvent& e) { drag_count++; };
  target.onMouseUp() += [&](const MouseEvent& e) { up_count++; };

  editor.addChild(&target);
  editor.setWindowless(100, 100);

  Window* window = editor.window();

  // Default pointer_id 0 (simulating mouse or single touch)
  window->handleMouseDown(kMouseButtonLeft, 50, 50, kMouseButtonLeft, 0);
  REQUIRE(down_count == 1);
  REQUIRE(last_pointer_id == 0);

  window->handleMouseMove(55, 55, kMouseButtonLeft, 0);
  REQUIRE(drag_count == 1);

  window->handleMouseUp(kMouseButtonLeft, 55, 55, 0, 0);
  REQUIRE(up_count == 1);
}

TEST_CASE("Multi-pointer: focus only changes from primary pointer", "[multitouch]") {
  ApplicationEditor editor;

  Frame frame_a;
  Frame frame_b;

  frame_a.setBounds(0, 0, 50, 100);
  frame_b.setBounds(50, 0, 50, 100);
  frame_a.setAcceptsKeystrokes(true);
  frame_b.setAcceptsKeystrokes(true);

  int a_focus_gained = 0;
  int b_focus_gained = 0;

  frame_a.onFocusChange() += [&](bool gained, bool) {
    if (gained)
      a_focus_gained++;
  };
  frame_b.onFocusChange() += [&](bool gained, bool) {
    if (gained)
      b_focus_gained++;
  };

  editor.addChild(&frame_a);
  editor.addChild(&frame_b);
  editor.setWindowless(100, 100);

  Window* window = editor.window();

  // Primary touch (pointer 0) on frame_a gives it focus
  window->handleMouseDown(kMouseButtonLeft, 25, 50, kMouseButtonLeft, 0, 0);
  REQUIRE(a_focus_gained == 1);
  REQUIRE(b_focus_gained == 0);

  // Secondary touch (pointer 1) on frame_b should NOT change focus
  window->handleMouseDown(kMouseButtonLeft, 75, 50, kMouseButtonLeft, 0, 1);
  REQUIRE(a_focus_gained == 1);
  REQUIRE(b_focus_gained == 0);

  window->handleMouseUp(kMouseButtonLeft, 25, 50, 0, 0, 0);
  window->handleMouseUp(kMouseButtonLeft, 75, 50, 0, 0, 1);
}

TEST_CASE("Multi-pointer: focus lost sends mouseUp to all pointers", "[multitouch]") {
  ApplicationEditor editor;

  Frame frame_a;
  Frame frame_b;

  frame_a.setBounds(0, 0, 50, 100);
  frame_b.setBounds(50, 0, 50, 100);

  int a_up_count = 0;
  int b_up_count = 0;

  frame_a.onMouseUp() += [&](const MouseEvent& e) { a_up_count++; };
  frame_b.onMouseUp() += [&](const MouseEvent& e) { b_up_count++; };

  editor.addChild(&frame_a);
  editor.addChild(&frame_b);
  editor.setWindowless(100, 100);

  Window* window = editor.window();

  // Two simultaneous touches
  window->handleMouseDown(kMouseButtonLeft, 25, 50, kMouseButtonLeft, 0, 0);
  window->handleMouseDown(kMouseButtonLeft, 75, 50, kMouseButtonLeft, 0, 1);

  // Focus lost should send mouseUp to both
  window->handleFocusLost();
  REQUIRE(a_up_count == 1);
  REQUIRE(b_up_count == 1);
}

TEST_CASE("Multi-pointer: three simultaneous touches", "[multitouch]") {
  ApplicationEditor editor;

  Frame frame_a;
  Frame frame_b;
  Frame frame_c;

  frame_a.setBounds(0, 0, 34, 100);
  frame_b.setBounds(34, 0, 33, 100);
  frame_c.setBounds(67, 0, 33, 100);

  int a_down = 0, b_down = 0, c_down = 0;
  int a_up = 0, b_up = 0, c_up = 0;

  frame_a.onMouseDown() += [&](const MouseEvent& e) { a_down++; };
  frame_b.onMouseDown() += [&](const MouseEvent& e) { b_down++; };
  frame_c.onMouseDown() += [&](const MouseEvent& e) { c_down++; };
  frame_a.onMouseUp() += [&](const MouseEvent& e) { a_up++; };
  frame_b.onMouseUp() += [&](const MouseEvent& e) { b_up++; };
  frame_c.onMouseUp() += [&](const MouseEvent& e) { c_up++; };

  editor.addChild(&frame_a);
  editor.addChild(&frame_b);
  editor.addChild(&frame_c);
  editor.setWindowless(100, 100);

  Window* window = editor.window();

  window->handleMouseDown(kMouseButtonLeft, 17, 50, kMouseButtonLeft, 0, 0);
  window->handleMouseDown(kMouseButtonLeft, 50, 50, kMouseButtonLeft, 0, 1);
  window->handleMouseDown(kMouseButtonLeft, 83, 50, kMouseButtonLeft, 0, 2);

  REQUIRE(a_down == 1);
  REQUIRE(b_down == 1);
  REQUIRE(c_down == 1);

  // Release in reverse order
  window->handleMouseUp(kMouseButtonLeft, 83, 50, 0, 0, 2);
  window->handleMouseUp(kMouseButtonLeft, 50, 50, 0, 0, 1);
  window->handleMouseUp(kMouseButtonLeft, 17, 50, 0, 0, 0);

  REQUIRE(a_up == 1);
  REQUIRE(b_up == 1);
  REQUIRE(c_up == 1);
}

TEST_CASE("Multi-pointer: move resets repeat click count for non-primary pointer", "[multitouch]") {
  ApplicationEditor editor;

  Frame target;
  target.setBounds(0, 0, 100, 100);

  int last_repeat_count = 0;

  target.onMouseDown() += [&](const MouseEvent& e) {
    last_repeat_count = e.repeatClickCount();
  };

  editor.addChild(&target);
  editor.setWindowless(100, 100);

  Window* window = editor.window();

  // Two quick taps at the same position with pointer 1 → double click
  window->handleMouseDown(kMouseButtonLeft, 50, 50, kMouseButtonLeft, 0, 1);
  window->handleMouseUp(kMouseButtonLeft, 50, 50, 0, 0, 1);
  window->handleMouseDown(kMouseButtonLeft, 50, 50, kMouseButtonLeft, 0, 1);
  REQUIRE(last_repeat_count == 2);
  window->handleMouseUp(kMouseButtonLeft, 50, 50, 0, 0, 1);

  // Move pointer 1 to a different position → should reset click count
  window->handleMouseMove(80, 80, 0, 0, 1);

  // Next tap at new position should be click count 1, not 3
  window->handleMouseDown(kMouseButtonLeft, 80, 80, kMouseButtonLeft, 0, 1);
  REQUIRE(last_repeat_count == 1);
  window->handleMouseUp(kMouseButtonLeft, 80, 80, 0, 0, 1);
}

TEST_CASE("Multi-pointer: focus lost uses per-pointer positions for mouseUp", "[multitouch]") {
  ApplicationEditor editor;

  Frame frame_a;
  Frame frame_b;

  frame_a.setBounds(0, 0, 50, 100);
  frame_b.setBounds(50, 0, 50, 100);

  float a_up_x = -1, b_up_x = -1;

  frame_a.onMouseUp() += [&](const MouseEvent& e) { a_up_x = e.position.x; };
  frame_b.onMouseUp() += [&](const MouseEvent& e) { b_up_x = e.position.x; };

  editor.addChild(&frame_a);
  editor.addChild(&frame_b);
  editor.setWindowless(100, 100);

  Window* window = editor.window();

  // Pointer 0 at x=25, pointer 1 at x=75
  window->handleMouseDown(kMouseButtonLeft, 25, 50, kMouseButtonLeft, 0, 0);
  window->handleMouseDown(kMouseButtonLeft, 75, 50, kMouseButtonLeft, 0, 1);

  // Move each pointer to distinct positions
  window->handleMouseMove(30, 50, kMouseButtonLeft, 0, 0);
  window->handleMouseMove(80, 50, kMouseButtonLeft, 0, 1);

  window->handleFocusLost();

  // Each frame should get mouseUp at its pointer's last position, not a shared one
  REQUIRE(a_up_x != b_up_x);
}
