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

#include "visage_ui/layout.h"
#include "visage_utils/dimension.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;
using namespace visage::dimension;

TEST_CASE("Layout padding", "[ui]") {
  Layout layout;
  layout.setFlex(true);

  Layout child;
  std::vector<Bounds> results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(0, 0, 1000, 0));

  layout.setPadding(10_px);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(20, 20, 960, 0));

  child.setFlexGrow(1.0f);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(20, 20, 960, 460));

  layout.setPadding(10_vmin);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(50, 50, 900, 400));

  layout.setPaddingLeft(10);
  layout.setPaddingRight(5);
  layout.setPaddingTop(20);
  layout.setPaddingBottom(30);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(10, 20, 985, 450));
}

TEST_CASE("Layout margin", "[ui]") {
  Layout layout;
  layout.setFlex(true);

  Layout child;
  std::vector<Bounds> results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(0, 0, 1000, 0));

  child.setMargin(10_px);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(20, 20, 960, 0));

  child.setFlexGrow(1.0f);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(20, 20, 960, 460));

  child.setMargin(10_vmin);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(50, 50, 900, 400));

  child.setMarginLeft(10);
  child.setMarginRight(5);
  child.setMarginTop(20);
  child.setMarginBottom(30);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(10, 20, 985, 450));
}

TEST_CASE("Layout padding and margin", "[ui]") {
  Layout layout;
  layout.setFlex(true);

  Layout child;
  std::vector<Bounds> results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(0, 0, 1000, 0));

  layout.setPadding(5_px);
  child.setMargin(5_px);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(20, 20, 960, 0));

  child.setFlexGrow(1.0f);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(20, 20, 960, 460));

  layout.setPadding(10_vmin);
  child.setMargin(10_vmin);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(90, 90, 820, 320));

  layout.setPaddingLeft(5);
  layout.setPaddingRight(3);
  layout.setPaddingTop(10);
  layout.setPaddingBottom(15);
  child.setMarginLeft(5);
  child.setMarginRight(3);
  child.setMarginTop(10);
  child.setMarginBottom(15);
  results = layout.flexPositions({ &child }, { 0, 0, 1000, 500 }, 2.0f);
  REQUIRE(results.size() == 1);
  REQUIRE(results[0] == Bounds(10, 20, 984, 450));
}

TEST_CASE("Layout flex grow", "[ui]") {
  Layout layout;
  layout.setPadding(100);
  layout.setFlex(true);
  layout.setFlexGap(100);

  Layout child0, child1, child2, child3;
  std::vector<const Layout*> layouts = { &child0, &child1, &child2, &child3 };

  child0.setWidth(300);
  child0.setHeight(100);
  child0.setFlexShrink(2.0f);
  child1.setWidth(10_vw);
  child2.setHeight(300);
  child3.setHeight(100);

  std::vector<Bounds> results = layout.flexPositions(layouts, { 0, 0, 1000, 1600 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 300, 100));
  REQUIRE(results[1] == Bounds(100, 300, 80, 0));
  REQUIRE(results[2] == Bounds(100, 400, 800, 300));
  REQUIRE(results[3] == Bounds(100, 800, 800, 100));

  child0.setFlexGrow(2.0f);
  results = layout.flexPositions(layouts, { 0, 0, 1000, 1600 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 300, 700));
  REQUIRE(results[1] == Bounds(100, 900, 80, 0));
  REQUIRE(results[2] == Bounds(100, 1000, 800, 300));
  REQUIRE(results[3] == Bounds(100, 1400, 800, 100));

  child1.setFlexGrow(1.0f);
  child2.setFlexGrow(3.0f);
  results = layout.flexPositions(layouts, { 0, 0, 1000, 1600 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 300, 300));
  REQUIRE(results[1] == Bounds(100, 500, 80, 100));
  REQUIRE(results[2] == Bounds(100, 700, 800, 600));
  REQUIRE(results[3] == Bounds(100, 1400, 800, 100));

  layout.setFlexReverseDirection(true);
  results = layout.flexPositions(layouts, { 0, 0, 1000, 1600 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 1200, 300, 300));
  REQUIRE(results[1] == Bounds(100, 1000, 80, 100));
  REQUIRE(results[2] == Bounds(100, 300, 800, 600));
  REQUIRE(results[3] == Bounds(100, 100, 800, 100));

  layout.setFlexReverseDirection(false);
  layout.setFlexRows(false);
  results = layout.flexPositions(layouts, { 0, 0, 1200, 1000 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 400, 100));
  REQUIRE(results[1] == Bounds(600, 100, 150, 800));
  REQUIRE(results[2] == Bounds(850, 100, 150, 300));
  REQUIRE(results[3] == Bounds(1100, 100, 0, 100));

  layout.setFlexReverseDirection(true);
  results = layout.flexPositions(layouts, { 0, 0, 1200, 1000 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(700, 100, 400, 100));
  REQUIRE(results[1] == Bounds(450, 100, 150, 800));
  REQUIRE(results[2] == Bounds(200, 100, 150, 300));
  REQUIRE(results[3] == Bounds(100, 100, 0, 100));
}

TEST_CASE("Layout flex shrink", "[ui]") {
  Layout layout;
  layout.setPadding(100);
  layout.setFlex(true);
  layout.setFlexGap(100);

  Layout child0, child1, child2, child3;
  std::vector<const Layout*> layouts = { &child0, &child1, &child2, &child3 };

  child0.setWidth(300);
  child0.setHeight(100);
  child0.setFlexGrow(2.0f);
  child1.setWidth(10_vw);
  child1.setFlexGrow(2.0f);
  child2.setHeight(300);
  child3.setHeight(100);

  std::vector<Bounds> results = layout.flexPositions(layouts, { 0, 0, 1000, 400 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 300, 100));
  REQUIRE(results[1] == Bounds(100, 300, 80, 0));
  REQUIRE(results[2] == Bounds(100, 400, 800, 300));
  REQUIRE(results[3] == Bounds(100, 800, 800, 100));

  child0.setFlexShrink(1.0f);
  results = layout.flexPositions(layouts, { 0, 0, 1000, 950 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 300, 50));
  REQUIRE(results[1] == Bounds(100, 250, 80, 0));
  REQUIRE(results[2] == Bounds(100, 350, 800, 300));
  REQUIRE(results[3] == Bounds(100, 750, 800, 100));

  results = layout.flexPositions(layouts, { 0, 0, 1000, 700 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 300, 0));
  REQUIRE(results[1] == Bounds(100, 200, 80, 0));
  REQUIRE(results[2] == Bounds(100, 300, 800, 300));
  REQUIRE(results[3] == Bounds(100, 700, 800, 100));

  child0.setFlexShrink(2.0f);
  child2.setFlexShrink(1.0f);
  results = layout.flexPositions(layouts, { 0, 0, 1000, 800 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 300, 20));
  REQUIRE(results[1] == Bounds(100, 220, 80, 0));
  REQUIRE(results[2] == Bounds(100, 320, 800, 180));
  REQUIRE(results[3] == Bounds(100, 600, 800, 100));

  layout.setFlexReverseDirection(true);
  results = layout.flexPositions(layouts, { 0, 0, 1000, 800 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 680, 300, 20));
  REQUIRE(results[1] == Bounds(100, 580, 80, 0));
  REQUIRE(results[2] == Bounds(100, 300, 800, 180));
  REQUIRE(results[3] == Bounds(100, 100, 800, 100));

  layout.setFlexReverseDirection(false);

  child0.setWidth(100);
  child0.setHeight(300);

  child1.setWidth({});
  child1.setHeight(10_vh);

  child2.setWidth(300);
  child2.setHeight({});

  child3.setWidth(100);
  child3.setHeight({});
  layout.setFlexRows(false);
  results = layout.flexPositions(layouts, { 0, 0, 800, 1000 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(100, 100, 20, 300));
  REQUIRE(results[1] == Bounds(220, 100, 0, 80));
  REQUIRE(results[2] == Bounds(320, 100, 180, 800));
  REQUIRE(results[3] == Bounds(600, 100, 100, 800));

  layout.setFlexReverseDirection(true);
  results = layout.flexPositions(layouts, { 0, 0, 800, 1000 }, 2.0f);
  REQUIRE(results.size() == 4);
  REQUIRE(results[0] == Bounds(680, 100, 20, 300));
  REQUIRE(results[1] == Bounds(580, 100, 0, 80));
  REQUIRE(results[2] == Bounds(300, 100, 180, 800));
  REQUIRE(results[3] == Bounds(100, 100, 100, 800));
}

TEST_CASE("Layout flex wrap constant size", "[ui]") {
  static constexpr int kNumChildren = 10;
  Layout layout;
  layout.setPadding(10);
  layout.setFlex(true);
  layout.setFlexWrap(true);
  layout.setFlexGap(10);

  Layout children[kNumChildren];
  for (int i = 0; i < kNumChildren; ++i) {
    children[i].setWidth(20 * (i + 1));
    children[i].setHeight(10 * (i + 1));
  }

  std::vector<const Layout*> layouts;
  for (int i = 0; i < kNumChildren; ++i)
    layouts.push_back(&children[i]);

  std::vector<Bounds> results = layout.flexPositions(layouts, { 0, 0, 100, 660 }, 2.0f);
  REQUIRE(results.size() == kNumChildren);
  REQUIRE(results[0] == Bounds(10, 10, 20, 10));
  REQUIRE(results[1] == Bounds(10, 30, 40, 20));
  REQUIRE(results[2] == Bounds(10, 60, 60, 30));
  REQUIRE(results[3] == Bounds(10, 100, 80, 40));
  REQUIRE(results[4] == Bounds(10, 150, 100, 50));
  REQUIRE(results[5] == Bounds(10, 210, 120, 60));
  REQUIRE(results[6] == Bounds(10, 280, 140, 70));
  REQUIRE(results[7] == Bounds(10, 360, 160, 80));
  REQUIRE(results[8] == Bounds(10, 450, 180, 90));
  REQUIRE(results[9] == Bounds(10, 550, 200, 100));

  results = layout.flexPositions(layouts, { 0, 0, 100, 659 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 10, 20, 10));
  REQUIRE(results[1] == Bounds(10, 30, 40, 20));
  REQUIRE(results[2] == Bounds(10, 60, 60, 30));
  REQUIRE(results[3] == Bounds(10, 100, 80, 40));
  REQUIRE(results[4] == Bounds(10, 150, 100, 50));
  REQUIRE(results[5] == Bounds(10, 210, 120, 60));
  REQUIRE(results[6] == Bounds(10, 280, 140, 70));
  REQUIRE(results[7] == Bounds(10, 360, 160, 80));
  REQUIRE(results[8] == Bounds(10, 450, 180, 90));
  REQUIRE(results[9] == Bounds(200, 10, 200, 100));

  results = layout.flexPositions(layouts, { 0, 0, 100, 279 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 10, 20, 10));
  REQUIRE(results[1] == Bounds(10, 30, 40, 20));
  REQUIRE(results[2] == Bounds(10, 60, 60, 30));
  REQUIRE(results[3] == Bounds(10, 100, 80, 40));
  REQUIRE(results[4] == Bounds(10, 150, 100, 50));
  REQUIRE(results[5] == Bounds(120, 10, 120, 60));
  REQUIRE(results[6] == Bounds(120, 80, 140, 70));
  REQUIRE(results[7] == Bounds(120, 160, 160, 80));
  REQUIRE(results[8] == Bounds(290, 10, 180, 90));
  REQUIRE(results[9] == Bounds(290, 110, 200, 100));

  layout.setFlexReverseDirection(true);
  results = layout.flexPositions(layouts, { 0, 0, 100, 279 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 259, 20, 10));
  REQUIRE(results[1] == Bounds(10, 229, 40, 20));
  REQUIRE(results[2] == Bounds(10, 189, 60, 30));
  REQUIRE(results[3] == Bounds(10, 139, 80, 40));
  REQUIRE(results[4] == Bounds(10, 79, 100, 50));
  REQUIRE(results[5] == Bounds(120, 209, 120, 60));
  REQUIRE(results[6] == Bounds(120, 129, 140, 70));
  REQUIRE(results[7] == Bounds(120, 39, 160, 80));
  REQUIRE(results[8] == Bounds(290, 179, 180, 90));
  REQUIRE(results[9] == Bounds(290, 69, 200, 100));

  layout.setFlexWrapReverse(true);
  results = layout.flexPositions(layouts, { 0, 0, 400, 279 }, 2.0f);
  REQUIRE(results[0] == Bounds(370, 259, 20, 10));
  REQUIRE(results[1] == Bounds(350, 229, 40, 20));
  REQUIRE(results[2] == Bounds(330, 189, 60, 30));
  REQUIRE(results[3] == Bounds(310, 139, 80, 40));
  REQUIRE(results[4] == Bounds(290, 79, 100, 50));
  REQUIRE(results[5] == Bounds(160, 209, 120, 60));
  REQUIRE(results[6] == Bounds(140, 129, 140, 70));
  REQUIRE(results[7] == Bounds(120, 39, 160, 80));
  REQUIRE(results[8] == Bounds(-70, 179, 180, 90));
  REQUIRE(results[9] == Bounds(-90, 69, 200, 100));
}

TEST_CASE("Layout flex wrap grow", "[ui]") {
  static constexpr int kNumChildren = 10;
  Layout layout;
  layout.setPadding(10);
  layout.setFlex(true);
  layout.setFlexWrap(true);
  layout.setFlexGap(10);

  Layout children[kNumChildren];
  for (int i = 0; i < kNumChildren; ++i) {
    children[i].setWidth(20 * (i + 1));
    children[i].setHeight(10 * (i + 1));
  }

  std::vector<const Layout*> layouts;
  for (int i = 0; i < kNumChildren; ++i)
    layouts.push_back(&children[i]);

  for (int i = 0; i < 6; ++i)
    children[i].setFlexGrow(1.0f);
  children[6].setWidth({});

  auto results = layout.flexPositions(layouts, { 0, 0, 100, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 10, 20, 20));
  REQUIRE(results[1] == Bounds(10, 40, 40, 30));
  REQUIRE(results[2] == Bounds(10, 80, 60, 40));
  REQUIRE(results[3] == Bounds(10, 130, 80, 50));
  REQUIRE(results[4] == Bounds(10, 190, 100, 60));
  REQUIRE(results[5] == Bounds(120, 10, 120, 70));
  REQUIRE(results[6] == Bounds(120, 90, 160, 70));
  REQUIRE(results[7] == Bounds(120, 170, 160, 80));
  REQUIRE(results[8] == Bounds(290, 10, 180, 90));
  REQUIRE(results[9] == Bounds(290, 110, 200, 100));

  layout.setFlexReverseDirection(true);
  results = layout.flexPositions(layouts, { 0, 0, 100, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 230, 20, 20));
  REQUIRE(results[1] == Bounds(10, 190, 40, 30));
  REQUIRE(results[2] == Bounds(10, 140, 60, 40));
  REQUIRE(results[3] == Bounds(10, 80, 80, 50));
  REQUIRE(results[4] == Bounds(10, 10, 100, 60));
  REQUIRE(results[5] == Bounds(120, 180, 120, 70));
  REQUIRE(results[6] == Bounds(120, 100, 160, 70));
  REQUIRE(results[7] == Bounds(120, 10, 160, 80));
  REQUIRE(results[8] == Bounds(290, 160, 180, 90));
  REQUIRE(results[9] == Bounds(290, 50, 200, 100));

  layout.setFlexWrapReverse(true);
  results = layout.flexPositions(layouts, { 0, 0, 400, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(370, 230, 20, 20));
  REQUIRE(results[1] == Bounds(350, 190, 40, 30));
  REQUIRE(results[2] == Bounds(330, 140, 60, 40));
  REQUIRE(results[3] == Bounds(310, 80, 80, 50));
  REQUIRE(results[4] == Bounds(290, 10, 100, 60));
  REQUIRE(results[5] == Bounds(160, 180, 120, 70));
  REQUIRE(results[6] == Bounds(120, 100, 160, 70));
  REQUIRE(results[7] == Bounds(120, 10, 160, 80));
  REQUIRE(results[8] == Bounds(-70, 160, 180, 90));
  REQUIRE(results[9] == Bounds(-90, 50, 200, 100));
}

TEST_CASE("Layout flex alignment", "[ui]") {
  static constexpr int kNumChildren = 10;
  Layout layout;
  layout.setPadding(10);
  layout.setFlex(true);
  layout.setFlexWrap(true);
  layout.setFlexGap(10);

  Layout children[kNumChildren];
  for (int i = 0; i < kNumChildren; ++i) {
    children[i].setWidth(20 * (i + 1));
    children[i].setHeight(10 * (i + 1));
  }

  std::vector<const Layout*> layouts;
  for (int i = 0; i < kNumChildren; ++i)
    layouts.push_back(&children[i]);

  for (int i = 0; i < 6; ++i)
    children[i].setFlexGrow(1.0f);
  children[6].setWidth({});

  layout.setFlexWrapAlignment(Layout::WrapAlignment::Start);
  auto results = layout.flexPositions(layouts, { 0, 0, 300, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 10, 20, 20));
  REQUIRE(results[1] == Bounds(10, 40, 40, 30));
  REQUIRE(results[2] == Bounds(10, 80, 60, 40));
  REQUIRE(results[3] == Bounds(10, 130, 80, 50));
  REQUIRE(results[4] == Bounds(10, 190, 100, 60));
  REQUIRE(results[5] == Bounds(120, 10, 120, 70));
  REQUIRE(results[6] == Bounds(120, 90, 160, 70));
  REQUIRE(results[7] == Bounds(120, 170, 160, 80));
  REQUIRE(results[8] == Bounds(290, 10, 180, 90));
  REQUIRE(results[9] == Bounds(290, 110, 200, 100));

  layout.setFlexWrapAlignment(Layout::WrapAlignment::Center);
  results = layout.flexPositions(layouts, { 0, 0, 300, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(-90, 10, 20, 20));
  REQUIRE(results[1] == Bounds(-90, 40, 40, 30));
  REQUIRE(results[2] == Bounds(-90, 80, 60, 40));
  REQUIRE(results[3] == Bounds(-90, 130, 80, 50));
  REQUIRE(results[4] == Bounds(-90, 190, 100, 60));
  REQUIRE(results[5] == Bounds(20, 10, 120, 70));
  REQUIRE(results[6] == Bounds(20, 90, 160, 70));
  REQUIRE(results[7] == Bounds(20, 170, 160, 80));
  REQUIRE(results[8] == Bounds(190, 10, 180, 90));
  REQUIRE(results[9] == Bounds(190, 110, 200, 100));

  layout.setFlexWrapAlignment(Layout::WrapAlignment::End);
  results = layout.flexPositions(layouts, { 0, 0, 300, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(-190, 10, 20, 20));
  REQUIRE(results[1] == Bounds(-190, 40, 40, 30));
  REQUIRE(results[2] == Bounds(-190, 80, 60, 40));
  REQUIRE(results[3] == Bounds(-190, 130, 80, 50));
  REQUIRE(results[4] == Bounds(-190, 190, 100, 60));
  REQUIRE(results[5] == Bounds(-80, 10, 120, 70));
  REQUIRE(results[6] == Bounds(-80, 90, 160, 70));
  REQUIRE(results[7] == Bounds(-80, 170, 160, 80));
  REQUIRE(results[8] == Bounds(90, 10, 180, 90));
  REQUIRE(results[9] == Bounds(90, 110, 200, 100));

  layout.setFlexWrapAlignment(Layout::WrapAlignment::SpaceBetween);
  results = layout.flexPositions(layouts, { 0, 0, 700, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 10, 20, 20));
  REQUIRE(results[1] == Bounds(10, 40, 40, 30));
  REQUIRE(results[2] == Bounds(10, 80, 60, 40));
  REQUIRE(results[3] == Bounds(10, 130, 80, 50));
  REQUIRE(results[4] == Bounds(10, 190, 100, 60));
  REQUIRE(results[5] == Bounds(220, 10, 120, 70));
  REQUIRE(results[6] == Bounds(220, 90, 160, 70));
  REQUIRE(results[7] == Bounds(220, 170, 160, 80));
  REQUIRE(results[8] == Bounds(490, 10, 180, 90));
  REQUIRE(results[9] == Bounds(490, 110, 200, 100));

  layout.setFlexWrapAlignment(Layout::WrapAlignment::SpaceAround);
  results = layout.flexPositions(layouts, { 0, 0, 800, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(60, 10, 20, 20));
  REQUIRE(results[1] == Bounds(60, 40, 40, 30));
  REQUIRE(results[2] == Bounds(60, 80, 60, 40));
  REQUIRE(results[3] == Bounds(60, 130, 80, 50));
  REQUIRE(results[4] == Bounds(60, 190, 100, 60));
  REQUIRE(results[5] == Bounds(270, 10, 120, 70));
  REQUIRE(results[6] == Bounds(270, 90, 160, 70));
  REQUIRE(results[7] == Bounds(270, 170, 160, 80));
  REQUIRE(results[8] == Bounds(540, 10, 180, 90));
  REQUIRE(results[9] == Bounds(540, 110, 200, 100));

  layout.setFlexWrapAlignment(Layout::WrapAlignment::SpaceEvenly);
  results = layout.flexPositions(layouts, { 0, 0, 900, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(110, 10, 20, 20));
  REQUIRE(results[1] == Bounds(110, 40, 40, 30));
  REQUIRE(results[2] == Bounds(110, 80, 60, 40));
  REQUIRE(results[3] == Bounds(110, 130, 80, 50));
  REQUIRE(results[4] == Bounds(110, 190, 100, 60));
  REQUIRE(results[5] == Bounds(320, 10, 120, 70));
  REQUIRE(results[6] == Bounds(320, 90, 160, 70));
  REQUIRE(results[7] == Bounds(320, 170, 160, 80));
  REQUIRE(results[8] == Bounds(590, 10, 180, 90));
  REQUIRE(results[9] == Bounds(590, 110, 200, 100));

  layout.setFlexWrapAlignment(Layout::WrapAlignment::Stretch);
  results = layout.flexPositions(layouts, { 0, 0, 800, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 10, 20, 20));
  REQUIRE(results[1] == Bounds(10, 40, 40, 30));
  REQUIRE(results[2] == Bounds(10, 80, 60, 40));
  REQUIRE(results[3] == Bounds(10, 130, 80, 50));
  REQUIRE(results[4] == Bounds(10, 190, 100, 60));
  REQUIRE(results[5] == Bounds(220, 10, 120, 70));
  REQUIRE(results[6] == Bounds(220, 90, 260, 70));
  REQUIRE(results[7] == Bounds(220, 170, 160, 80));
  REQUIRE(results[8] == Bounds(490, 10, 180, 90));
  REQUIRE(results[9] == Bounds(490, 110, 200, 100));

  layout.setFlexItemAlignment(Layout::ItemAlignment::Start);
  results = layout.flexPositions(layouts, { 0, 0, 800, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(10, 10, 20, 20));
  REQUIRE(results[1] == Bounds(10, 40, 40, 30));
  REQUIRE(results[2] == Bounds(10, 80, 60, 40));
  REQUIRE(results[3] == Bounds(10, 130, 80, 50));
  REQUIRE(results[4] == Bounds(10, 190, 100, 60));
  REQUIRE(results[5] == Bounds(220, 10, 120, 70));
  REQUIRE(results[6] == Bounds(220, 90, 0, 70));
  REQUIRE(results[7] == Bounds(220, 170, 160, 80));
  REQUIRE(results[8] == Bounds(490, 10, 180, 90));
  REQUIRE(results[9] == Bounds(490, 110, 200, 100));

  layout.setFlexItemAlignment(Layout::ItemAlignment::Center);
  results = layout.flexPositions(layouts, { 0, 0, 800, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(100, 10, 20, 20));
  REQUIRE(results[1] == Bounds(90, 40, 40, 30));
  REQUIRE(results[2] == Bounds(80, 80, 60, 40));
  REQUIRE(results[3] == Bounds(70, 130, 80, 50));
  REQUIRE(results[4] == Bounds(60, 190, 100, 60));
  REQUIRE(results[5] == Bounds(290, 10, 120, 70));
  REQUIRE(results[6] == Bounds(350, 90, 0, 70));
  REQUIRE(results[7] == Bounds(270, 170, 160, 80));
  REQUIRE(results[8] == Bounds(550, 10, 180, 90));
  REQUIRE(results[9] == Bounds(540, 110, 200, 100));

  layout.setFlexItemAlignment(Layout::ItemAlignment::End);
  results = layout.flexPositions(layouts, { 0, 0, 800, 260 }, 2.0f);
  REQUIRE(results[0] == Bounds(190, 10, 20, 20));
  REQUIRE(results[1] == Bounds(170, 40, 40, 30));
  REQUIRE(results[2] == Bounds(150, 80, 60, 40));
  REQUIRE(results[3] == Bounds(130, 130, 80, 50));
  REQUIRE(results[4] == Bounds(110, 190, 100, 60));
  REQUIRE(results[5] == Bounds(360, 10, 120, 70));
  REQUIRE(results[6] == Bounds(480, 90, 0, 70));
  REQUIRE(results[7] == Bounds(320, 170, 160, 80));
  REQUIRE(results[8] == Bounds(610, 10, 180, 90));
  REQUIRE(results[9] == Bounds(590, 110, 200, 100));
}