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

#include "popup_menu.h"

#include "embedded/fonts.h"
#include "visage_graphics/theme.h"

namespace visage {
  VISAGE_THEME_IMPLEMENT_COLOR(PopupMenu, PopupMenuBackground, 0xff262a2e);
  VISAGE_THEME_IMPLEMENT_COLOR(PopupMenu, PopupMenuBorder, 0xff606265);
  VISAGE_THEME_IMPLEMENT_COLOR(PopupMenu, PopupMenuText, 0xffeeeeee);
  VISAGE_THEME_IMPLEMENT_COLOR(PopupMenu, PopupMenuDisabledText, 0xff888888);
  VISAGE_THEME_IMPLEMENT_COLOR(PopupMenu, PopupMenuSelection, 0xffaa88ff);
  VISAGE_THEME_IMPLEMENT_COLOR(PopupMenu, PopupMenuSelectionText, 0xffffffff);
  VISAGE_THEME_IMPLEMENT_COLOR(PopupMenu, PopupMenuSeparator, 0xffeeeeee);

  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupOptionHeight, 22.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupMinWidth, 175.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupTextPadding, 9.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupFontSize, 14.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupSelectionPadding, 4.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupRounding, 8.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupSelectionRounding, 4.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupBorderWidth, 1.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupSeparatorHeight, 22.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupSeparatorThickness, 1.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(PopupMenu, PopupSubMenuArrowSize, 3.0f);

  void PopupMenu::show(Frame* source, Point position) {
    std::unique_ptr<PopupMenuFrame> frame = std::make_unique<PopupMenuFrame>(*this);
    PopupMenuFrame* frame_ptr = frame.get();
    frame_ptr->show(std::move(frame), source, position);
  }

  int PopupList::optionHeight(int index) const {
    if (options_[index].isBreak())
      return paletteValue(PopupMenu::PopupSeparatorHeight);
    return paletteValue(PopupMenu::PopupOptionHeight);
  }

  float PopupList::renderHeight() const {
    float render_height = 2.0f * paletteValue(PopupMenu::PopupSelectionPadding);
    for (int i = 0; i < options_.size(); ++i)
      render_height += optionHeight(i);
    return render_height;
  }

  float PopupList::renderWidth() const {
    float width = paletteValue(PopupMenu::PopupMinWidth);
    float x_padding = paletteValue(PopupMenu::PopupSelectionPadding) +
                      paletteValue(PopupMenu::PopupTextPadding);
    for (const PopupMenu& option : options_) {
      float string_width = font_.stringWidth(option.name().c_str(), option.name().size()) + 2 * x_padding;
      width = std::max(width, string_width);
    }

    return width;
  }

  int PopupList::yForIndex(int index) const {
    int y = paletteValue(PopupMenu::PopupSelectionPadding);
    for (int i = 0; i < index && i < options_.size(); ++i)
      y += optionHeight(i);
    return y;
  }

  void PopupList::selectHoveredIndex() {
    if (hover_index_ >= 0 && hover_index_ < options_.size()) {
      if (options_[hover_index_].hasOptions()) {
        for (Listener* listener : listeners_)
          listener->subMenuSelected(options_[hover_index_], yForIndex(hover_index_), this);
        menu_open_index_ = hover_index_;
      }
      else {
        for (Listener* listener : listeners_)
          listener->optionSelected(options_[hover_index_], this);
      }
    }
  }

  void PopupList::setHoverFromPosition(Point position) {
    int y = paletteValue(PopupMenu::PopupSelectionPadding);
    for (int i = 0; i < options_.size(); ++i) {
      int option_height = optionHeight(i);
      if (!options_[i].isBreak() && options_[i].enabled() && position.y >= y &&
          position.y < y + option_height) {
        hover_index_ = i;
        return;
      }

      y += option_height;
    }

    hover_index_ = -1;
  }

  void PopupList::selectFromPosition(Point position) {
    setHoverFromPosition(position + Point(0, yPosition()));
    selectHoveredIndex();
  }

  void PopupList::draw(Canvas& canvas) {
    float rounding = paletteValue(PopupMenu::PopupRounding);
    float border_width = paletteValue(PopupMenu::PopupBorderWidth);
    Brush background = canvas.color(PopupMenu::PopupMenuBackground).withMultipliedAlpha(opacity_);
    Brush border = canvas.color(PopupMenu::PopupMenuBorder).withMultipliedAlpha(opacity_);
    canvas.setColor(background);
    canvas.roundedRectangle(0, 0, width(), height(), rounding);

    if (border_width > 0.0f) {
      canvas.setColor(border);
      canvas.roundedRectangleBorder(0, 0, width(), height(), rounding, border_width);
    }

    int selection_padding = paletteValue(PopupMenu::PopupSelectionPadding);
    int x_padding = selection_padding + paletteValue(PopupMenu::PopupTextPadding);
    int y = selection_padding - yPosition();
    float selection_rounding = paletteValue(PopupMenu::PopupSelectionRounding);
    float separator_thickness = paletteValue(PopupMenu::PopupSeparatorThickness);
    int arrow_size = paletteValue(PopupMenu::PopupSubMenuArrowSize);
    Font font = font_.withSize(paletteValue(PopupMenu::PopupFontSize));

    Brush text = canvas.color(PopupMenu::PopupMenuText).withMultipliedAlpha(opacity_);
    Brush disabled_text = canvas.color(PopupMenu::PopupMenuDisabledText).withMultipliedAlpha(opacity_);
    Brush selected_text = canvas.color(PopupMenu::PopupMenuSelectionText).withMultipliedAlpha(opacity_);
    Brush separator = canvas.color(PopupMenu::PopupMenuSeparator).withMultipliedAlpha(opacity_);
    for (int i = 0; i < options_.size(); ++i) {
      int option_height = optionHeight(i);
      if (y + option_height > 0 && y < height()) {
        if (options_[i].isBreak()) {
          int separator_y = y + option_height / 2 - static_cast<int>(separator_thickness / 2.0f);
          canvas.setColor(separator);
          canvas.rectangle(x_padding, separator_y, width() - 2 * x_padding, separator_thickness);
        }
        else {
          if (i == hover_index_) {
            Brush selected = canvas.color(PopupMenu::PopupMenuSelection).withMultipliedAlpha(opacity_);
            canvas.setColor(selected);
            canvas.roundedRectangle(selection_padding, y, width() - 2 * selection_padding,
                                    option_height, selection_rounding);
            canvas.setColor(selected_text);
          }
          else if (options_[i].enabled())
            canvas.setColor(text);
          else
            canvas.setColor(disabled_text);

          canvas.text(options_[i].name(), font, Font::kLeft, x_padding, y, width(), option_height);

          if (options_[i].hasOptions()) {
            int arrow_x = width() - x_padding - arrow_size;
            int arrow_y = y + option_height / 2 - arrow_size;
            canvas.triangleRight(arrow_x, arrow_y, arrow_size);
          }
        }
      }
      y += option_height;
    }
  }

  void PopupList::resized() {
    ScrollableFrame::resized();
    setScrollableHeight(renderHeight(), height());
  }

  void PopupList::mouseDown(const MouseEvent& e) {
    Point position = e.relativeTo(this).position;
    if (!isVisible() || !localBounds().contains(position))
      return;

    setHoverFromPosition(e.relativeTo(this).position + Point(0, yPosition()));

    if (hover_index_ < options_.size() && hover_index_ >= 0 && options_[hover_index_].hasOptions())
      selectHoveredIndex();

    redraw();
  }

  void PopupList::mouseExit(const MouseEvent& e) {
    if (!isVisible())
      return;

    hover_index_ = menu_open_index_;
    for (Listener* listener : listeners_)
      listener->mouseMovedOnMenu(e.relativeTo(this).position, this);

    redraw();
  }

  void PopupList::mouseMove(const MouseEvent& e) {
    if (!isVisible())
      return;

    for (Listener* listener : listeners_)
      listener->mouseMovedOnMenu(e.relativeTo(this).position, this);

    redraw();
  }

  void PopupList::mouseDrag(const MouseEvent& e) {
    if (!isVisible())
      return;

    for (Listener* listener : listeners_)
      listener->mouseDraggedOnMenu(e.relativeTo(this).position, this);

    redraw();
  }

  void PopupList::mouseUp(const MouseEvent& e) {
    if (!isVisible() || e.isCancelled())
      return;

    Point position = e.relativeTo(this).position;
    if (localBounds().contains(position) == 0) {
      for (Listener* listener : listeners_)
        listener->mouseUpOutside(position, this);
      return;
    }

    if (enable_mouse_up_)
      selectFromPosition(position);
    enable_mouse_up_ = true;

    redraw();
  }

  PopupMenuFrame::PopupMenuFrame(PopupMenu menu) :
      menu_(std::move(menu)),
      font_(menu_.font().packedFont() ? menu_.font() : Font(10, fonts::Lato_Regular_ttf)) {
    opacity_animation_.setTargetValue(1.0f);
    setAcceptsKeystrokes(true);
    setIgnoresMouseEvents(true, true);
    popupList(0);
  }

  PopupMenuFrame::~PopupMenuFrame() = default;

  PopupList& PopupMenuFrame::popupList(int index) {
    while (lists_.size() <= index) {
      auto list = std::make_unique<PopupList>();
      addChild(list.get(), false);
      list->addListener(this);
      list->setFont(font_);
      list->setOpacity(opacity_animation_.value());
      list->enableMouseUp(mouse_up_enabled_);
      lists_.push_back(std::move(list));
    }
    return *lists_[index];
  }

  Frame* PopupMenuFrame::keyboardFocusedFrame() const {
    FrameEventHandler* handler = eventHandler();
    return handler && handler->keyboard_focused_frame ? handler->keyboard_focused_frame() : nullptr;
  }

  int PopupMenuFrame::listIndex(const PopupList* list) const {
    for (int i = 0; i < lists_.size(); ++i) {
      if (lists_[i].get() == list)
        return i;
    }
    return 0;
  }

  void PopupMenuFrame::draw(Canvas& canvas) {
    float opacity = opacity_animation_.update();
    for (auto& list : lists_)
      list->setOpacity(opacity);

    if (opacity_animation_.isAnimating())
      redraw();
    else if (top_parent_ && !opacity_animation_.isTargeting())
      exit();
  }

  void PopupMenuFrame::show(std::unique_ptr<PopupMenuFrame> self, Frame* source, Point point) {
    top_parent_ = source->topParentFrame();
    top_parent_->addChild(std::move(self));
    if (source->palette())
      setPalette(source->palette());

    setOnTop(true);
    setBounds(top_parent_->bounds());

    for (int i = 1; i < lists_.size(); ++i)
      lists_[i]->setVisible(false);

    font_ = font_.withSize(paletteValue(PopupMenu::PopupFontSize));
    setListFonts(font_);

    PopupList& main_list = popupList(0);
    main_list.setOptions(menu_.options());
    int h = std::min(height(), main_list.renderHeight());
    int w = main_list.renderWidth();

    Bounds window_bounds = top_parent_->relativeBounds(source);
    int x = point.x == PopupMenu::kNotSet ? window_bounds.x() : window_bounds.x() + point.x;
    int y = point.y == PopupMenu::kNotSet ? window_bounds.bottom() : window_bounds.y() + point.y;
    int bottom = y + h;
    int right = x + w;
    if (bottom > height()) {
      int top = point.y == PopupMenu::kNotSet ? window_bounds.y() : window_bounds.y() + point.y;
      y = std::max(0, top - h);
    }
    if (right > width())
      x = std::max(0, x - w);

    for (auto& list : lists_) {
      list->resetOpenMenu();
      list->setNoHover();
    }

    main_list.setBounds(x, y, w, h);
    main_list.setVisible(true);
    main_list.redraw();
    opacity_animation_.target(true, true);

    mouse_up_enabled_ = false;
    for (auto& list : lists_)
      list->enableMouseUp(false);

    stopTimer();
    startTimer(takeMouseDrag() ? kPauseMs : kWaitForSelection);

    previous_focus_ = keyboardFocusedFrame();
    if (previous_focus_)
      previous_focus_alive_ = previous_focus_->aliveToken();
    requestKeyboardFocus();
    redraw();
  }

  void PopupMenuFrame::mouseDrag(const MouseEvent& e) {
    PopupList& main_list = popupList(0);
    moveHover(e.position - main_list.topLeft(), &main_list);
    dragged_onto_menu_ = dragged_onto_menu_ || hover_list_ != nullptr;
  }

  void PopupMenuFrame::mouseUp(const MouseEvent& e) {
    bool over_menu = false;
    for (auto& list : lists_)
      over_menu = over_menu || (list->isVisible() && list->bounds().contains(e.position));

    if (!e.isCancelled() && dragged_onto_menu_ && (mouse_up_enabled_ || !over_menu)) {
      PopupList& main_list = popupList(0);
      mouseUpOutside(e.position - main_list.topLeft(), &main_list);
    }

    dragged_onto_menu_ = false;
    mouse_up_enabled_ = true;
    for (auto& list : lists_)
      list->enableMouseUp(true);
  }

  void PopupMenuFrame::exit() {
    done_ = true;
    startTimer(1);
  }

  void PopupMenuFrame::hierarchyChanged() {
    if (parent() == nullptr)
      startTimer(1);
  }

  void PopupMenuFrame::focusChanged(bool is_focused, bool was_clicked) {
    if (!is_focused && isVisible()) {
      if (!done_ && opacity_animation_.isTargeting())
        menu_.onCancel().callback();

      startTimer(1);
      opacity_animation_.target(false);
    }

    redraw();
  }

  void PopupMenuFrame::timerCallback() {
    if (top_parent_ && done_) {
      if (keyboardFocusedFrame() == this && !previous_focus_alive_.expired() && previous_focus_->isDrawing())
        previous_focus_->requestKeyboardFocus();
      top_parent_->removeChild(this);
      return;
    }

    redraw();
    stopTimer();

    mouse_up_enabled_ = true;
    for (auto& list : lists_)
      list->enableMouseUp(true);

    if (hover_list_ && hover_index_ >= 0 && hover_index_ < hover_list_->numOptions()) {
      const PopupMenu& option = hover_list_->option(hover_index_);
      if (option.hasOptions()) {
        subMenuSelected(option, hover_list_->hoverY(), hover_list_);
        return;
      }
    }

    int last_open_menu = lists_.size() - 1;
    for (; last_open_menu > 0 && hover_list_ != lists_[last_open_menu].get(); --last_open_menu) {
      lists_[last_open_menu]->setVisible(false);
      lists_[last_open_menu]->resetOpenMenu();
    }

    lists_[last_open_menu]->resetOpenMenu();
    if (hover_index_ < 0)
      lists_[last_open_menu]->setNoHover();
  }

  void PopupMenuFrame::optionSelected(const PopupMenu& option, PopupList* list) {
    if (done_)
      return;

    if (isVisible()) {
      option.onSelection().callback(option.id());
      menu_.onSelection().callback(option.id());
    }
    else
      menu_.onCancel().callback();

    exit();
  }

  void PopupMenuFrame::subMenuSelected(const PopupMenu& option, int selection_y, PopupList* list) {
    int source_index = listIndex(list);
    PopupList& source = *lists_[source_index];
    PopupList& sub_list = popupList(source_index + 1);

    bool reopening = source.openMenuIndex() == source.hoverIndex() && sub_list.isVisible();
    source.setOpenMenu(source.hoverIndex());
    sub_list.setOptions(option.options());
    if (option.font().packedFont())
      sub_list.setFont(option.font().withSize(paletteValue(PopupMenu::PopupFontSize)));
    else
      sub_list.setFont(source.font());
    int h = std::min(height(), sub_list.renderHeight());
    int w = sub_list.renderWidth();
    int y = list->y() + selection_y - list->yPosition();
    int bottom = y + h;
    int x = source.right();
    int right = x + w;
    if (bottom > height())
      y = height() - h;
    y = std::max(0, y);
    if (right > width())
      x = source.x() - w;

    sub_list.setBounds(x, y, w, h);
    if (!reopening) {
      sub_list.setYPosition(0);
      sub_list.setScrollableHeight(sub_list.renderHeight(), h);
    }
    sub_list.setNoHover();
    sub_list.setVisible(true);
  }

  void PopupMenuFrame::moveHover(Point position, const PopupList* list) {
    PopupList* last_hover_list = hover_list_;
    int last_hover_index = hover_index_;
    position += list->topLeft();
    hover_list_ = nullptr;
    hover_index_ = -1;
    for (auto& entry : lists_) {
      PopupList& sub_list = *entry;
      if (sub_list.isVisible() && sub_list.bounds().contains(position)) {
        sub_list.setHoverFromPosition(position - sub_list.topLeft() + Point(0, sub_list.yPosition()));
        sub_list.redraw();

        hover_list_ = &sub_list;
        hover_index_ = hover_list_->hoverIndex();
      }
    }
    if (hover_list_ != last_hover_list || hover_index_ != last_hover_index) {
      stopTimer();
      startTimer(kPauseMs);
    }
  }

  void PopupMenuFrame::mouseUpOutside(Point position, PopupList* list) {
    if (done_)
      return;

    position += list->topLeft();
    for (auto& entry : lists_) {
      PopupList& sub_list = *entry;
      if (sub_list.isVisible() && sub_list.bounds().contains(position)) {
        sub_list.selectFromPosition(position - sub_list.topLeft());
        return;
      }
    }

    if (isRunning())
      return;

    menu_.onCancel().callback();
    exit();
  }

  void ValueDisplay::showDisplay(const String& text, Bounds bounds, Font::Justification justification) {
    setVisible(true);
    text_ = text;

    Font font = font_.withSize(paletteValue(PopupMenu::PopupFontSize));
    int x_padding = paletteValue(PopupMenu::PopupSelectionPadding) +
                    paletteValue(PopupMenu::PopupTextPadding);
    int width = font.stringWidth(text.c_str(), text.length()) + 2 * x_padding;
    int height = paletteValue(PopupMenu::PopupOptionHeight);
    int x = bounds.xCenter() - width / 2;
    int y = bounds.yCenter() - height / 2;
    if (justification & Font::kLeft)
      x = bounds.x() - width;
    else if (justification & Font::kRight)
      x = bounds.right();
    if (justification & Font::kBottom)
      y = bounds.bottom();
    else if (justification & Font::kTop)
      y = bounds.y() - height;

    setBounds(x, y, width, height);
    redraw();
  }

  void ValueDisplay::draw(Canvas& canvas) {
    Font font = font_.withSize(paletteValue(PopupMenu::PopupFontSize));
    float rounding = paletteValue(PopupMenu::PopupRounding);
    float border_width = paletteValue(PopupMenu::PopupBorderWidth);
    canvas.setColor(PopupMenu::PopupMenuBackground);
    canvas.roundedRectangle(0, 0, width(), height(), rounding);

    if (border_width > 0.0f) {
      canvas.setColor(PopupMenu::PopupMenuBorder);
      canvas.roundedRectangleBorder(0, 0, width(), height(), rounding, border_width);
    }

    canvas.setColor(PopupMenu::PopupMenuText);
    canvas.text(text_, font, Font::kCenter, 0, 0, width(), height());
  }
}
