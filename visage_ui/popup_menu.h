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

#include "events.h"
#include "frame.h"
#include "scroll_bar.h"
#include "visage_graphics/font.h"

namespace visage {
  class PopupList : public ScrollableComponent {
  public:
    class Listener {
    public:
      virtual ~Listener() = default;
      virtual void optionSelected(const PopupOptions& option, PopupList* list) = 0;
      virtual void subMenuSelected(const PopupOptions& option, int selected_y, PopupList* list) = 0;
      virtual void mouseMovedOnMenu(Point position, PopupList* list) = 0;
      virtual void mouseDraggedOnMenu(Point position, PopupList* list) = 0;
      virtual void mouseUpOutside(Point position, PopupList* list) = 0;
    };

    PopupList() = default;

    void setOptions(std::vector<PopupOptions> options) { options_ = std::move(options); }
    void setFont(const Font& font) { font_ = font; }

    int renderHeight();
    int renderWidth();

    int yForIndex(int index);
    int hoverY() { return yForIndex(hover_index_); }
    int hoverIndex() const { return hover_index_; }
    int numOptions() const { return options_.size(); }
    const PopupOptions& option(int index) const { return options_[index]; }
    void selectHoveredIndex();
    void setHoverFromPosition(Point position);
    void setNoHover() { hover_index_ = -1; }
    void selectFromPosition(Point position);

    void draw(Canvas& canvas) override;
    void resized() override;

    void enableMouseUp(bool enable) { enable_mouse_up_ = enable; }

    void onMouseExit(const MouseEvent& e) override;
    void onMouseDown(const MouseEvent& e) override;
    void onMouseMove(const MouseEvent& e) override;
    void onMouseDrag(const MouseEvent& e) override;
    void onMouseUp(const MouseEvent& e) override;
    void onMouseWheel(const MouseEvent& e) override {
      ScrollableComponent::onMouseWheel(e);
      if (isVisible()) {
        for (Listener* listener : listeners_)
          listener->mouseMovedOnMenu(e.relativeTo(this).position, this);
      }
    }
    void addListener(Listener* listener) { listeners_.push_back(listener); }
    void resetOpenMenu() { menu_open_index_ = -1; }
    void setOpenMenu(int index) { menu_open_index_ = index; }
    void setOpacity(float opacity) {
      opacity_ = opacity;
      redraw();
    }

  private:
    std::vector<Listener*> listeners_;
    std::vector<PopupOptions> options_;
    float opacity_ = 0.0f;
    int hover_index_ = -1;
    int menu_open_index_ = -1;
    bool enable_mouse_up_ = false;
    Font font_;
  };

  class PopupMenu : public Frame,
                    public PopupList::Listener,
                    public EventTimer {
  public:
    static constexpr int kMaxSubMenus = 4;
    static constexpr int kWaitForSelection = 20;
    static constexpr int kPauseMs = 400;

    PopupMenu() {
      opacity_animation_.setTargetValue(1.0f);
      setAcceptsKeystrokes(true);
      setIgnoresMouseEvents(true, true);

      for (auto& list : lists_) {
        addChild(&list);
        list.setVisible(false);
        list.addListener(this);
      }
    }

    void draw(Canvas& canvas) override;

    void showMenu(const PopupOptions& options, Bounds bounds, std::function<void(int)> callback,
                  std::function<void()> cancel);
    void setFont(const Font& font) {
      font_ = font;
      setListFonts(font);
    }
    void setListFonts(const Font& font) {
      for (auto& list : lists_)
        list.setFont(font);
    }

    void onFocusChange(bool is_focused, bool was_clicked) override;
    void onVisibilityChange() override { opacity_animation_.target(isVisible(), true); }
    void timerCallback() override;

    void optionSelected(const PopupOptions& option, PopupList* list) override;
    void subMenuSelected(const PopupOptions& option, int selection_y, PopupList* list) override;
    void mouseMovedOnMenu(Point position, PopupList* list) override { moveHover(position, list); }
    void mouseDraggedOnMenu(Point position, PopupList* list) override { moveHover(position, list); }
    void mouseUpOutside(Point position, PopupList* list) override;
    void moveHover(Point position, const PopupList* list);
    float opacity() const { return opacity_animation_.value(); }

  private:
    Animation<float> opacity_animation_;
    std::function<void(int)> callback_;
    std::function<void()> cancel_;
    PopupList lists_[kMaxSubMenus];
    int hover_index_ = -1;
    Font font_;
    Bounds last_bounds_;

    PopupList* hover_list_ = nullptr;

    VISAGE_LEAK_CHECKER(PopupMenu)
  };

  class ValueDisplay : public Frame {
  public:
    ValueDisplay() { setIgnoresMouseEvents(true, false); }

    void draw(Canvas& canvas) override;

    void showDisplay(const String& text, Bounds bounds, Font::Justification justification);
    void setFont(const Font* font) { font_ = font; }

  private:
    const Font* font_ = nullptr;
    String text_;

    VISAGE_LEAK_CHECKER(ValueDisplay)
  };

  class PopupDisplayer {
  public:
    virtual ~PopupDisplayer() = default;
    virtual bool isPopupVisible() = 0;
    virtual void showPopup(const PopupOptions& options, Frame* frame, Bounds bounds,
                           std::function<void(int)> callback, std::function<void()> cancel) = 0;
    virtual void showValueDisplay(const std::string& text, Frame* frame, Bounds bounds,
                                  Font::Justification justification, bool primary) = 0;
    virtual void hideValueDisplay(bool primary) = 0;
  };
}