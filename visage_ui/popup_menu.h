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

#include "events.h"
#include "frame.h"
#include "scroll_bar.h"
#include "visage_graphics/font.h"

#include <climits>

namespace visage {
  class PopupMenu;
#if defined(__APPLE__) && defined(__MACH__)
  void setNativeMenuBar(const PopupMenu& menu);
#else
  inline void setNativeMenuBar(const PopupMenu& menu) { }
#endif

  class PopupMenu {
  public:
    static constexpr int kNotSet = INT_MIN;

    PopupMenu() = default;
    PopupMenu(const String& name, int id = -1, std::vector<PopupMenu> options = {}, bool is_break = false) :
        name_(name), id_(id), options_(std::move(options)), is_break_(is_break) { }

    void show(Frame* source, Point position = { kNotSet, kNotSet });
    void setAsNativeMenuBar() { setNativeMenuBar(*this); }

    void addOption(int option_id, const String& option_name, bool option_selected = false,
                   bool active = true) {
      options_.push_back({ option_name, option_id });
      options_.back().selected_ = option_selected;
      options_.back().active_ = active;
    }

    auto& onSelection() { return on_selection_; }
    auto& onCancel() { return on_cancel_; }

    const auto& onSelection() const { return on_selection_; }
    const auto& onCancel() const { return on_cancel_; }

    void addSubMenu(PopupMenu sub_menu) { options_.push_back(std::move(sub_menu)); }

    void addBreak() { options_.push_back({ "", -1, {}, true }); }

    const std::vector<PopupMenu>& options() const { return options_; }
    int size() const { return options_.size(); }

    int id() const { return id_; }
    const String& name() const { return name_; }
    bool isBreak() const { return is_break_; }
    bool hasOptions() const { return !options_.empty(); }

    void setActive(bool active) { active_ = active; }
    bool isActive() const { return active_; }

    void setSelected(bool selected) { selected_ = selected; }
    bool isSelected() const { return selected_; }

  private:
    CallbackList<void(int)> on_selection_;
    CallbackList<void()> on_cancel_;
    String name_;
    int id_ = -1;
    bool is_break_ = false;
    bool selected_ = false;
    bool active_ = true;
    std::vector<PopupMenu> options_;
  };

  class PopupList : public ScrollableFrame {
  public:
    class Listener {
    public:
      virtual ~Listener() = default;
      virtual void optionSelected(const PopupMenu& option, PopupList* list) = 0;
      virtual void subMenuSelected(const PopupMenu& option, int selected_y, PopupList* list) = 0;
      virtual void mouseMovedOnMenu(Point position, PopupList* list) = 0;
      virtual void mouseDraggedOnMenu(Point position, PopupList* list) = 0;
      virtual void mouseUpOutside(Point position, PopupList* list) = 0;
    };

    PopupList() = default;

    void setOptions(std::vector<PopupMenu> options) { options_ = std::move(options); }
    void setFont(const Font& font) { font_ = font.withDpiScale(dpiScale()); }

    float renderHeight() const;
    float renderWidth() const;

    int yForIndex(int index) const;
    int hoverY() { return yForIndex(hover_index_); }
    int hoverIndex() const { return hover_index_; }
    int numOptions() const { return options_.size(); }
    const PopupMenu& option(int index) const { return options_[index]; }
    void selectHoveredIndex();
    void setHoverFromPosition(Point position);
    void setNoHover() { hover_index_ = -1; }
    void selectFromPosition(Point position);

    void draw(Canvas& canvas) override;
    void resized() override;

    void enableMouseUp(bool enable) { enable_mouse_up_ = enable; }

    void mouseExit(const MouseEvent& e) override;
    void mouseDown(const MouseEvent& e) override;
    void mouseMove(const MouseEvent& e) override;
    void mouseDrag(const MouseEvent& e) override;
    void mouseUp(const MouseEvent& e) override;
    bool mouseWheel(const MouseEvent& e) override {
      bool result = ScrollableFrame::mouseWheel(e);
      if (!isVisible())
        return result;

      for (Listener* listener : listeners_)
        listener->mouseMovedOnMenu(e.relativeTo(this).position, this);
      return result;
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
    std::vector<PopupMenu> options_;
    float opacity_ = 0.0f;
    int hover_index_ = -1;
    int menu_open_index_ = -1;
    bool enable_mouse_up_ = false;
    Font font_;
  };

  class PopupMenuFrame : public Frame,
                         public PopupList::Listener,
                         public EventTimer {
  public:
    static constexpr int kMaxSubMenus = 4;
    static constexpr int kWaitForSelection = 20;
    static constexpr int kPauseMs = 400;

    PopupMenuFrame(PopupMenu menu);
    ~PopupMenuFrame() override;

    void draw(Canvas& canvas) override;

    void show(std::unique_ptr<PopupMenuFrame> self, Frame* source, Point point = {});
    void setFont(const Font& font) {
      font_ = font;
      setListFonts(font);
    }
    void setListFonts(const Font& font) {
      for (auto& list : lists_)
        list.setFont(font);
    }

    void exit();
    void hierarchyChanged() override;
    void focusChanged(bool is_focused, bool was_clicked) override;
    void visibilityChanged() override { opacity_animation_.target(isVisible(), true); }
    void timerCallback() override;

    void optionSelected(const PopupMenu& option, PopupList* list) override;
    void subMenuSelected(const PopupMenu& option, int selection_y, PopupList* list) override;
    void mouseMovedOnMenu(Point position, PopupList* list) override { moveHover(position, list); }
    void mouseDraggedOnMenu(Point position, PopupList* list) override { moveHover(position, list); }
    void mouseUpOutside(Point position, PopupList* list) override;
    void moveHover(Point position, const PopupList* list);
    float opacity() const { return opacity_animation_.value(); }

  private:
    PopupMenu menu_;
    Frame* parent_ = nullptr;
    bool done_ = false;
    Animation<float> opacity_animation_;
    PopupList lists_[kMaxSubMenus];
    int hover_index_ = -1;
    Font font_;
    PopupList* hover_list_ = nullptr;

    VISAGE_LEAK_CHECKER(PopupMenuFrame)
  };

  class ValueDisplay : public Frame {
  public:
    ValueDisplay() { setIgnoresMouseEvents(true, false); }

    void draw(Canvas& canvas) override;

    void showDisplay(const String& text, Bounds bounds, Font::Justification justification);
    void setFont(const Font& font) { font_ = font; }

  private:
    Font font_;
    String text_;

    VISAGE_LEAK_CHECKER(ValueDisplay)
  };
}