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

#include "embedded/example_icons.h"
#include "test_helpers.h"

#include <atomic>
#include <cmath>
#include <thread>
#include <visage_utils/file_system.h>

using namespace visage::dimension;

VISAGE_THEME_COLOR(ManualSwatchPrimary, 0xff5ab0ff);
VISAGE_THEME_COLOR(ManualSwatchSecondary, 0xffe5534b);
VISAGE_THEME_COLOR(ManualSwatchTertiary, 0xff4caf50);
VISAGE_THEME_COLOR(ManualSwatchBackground, 0xff262b32);
VISAGE_THEME_VALUE(ManualSwatchRounding, 12.0f);
VISAGE_THEME_VALUE(ManualSwatchBorder, 2.0f);

namespace manual {
  namespace {
    class ActionFrame : public visage::Frame {
    public:
      ActionFrame(std::string text, unsigned int color) : text_(std::move(text)), color_(color) { }

      auto& onDown() { return on_down_; }
      auto& onUp() { return on_up_; }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(pressed_ ? 0xffffffff : color_);
        canvas.roundedRectangle(0, 0, width(), height(), 6);
        canvas.setColor(pressed_ ? 0xff000000 : 0xffffffff);
        drawLines(canvas, text_, font(13), visage::Font::kCenter, 4, 0, width() - 8, height());
      }

      void mouseDown(const visage::MouseEvent& e) override {
        pressed_ = true;
        redraw();
        on_down_.callback();
      }

      void mouseUp(const visage::MouseEvent& e) override {
        pressed_ = false;
        redraw();
        on_up_.callback();
      }

    private:
      std::string text_;
      unsigned int color_ = 0;
      bool pressed_ = false;
      visage::CallbackList<void()> on_down_;
      visage::CallbackList<void()> on_up_;
    };

    class FrameLifecyclePage : public visage::Frame {
    public:
      FrameLifecyclePage() :
          self_box_("Frames that remove or delete themselves"), left_("Container A"),
          right_("Container B"), focus_box_("Focused editor inside a box that gets hidden"),
          hidden_parent_("Hidden parent"), visible_parent_("Visible parent"),
          alpha_box_("Group alpha transparency"), mover_("Mover", 0xff8a5a2f),
          orphan_("Orphan", 0xff5a8a2f) {
        addChild(&self_box_);
        addChild(&left_);
        addChild(&right_);
        addChild(&focus_box_);
        addChild(&hidden_parent_);
        addChild(&visible_parent_);
        addChild(&alpha_box_);
        addChild(&log_);

        createSelfFrames();

        recreate_ = makeButton("Recreate", [this] { createSelfFrames(); });
        self_box_.addChild(recreate_.get());

        left_.addChild(&mover_);
        mover_.onDown() = [this] {
          Box* target = mover_.parent() == &left_ ? &right_ : &left_;
          target->addChild(&mover_);
          layoutMover();
          log_.add("Mover reparented. A has " + std::to_string(left_.children().size()) +
                   " child(ren), B has " + std::to_string(right_.children().size()));
        };
        mover_.onHierarchyChange() += [this] { log_.add("Mover: onHierarchyChange"); };

        editor_.setFont(font(15));
        editor_.setDefaultText("Click, then press the button");
        editor_.onFocusChange() += [this](bool focused, bool clicked) {
          log_.add(std::string("Editor focus ") + (focused ? "gained" : "lost"));
        };
        focus_box_.addChild(&editor_);
        hide_focus_ = makeButton("Hide this box in 2 seconds", [this] {
          hide_timer_.startTimer(2000);
          log_.add("Hiding the editor box in 2 seconds. Keep the editor focused.");
        });
        focus_box_.addChild(hide_focus_.get());
        show_focus_ = makeButton("Show the editor box again", [this] { focus_box_.setVisible(true); });
        addChild(show_focus_.get());
        hide_timer_.onTimerCallback() = [this] {
          hide_timer_.stopTimer();
          focus_box_.setVisible(false);
          log_.add("Editor box hidden. Typing now should go nowhere.");
        };

        hidden_parent_.setVisible(false);
        hidden_parent_.addChild(&orphan_);
        orphan_.onDown() = [this] { moveOrphan(); };
        move_orphan_ = makeButton("Move Orphan between hidden and visible parent",
                                  [this] { moveOrphan(); });
        addChild(move_orphan_.get());
        orphan_.onHierarchyChange() += [this] { log_.add("Orphan: onHierarchyChange"); };

        for (int i = 0; i < 3; ++i) {
          unsigned int colors[] = { 0xffff4040, 0xff40ff40, 0xff4060ff };
          unsigned int color = colors[i];
          auto circle = std::make_unique<visage::Frame>();
          visage::Frame* raw = circle.get();
          circle->onDraw() = [raw, color](visage::Canvas& canvas) {
            canvas.setColor(color);
            canvas.circle(0, 0, raw->width());
          };
          circle->setIgnoresMouseEvents(true, false);
          alpha_group_.addChild(circle.get());
          circles_.push_back(std::move(circle));
        }
        alpha_group_.setIgnoresMouseEvents(true, true);
        alpha_box_.addChild(&alpha_group_);
        alpha_slider_.onChange() += [this](float value) { alpha_group_.setAlphaTransparency(value); };
        alpha_group_.setAlphaTransparency(alpha_slider_.value());
        alpha_box_.addChild(&alpha_slider_);
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        auto columns = evenColumns(bounds, 2);
        auto left = splitRows(columns[0], { 1.3f, 1.0f, 1.0f });
        self_box_.setBounds(left[0]);
        layoutSelfFrames();
        auto movers = evenColumns(left[1], 2);
        left_.setBounds(movers[0]);
        right_.setBounds(movers[1]);
        layoutMover();
        visage::Bounds focus_area = left[2];
        show_focus_->setBounds(takeBottom(focus_area, 32, 4));
        focus_box_.setBounds(focus_area);
        visage::Bounds focus_content = focus_box_.contentBounds();
        hide_focus_->setBounds(takeBottom(focus_content, 32, 4));
        editor_.setBounds(focus_content);

        auto right = splitRows(columns[1], { 1.0f, 1.2f, 1.5f });
        visage::Bounds orphan_area = right[0];
        move_orphan_->setBounds(takeBottom(orphan_area, 32, 4));
        auto parents = evenColumns(orphan_area, 2);
        hidden_parent_.setBounds(parents[0]);
        visible_parent_.setBounds(parents[1]);
        orphan_.setBounds(hidden_parent_.contentBounds());

        alpha_box_.setBounds(right[1]);
        visage::Bounds alpha_content = alpha_box_.contentBounds();
        alpha_slider_.setBounds(takeBottom(alpha_content, 34));
        float size = std::min(alpha_content.height(), alpha_content.width());
        alpha_group_.setBounds(alpha_content.x() + (alpha_content.width() - size) * 0.5f,
                               alpha_content.y(), size, size);
        float d = size * 0.6f;
        circles_[0]->setBounds(0, 0, d, d);
        circles_[1]->setBounds(size - d, 0, d, d);
        circles_[2]->setBounds((size - d) * 0.5f, size - d, d, d);
        log_.setBounds(right[2]);
      }

    private:
      void createSelfFrames() {
        if (!delete_on_up_) {
          delete_on_up_ =
              std::make_unique<ActionFrame>("Delete me on mouse up (also try releasing outside)", 0xff2f5d8a);
          delete_on_up_->onUp() = [this] {
            log_.add("Deleting a frame from inside its own mouseUp");
            delete_on_up_ = nullptr;
          };
          self_box_.addChild(delete_on_up_.get());
        }
        if (!remove_on_down_ || remove_on_down_->parent() == nullptr) {
          if (!remove_on_down_) {
            remove_on_down_ =
                std::make_unique<ActionFrame>("Remove me from my parent on mouse down", 0xff2f8a5d);
            remove_on_down_->onDown() = [this] {
              log_.add("Removing a frame from its parent inside its own mouseDown");
              self_box_.removeChild(remove_on_down_.get());
            };
          }
          self_box_.addChild(remove_on_down_.get());
        }
        if (!doomed_parent_) {
          doomed_parent_ = std::make_unique<Box>("Doomed parent");
          auto child = std::make_unique<ActionFrame>("Delete my parent on mouse down", 0xff8a2f5d);
          child->onDown() = [this] {
            log_.add("Deleting the parent of the frame handling mouseDown");
            doomed_parent_ = nullptr;
          };
          doomed_parent_->addChild(std::move(child));
          self_box_.addChild(doomed_parent_.get());
        }
        layoutSelfFrames();
      }

      void layoutSelfFrames() {
        visage::Bounds content = self_box_.contentBounds();
        if (recreate_)
          recreate_->setBounds(takeBottom(content, 32, 6));
        auto cells = evenRows(content, 3, 6);
        if (delete_on_up_)
          delete_on_up_->setBounds(cells[0]);
        if (remove_on_down_)
          remove_on_down_->setBounds(cells[1]);
        if (doomed_parent_) {
          doomed_parent_->setBounds(cells[2]);
          if (!doomed_parent_->children().empty())
            doomed_parent_->children()[0]->setBounds(doomed_parent_->contentBounds());
        }
      }

      void layoutMover() {
        Box* parent = mover_.parent() == &right_ ? &right_ : &left_;
        visage::Bounds content = parent->contentBounds();
        mover_.setBounds(content.x(), content.y(), content.width(), std::min(60.0f, content.height()));
      }

      void moveOrphan() {
        Box* target = orphan_.parent() == &hidden_parent_ ? &visible_parent_ : &hidden_parent_;
        target->addChild(&orphan_);
        orphan_.setBounds(target->contentBounds());
        log_.add(std::string("Orphan moved to the ") +
                 (target == &visible_parent_ ? "visible" : "hidden") + " parent");
      }

      Box self_box_;
      Box left_;
      Box right_;
      Box focus_box_;
      Box hidden_parent_;
      Box visible_parent_;
      Box alpha_box_;
      LogView log_;
      ActionFrame mover_;
      ActionFrame orphan_;
      visage::TextEditor editor_;
      visage::Frame alpha_group_;
      Slider alpha_slider_ { "Alpha", 0.0f, 1.0f, 0.5f, 2 };
      std::vector<std::unique_ptr<visage::Frame>> circles_;
      std::unique_ptr<ActionFrame> delete_on_up_;
      std::unique_ptr<ActionFrame> remove_on_down_;
      std::unique_ptr<Box> doomed_parent_;
      std::unique_ptr<visage::UiButton> recreate_;
      std::unique_ptr<visage::UiButton> hide_focus_;
      std::unique_ptr<visage::UiButton> show_focus_;
      std::unique_ptr<visage::UiButton> move_orphan_;
      visage::EventTimer hide_timer_;
    };

    class Row : public visage::Frame {
    public:
      explicit Row(int index) : index_(index) { }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(hover_ ? 0xff3c4c66 : ((index_ % 2) ? kPanel : kPanelLight));
        canvas.fill(0, 0, width(), height());
        canvas.setColor(kText);
        std::string text = "Row " + std::to_string(index_);
        if (clicks_)
          text += "   clicked " + std::to_string(clicks_) + "x";
        canvas.text(text, font(14), visage::Font::kLeft, 12, 0, width() - 24, height());
      }

      void mouseEnter(const visage::MouseEvent& e) override {
        hover_ = true;
        redraw();
      }
      void mouseExit(const visage::MouseEvent& e) override {
        hover_ = false;
        redraw();
      }
      void mouseDown(const visage::MouseEvent& e) override {
        ++clicks_;
        redraw();
      }

    private:
      int index_ = 0;
      int clicks_ = 0;
      bool hover_ = false;
    };

    class RowList : public visage::ScrollableFrame {
    public:
      static constexpr float kRowHeight = 28.0f;

      explicit RowList(int rows) { addRows(rows); }

      void addRows(int count) {
        for (int i = 0; i < count; ++i) {
          rows_.push_back(std::make_unique<Row>(rows_.size()));
          addScrolledChild(*rows_.back());
        }
        if (!hidden_) {
          hidden_ = std::make_unique<Label>("You should never see this row", 14, kFail);
          addScrolledChild(*hidden_, false);
        }
        resized();
      }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(0xff111418);
        canvas.fill(0, 0, width(), height());
      }

      void resized() override {
        ScrollableFrame::resized();
        for (int i = 0; i < rows_.size(); ++i)
          rows_[i]->setBounds(0, i * kRowHeight, width() - 12, kRowHeight);
        if (hidden_)
          hidden_->setBounds(0, 3 * kRowHeight, width() - 12, kRowHeight);
        setScrollableHeight(rows_.size() * kRowHeight);
      }

    private:
      std::vector<std::unique_ptr<Row>> rows_;
      std::unique_ptr<Label> hidden_;
    };

    class ScrollingPage : public visage::Frame {
    public:
      ScrollingPage() {
        createList();
        add_ = makeButton("Add 50 rows", [this] {
          if (list_)
            list_->addRows(50);
        });
        delete_ = makeButton("Delete list in 1 second", [this] {
          delete_timer_.startTimer(1000);
          status_.setText("Flick-scroll the list now. It gets deleted in 1 second.");
        });
        recreate_ = makeButton("Recreate list", [this] { createList(); });
        jump_ = makeButton("setYPosition(1234.6)", [this] {
          if (list_)
            list_->setYPosition(1234.6f);
        });
        for (auto* button : { add_.get(), delete_.get(), recreate_.get(), jump_.get() })
          addChild(button);
        delete_timer_.onTimerCallback() = [this] {
          delete_timer_.stopTimer();
          if (list_) {
            removeChild(list_.get());
            list_ = nullptr;
          }
          status_.setText("List deleted while scrolling.");
        };
        addChild(&status_);
        status_.setMono(true);
      }

      void draw(visage::Canvas& canvas) override {
        if (list_)
          status_.setText("yPosition " + visage::String(list_->yPosition(), 2).toUtf8() + " of " +
                          visage::String(list_->scrollableHeight() - list_->height(), 2).toUtf8() +
                          "   dpi " + visage::String(dpiScale(), 2).toUtf8());
        redraw();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        layoutButtonRow(takeTop(bounds, 34), { add_.get(), delete_.get(), recreate_.get(), jump_.get() });
        status_.setBounds(takeTop(bounds, 22));
        list_bounds_ = bounds;
        if (list_)
          list_->setBounds(list_bounds_);
      }

    private:
      void createList() {
        if (list_)
          removeChild(list_.get());
        list_ = std::make_unique<RowList>(300);
        addChild(list_.get());
        list_->setBounds(list_bounds_);
      }

      std::unique_ptr<RowList> list_;
      visage::Bounds list_bounds_;
      std::unique_ptr<visage::UiButton> add_;
      std::unique_ptr<visage::UiButton> delete_;
      std::unique_ptr<visage::UiButton> recreate_;
      std::unique_ptr<visage::UiButton> jump_;
      visage::EventTimer delete_timer_;
      Label status_ { "", 13, kDimText };
    };

    class LayoutPage : public visage::Frame {
    public:
      static constexpr int kItems = 14;

      LayoutPage() {
        container_.onDraw() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xff111418);
          canvas.fill(0, 0, container_.width(), container_.height());
        };
        container_.setFlexLayout(true);
        container_.layout().setPadding(8_px);
        for (int i = 0; i < kItems; ++i) {
          auto item = std::make_unique<visage::Frame>();
          visage::Frame* raw = item.get();
          unsigned int color = 0xff304870 + 0x00080c00 * (i % 8);
          item->onDraw() = [raw, color, i](visage::Canvas& canvas) {
            canvas.setColor(color);
            canvas.roundedRectangle(0, 0, raw->width(), raw->height(), 6);
            canvas.setColor(0xffffffff);
            canvas.text(std::to_string(i) + "  " + visage::String(raw->width(), 0).toUtf8() + "x" +
                            visage::String(raw->height(), 0).toUtf8(),
                        font(12), visage::Font::kCenter, 0, 0, raw->width(), raw->height());
          };
          container_.addChild(item.get());
          items_.push_back(std::move(item));
        }
        addChild(&container_);

        addToggle("Rows", true, [this](bool on) { container_.layout().setFlexRows(on); });
        addToggle("Wrap", true, [this](bool on) {
          wrap_ = on;
          applyWrap();
        });
        addToggle("Wrap reverse", false, [this](bool on) {
          wrap_reverse_ = on;
          applyWrap();
        });
        addToggle("Reverse direction", false,
                  [this](bool on) { container_.layout().setFlexReverseDirection(on); });
        addToggle("Percent sizes", false, [this](bool on) {
          percent_ = on;
          applyItemSizes();
        });
        addToggle("Grow", false, [this](bool on) {
          grow_ = on;
          applyItemSizes();
        });
        gap_.onChange() += [this](float value) {
          container_.layout().setFlexGap(visage::Dimension::logicalPixels(value));
          relayout();
        };
        addChild(&gap_);
        align_ = makeButton("Item alignment: Stretch", [this] {
          alignment_ = (alignment_ + 1) % 4;
          static const char* kNames[] = { "Stretch", "Start", "Center", "End" };
          static const visage::Layout::ItemAlignment kAlignments[] = {
            visage::Layout::ItemAlignment::Stretch, visage::Layout::ItemAlignment::Start,
            visage::Layout::ItemAlignment::Center, visage::Layout::ItemAlignment::End
          };
          container_.layout().setFlexItemAlignment(kAlignments[alignment_]);
          align_->setText(std::string("Item alignment: ") + kNames[alignment_]);
          relayout();
        });
        addChild(align_.get());

        container_.layout().setFlexRows(true);
        container_.layout().setFlexGap(visage::Dimension::logicalPixels(gap_.value()));
        applyWrap();
        applyItemSizes();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        std::vector<visage::Frame*> top;
        for (auto& toggle : toggles_)
          top.push_back(toggle.get());
        layoutButtonRow(takeTop(bounds, 32), top, 6);
        visage::Bounds second = takeTop(bounds, 36);
        auto columns = evenColumns(second, 2);
        gap_.setBounds(columns[0]);
        align_->setBounds(columns[1]);
        container_.setBounds(bounds);
      }

    private:
      void addToggle(const std::string& name, bool on, std::function<void(bool)> callback) {
        auto toggle = std::make_unique<visage::ToggleTextButton>(name);
        toggle->setFont(font(12));
        toggle->setToggled(on);
        toggle->setUndoable(false);
        toggle->onToggle() = [this, callback = std::move(callback)](visage::Button*, bool on) {
          callback(on);
          relayout();
        };
        addChild(toggle.get());
        toggles_.push_back(std::move(toggle));
      }

      void applyWrap() {
        if (wrap_reverse_)
          container_.layout().setFlexWrapReverse(true);
        else
          container_.layout().setFlexWrap(wrap_);
      }

      void applyItemSizes() {
        for (int i = 0; i < items_.size(); ++i) {
          visage::Layout& layout = items_[i]->layout();
          if (percent_) {
            layout.setWidth(visage::Dimension::widthPercent(12.0f + (i % 4) * 4.0f));
            layout.setHeight(visage::Dimension::heightPercent(10.0f + (i % 3) * 5.0f));
          }
          else {
            layout.setWidth(visage::Dimension::logicalPixels(70.0f + (i % 5) * 30.0f));
            layout.setHeight(visage::Dimension::logicalPixels(50.0f + (i % 3) * 25.0f));
          }
          layout.setFlexGrow(grow_ ? 1.0f : 0.0f);
        }
      }

      void relayout() {
        container_.computeLayout();
        container_.redrawAll();
      }

      visage::Frame container_;
      std::vector<std::unique_ptr<visage::Frame>> items_;
      std::vector<std::unique_ptr<visage::ToggleTextButton>> toggles_;
      Slider gap_ { "Gap", 0.0f, 40.0f, 8.0f, 0 };
      std::unique_ptr<visage::UiButton> align_;
      int alignment_ = 0;
      bool wrap_ = true;
      bool wrap_reverse_ = false;
      bool percent_ = false;
      bool grow_ = false;
    };

    class LoggingToggle : public visage::ToggleTextButton {
    public:
      LoggingToggle(const std::string& name, LogView* log) : ToggleTextButton(name), log_(log) {
        setFont(font(14));
      }

      void toggleValueChanged() override {
        log_->add(name() + ": toggleValueChanged -> " + (toggled() ? "on" : "off"));
      }

    private:
      LogView* log_ = nullptr;
    };

    class UndoPage : public visage::Frame,
                     public visage::UndoHistory,
                     public visage::UndoHistory::Listener {
    public:
      UndoPage() {
        addListener(this);
        createToggles();
        add_action_ = makeButton("Add counter +1 (undoable)", [this] {
          ++counter_;
          push(std::make_unique<visage::LambdaAction>([this] { --counter_; }, [this] { ++counter_; }));
        });
        undo_ = makeButton("Undo", [this] { undo(); });
        redo_ = makeButton("Redo", [this] { redo(); });
        clear_ = makeButton("Clear history", [this] { clearUndoHistory(); });
        delete_toggle_ = makeButton("Delete toggle A", [this] {
          if (toggle_a_) {
            removeChild(toggle_a_.get());
            toggle_a_ = nullptr;
            log_.add("Deleted toggle A. Its undo entries are still in the history.");
          }
        });
        recreate_ = makeButton("Recreate toggles", [this] { createToggles(); });
        for (auto* button : { add_action_.get(), undo_.get(), redo_.get(), clear_.get(),
                              delete_toggle_.get(), recreate_.get() })
          addChild(button);
        addChild(&log_);
        addChild(&state_);
        state_.setMono(true);
      }

      ~UndoPage() override { removeListener(this); }

      void undoPerformed() override { log_.add("Listener: undoPerformed"); }
      void redoPerformed() override { log_.add("Listener: redoPerformed"); }
      void undoActionAdded() override { log_.add("Listener: undoActionAdded"); }
      void undoHistoryCleared() override { log_.add("Listener: undoHistoryCleared"); }

      void draw(visage::Canvas& canvas) override {
        state_.setText(std::string("counter ") + std::to_string(counter_) + "   canUndo " +
                       (UndoHistory::canUndo() ? "yes" : "no") + "   canRedo " +
                       (UndoHistory::canRedo() ? "yes" : "no"));
        redraw();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        visage::Bounds toggles = takeTop(bounds, 40);
        std::vector<visage::Frame*> toggle_frames;
        if (toggle_a_)
          toggle_frames.push_back(toggle_a_.get());
        toggle_frames.push_back(toggle_b_.get());
        layoutButtonRow(toggles, toggle_frames);
        auto cells = grid(takeTop(bounds, 76), 3, 2, 6);
        visage::Frame* buttons[] = { add_action_.get(), undo_.get(),          redo_.get(),
                                     clear_.get(),      delete_toggle_.get(), recreate_.get() };
        for (int i = 0; i < 6; ++i)
          buttons[i]->setBounds(cells[i]);
        state_.setBounds(takeTop(bounds, 22));
        log_.setBounds(bounds);
      }

    private:
      void createToggles() {
        if (!toggle_a_) {
          toggle_a_ = std::make_unique<LoggingToggle>("Toggle A", &log_);
          addChild(toggle_a_.get());
        }
        if (!toggle_b_) {
          toggle_b_ = std::make_unique<LoggingToggle>("Toggle B", &log_);
          addChild(toggle_b_.get());
        }
        if (recreate_)
          resized();
      }

      LogView log_;
      Label state_ { "", 13, kText };
      int counter_ = 0;
      std::unique_ptr<LoggingToggle> toggle_a_;
      std::unique_ptr<LoggingToggle> toggle_b_;
      std::unique_ptr<visage::UiButton> add_action_;
      std::unique_ptr<visage::UiButton> undo_;
      std::unique_ptr<visage::UiButton> redo_;
      std::unique_ptr<visage::UiButton> clear_;
      std::unique_ptr<visage::UiButton> delete_toggle_;
      std::unique_ptr<visage::UiButton> recreate_;
    };

    class TimersPage : public visage::Frame {
    public:
      TimersPage() :
          int_animation_(1000), float_animation_(1000, visage::Animation<float>::kEaseInOut,
                                                 visage::Animation<float>::kEaseInOut) {
        int_animation_.setSourceValue(0);
        int_animation_.setTargetValue(100);
        float_animation_.setSourceValue(0.0f);
        float_animation_.setTargetValue(1.0f);

        timer_a_.onTimerCallback() = [this] { ++count_a_; };
        timer_b_.onTimerCallback() = [this] {
          timer_b_.stopTimer();
          timer_a_.stopTimer();
          timer_c_ = nullptr;
          log_.add("Timer B stopped timer A and deleted timer C at A=" + std::to_string(count_a_) +
                   " C=" + std::to_string(count_c_));
        };

        start_ = makeButton("Start A and C, B stops them in 2s", [this] {
          count_a_ = 0;
          count_c_ = 0;
          timer_a_.startTimer(50);
          timer_c_ = std::make_unique<visage::EventTimer>();
          timer_c_->onTimerCallback() = [this] { ++count_c_; };
          timer_c_->startTimer(50);
          timer_b_.startTimer(2000);
        });
        animate_ = makeButton("Toggle animations", [this] {
          targeting_ = !targeting_;
          int_animation_.target(targeting_);
          float_animation_.target(targeting_);
        });
        post_ = makeButton("Post 1000 callbacks from 4 threads", [this] {
          if (kWebBuild) {
            log_.add("This build has no threads, so callbacks can't be posted from other threads.");
            return;
          }
          posted_ = 0;
          std::weak_ptr<bool> alive = alive_;
          for (int t = 0; t < 4; ++t) {
            std::thread([this, alive] {
              for (int i = 0; i < 250; ++i) {
                visage::runOnEventThread([this, alive] {
                  if (!alive.expired())
                    ++posted_;
                });
              }
            }).detach();
          }
        });
        long_running_ = makeButton("Start 1s repeating clock", [this] {
          clock_.onTimerCallback() = [this] { ++clock_ticks_; };
          clock_.startTimer(1000);
        });
        for (auto* button : { start_.get(), animate_.get(), post_.get(), long_running_.get() })
          addChild(button);
        addChild(&log_);
      }

      void draw(visage::Canvas& canvas) override {
        int int_value = int_animation_.update();
        float float_value = float_animation_.update();

        visage::Bounds area = readout_area_;
        canvas.setColor(kPanel);
        canvas.roundedRectangle(area.x(), area.y(), area.width(), area.height(), 6);
        canvas.setColor(kText);
        std::string text = "Timer A ticks: " + std::to_string(count_a_) +
                           "    Timer C ticks: " + std::to_string(count_c_) +
                           "\nAnimation<int>: " + std::to_string(int_value) +
                           "    Animation<float>: " + visage::String(float_value, 3).toUtf8() +
                           "\nCallbacks run on event thread: " + std::to_string(posted_) +
                           "\n1 second clock ticks: " + std::to_string(clock_ticks_);
        drawLines(canvas, text, monoFont(14), visage::Font::kLeft, area.x() + 12, area.y(),
                  area.width() - 24, area.height() * 0.6f);

        float bar_y = area.y() + area.height() * 0.68f;
        float bar_w = area.width() - 24;
        canvas.setColor(kPanelLight);
        canvas.roundedRectangle(area.x() + 12, bar_y, bar_w, 10, 5);
        canvas.roundedRectangle(area.x() + 12, bar_y + 20, bar_w, 10, 5);
        canvas.setColor(kAccent);
        canvas.roundedRectangle(area.x() + 12, bar_y, bar_w * int_value / 100.0f, 10, 5);
        canvas.setColor(kPass);
        canvas.roundedRectangle(area.x() + 12, bar_y + 20, bar_w * float_value, 10, 5);
        redraw();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        layoutButtonRow(takeTop(bounds, 34),
                        { start_.get(), animate_.get(), post_.get(), long_running_.get() });
        readout_area_ = takeTop(bounds, 170);
        log_.setBounds(bounds);
      }

    private:
      std::shared_ptr<bool> alive_ = std::make_shared<bool>(true);
      visage::EventTimer timer_a_;
      visage::EventTimer timer_b_;
      visage::EventTimer clock_;
      std::unique_ptr<visage::EventTimer> timer_c_;
      int count_a_ = 0;
      int count_c_ = 0;
      int posted_ = 0;
      int clock_ticks_ = 0;
      bool targeting_ = false;
      visage::Animation<int> int_animation_;
      visage::Animation<float> float_animation_;
      visage::Bounds readout_area_;
      LogView log_;
      std::unique_ptr<visage::UiButton> start_;
      std::unique_ptr<visage::UiButton> animate_;
      std::unique_ptr<visage::UiButton> post_;
      std::unique_ptr<visage::UiButton> long_running_;
    };

    class WidgetsPage : public visage::Frame {
    public:
      static constexpr int kPoints = 200;
      static constexpr int kBars = 24;

      WidgetsPage() :
          lines_box_("GraphLine: bottom, top and center fills"), heat_box_("HeatMap"),
          bars_box_("BarList"), picker_box_("ColorPicker"), buttons_box_("Buttons"), bars_(kBars),
          heat_map_(64, 32) {
        visage::GraphLine::FillCenter centers[] = { visage::GraphLine::kBottom, visage::GraphLine::kTop,
                                                    visage::GraphLine::kCenter };
        for (int i = 0; i < 3; ++i) {
          lines_[i] = std::make_unique<visage::GraphLine>(kPoints);
          lines_[i]->setFilled(true);
          lines_[i]->setFillCenter(centers[i]);
          lines_box_.addChild(lines_[i].get());
        }
        addChild(&lines_box_);
        filled_ = std::make_unique<visage::ToggleTextButton>("Filled");
        filled_->setToggled(true);
        active_ = std::make_unique<visage::ToggleTextButton>("Active");
        active_->setToggled(true);
        animate_ = std::make_unique<visage::ToggleTextButton>("Animate");
        animate_->setToggled(true);
        for (auto* toggle : { filled_.get(), active_.get(), animate_.get() }) {
          toggle->setFont(font(12));
          toggle->setUndoable(false);
          lines_box_.addChild(toggle);
        }
        filled_->onToggle() = [this](visage::Button*, bool on) {
          for (auto& line : lines_)
            line->setFilled(on);
        };
        active_->onToggle() = [this](visage::Button*, bool on) {
          for (auto& line : lines_)
            line->setActive(on);
        };
        animate_->onToggle() = [this](visage::Button*, bool on) {
          animating_ = on;
          redraw();
        };

        heat_box_.addChild(&heat_map_);
        addChild(&heat_box_);
        bars_box_.addChild(&bars_);
        addChild(&bars_box_);

        picker_.onColorChange() += [this](visage::Color color) {
          picked_.setText("onColorChange: #" + color.toARGBHexString() + "  hdr " +
                          visage::String(color.hdr(), 2).toUtf8());
        };
        picker_box_.addChild(&picker_);
        picker_box_.addChild(&picked_);
        picked_.setMono(true);
        addChild(&picker_box_);

        icon_ = std::make_unique<visage::ToggleIconButton>(resources::icons::check_circle_svg.data,
                                                           resources::icons::check_circle_svg.size, true);
        icon_->setUndoable(false);
        text_toggle_ = std::make_unique<visage::ToggleTextButton>("Toggle text button");
        text_toggle_->setFont(font(14));
        action_ = std::make_unique<visage::UiButton>("Action button");
        action_->setFont(font(14));
        action_->setActionButton();
        plain_ = std::make_unique<visage::UiButton>("Toggle on mouse down");
        plain_->setFont(font(14));
        plain_->setToggleOnMouseDown(true);
        for (visage::Frame* button : { (visage::Frame*)icon_.get(), (visage::Frame*)text_toggle_.get(),
                                       (visage::Frame*)action_.get(), (visage::Frame*)plain_.get() })
          buttons_box_.addChild(button);
        addChild(&buttons_box_);
      }

      void draw(visage::Canvas& canvas) override {
        if (!animating_)
          return;

        double t = canvas.time();
        for (int l = 0; l < 3; ++l) {
          for (int i = 0; i < kPoints; ++i) {
            float x = i / (kPoints - 1.0f);
            lines_[l]->set(i, 0.5f + 0.35f * std::sin(6.0f * x + t * (1.0 + 0.3 * l)) *
                                         std::cos(2.0f * x - t * 0.5));
          }
        }
        for (int y = 0; y < heat_map_.dataHeight(); ++y) {
          for (int x = 0; x < heat_map_.dataWidth(); ++x)
            heat_map_.set(x, y, 0.5f + 0.5f * std::sin(x * 0.2f + t) * std::cos(y * 0.3f - t * 0.7f));
        }
        float bar_width = bars_.width() / kBars;
        for (int i = 0; i < kBars; ++i) {
          float h = (0.5f + 0.5f * std::sin(t * 2.0 + i * 0.4)) * bars_.height();
          bars_.positionBar(i, i * bar_width + 1, bars_.height() - h, bar_width - 2, h);
        }
        redraw();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        auto rows = splitRows(bounds, { 1.4f, 1.0f, 1.3f });
        lines_box_.setBounds(rows[0]);
        visage::Bounds lines_content = lines_box_.contentBounds();
        layoutButtonRow(takeBottom(lines_content, 30, 6),
                        { filled_.get(), active_.get(), animate_.get() });
        auto line_columns = evenColumns(lines_content, 3);
        for (int i = 0; i < 3; ++i)
          lines_[i]->setBounds(line_columns[i]);

        auto middle = evenColumns(rows[1], 2);
        heat_box_.setBounds(middle[0]);
        heat_map_.setBounds(heat_box_.contentBounds());
        bars_box_.setBounds(middle[1]);
        bars_.setBounds(bars_box_.contentBounds());

        auto bottom = evenColumns(rows[2], 2);
        picker_box_.setBounds(bottom[0]);
        visage::Bounds picker_content = picker_box_.contentBounds();
        picked_.setBounds(takeBottom(picker_content, 20, 4));
        picker_.setBounds(picker_content);
        buttons_box_.setBounds(bottom[1]);
        auto cells = grid(buttons_box_.contentBounds(), 2, 2);
        icon_->setBounds(cells[0]);
        text_toggle_->setBounds(cells[1]);
        action_->setBounds(cells[2]);
        plain_->setBounds(cells[3]);
      }

    private:
      Box lines_box_;
      Box heat_box_;
      Box bars_box_;
      Box picker_box_;
      Box buttons_box_;
      std::unique_ptr<visage::GraphLine> lines_[3];
      std::unique_ptr<visage::ToggleTextButton> filled_;
      std::unique_ptr<visage::ToggleTextButton> active_;
      std::unique_ptr<visage::ToggleTextButton> animate_;
      visage::BarList bars_;
      visage::HeatMap heat_map_;
      visage::ColorPicker picker_;
      Label picked_ { "Pick a color or type hex into the picker", 12, kDimText };
      std::unique_ptr<visage::ToggleIconButton> icon_;
      std::unique_ptr<visage::ToggleTextButton> text_toggle_;
      std::unique_ptr<visage::UiButton> action_;
      std::unique_ptr<visage::UiButton> plain_;
      bool animating_ = true;
    };

    class PaletteColorWindow : public visage::ApplicationWindow {
    public:
      explicit PaletteColorWindow(visage::Palette* palette) : editor_(palette) {
        setTitle("Palette colors");
        addChild(&editor_);
        editor_.layout().setMargin(0);
      }

      void setEditedPalette(visage::Palette* palette) { editor_.setEditedPalette(palette); }

    private:
      visage::PaletteColorEditor editor_;
    };

    class PaletteValueWindow : public visage::ApplicationWindow {
    public:
      explicit PaletteValueWindow(visage::Palette* palette) : editor_(palette) {
        setTitle("Palette values");
        addChild(&editor_);
        editor_.layout().setMargin(0);
      }

      void setEditedPalette(visage::Palette* palette) { editor_.setEditedPalette(palette); }

    private:
      visage::PaletteValueEditor editor_;
    };

    class PalettePage : public visage::Frame {
    public:
      PalettePage() : color_window_(&palette_a_), value_window_(&palette_a_) {
        palette_a_.initWithDefaults();
        palette_b_.initWithDefaults();
        palette_b_.setColor(ManualSwatchPrimary, visage::Color(0xffffaa00));
        palette_b_.setColor(ManualSwatchBackground, visage::Color(0xff3a2a40));
        palette_b_.setValue(ManualSwatchRounding, 40.0f);
        setPalette(&palette_a_);

        swatches_.onDraw() = [this](visage::Canvas& canvas) {
          float w = swatches_.width();
          float h = swatches_.height();
          float rounding = swatches_.paletteValue(ManualSwatchRounding);
          float border = swatches_.paletteValue(ManualSwatchBorder);
          canvas.setColor(ManualSwatchBackground);
          canvas.roundedRectangle(0, 0, w, h, rounding);
          visage::theme::ColorId ids[] = { ManualSwatchPrimary, ManualSwatchSecondary, ManualSwatchTertiary };
          for (int i = 0; i < 3; ++i) {
            float x = 20 + i * (w - 40) / 3.0f;
            canvas.setColor(ids[i]);
            canvas.roundedRectangle(x, 20, (w - 40) / 3.0f - 16, h - 40, rounding);
            canvas.setColor(0xffffffff);
            canvas.roundedRectangleBorder(x, 20, (w - 40) / 3.0f - 16, h - 40, rounding, border);
          }
        };
        addChild(&swatches_);

        addButton("Open color editor", [this] { color_window_.show(320_px, 700_px); });
        addButton("Open value editor", [this] { value_window_.show(320_px, 500_px); });
        addButton("Switch to palette B", [this] { switchPalette(); });
        addButton("Save current palette", [this] {
          visage::File file = visage::appDataDirectory() / "VisageManualTests" / "palette.txt";
          visage::replaceFileWithText(file, current()->encode());
          status_.setText("Saved to " + pathString(file));
        });
        addButton("Load saved palette", [this] {
          visage::File file = visage::appDataDirectory() / "VisageManualTests" / "palette.txt";
          if (!visage::fileExists(file)) {
            status_.setText("Nothing saved yet");
            return;
          }
          current()->decode(visage::loadFileAsString(file));
          swatches_.redraw();
          redrawAll();
          status_.setText("Loaded " + pathString(file));
        });
        addButton("Load garbage palette data", [this] {
          visage::Palette scratch;
          scratch.decode("this is not palette data \x01\x02\x03 999999999 -1 nan");
          status_.setText("Decoded garbage into a scratch palette without crashing");
        });
        addChild(&status_);
        status_.setMultiLine(true);
      }

      ~PalettePage() override {
        color_window_.hide();
        value_window_.hide();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        std::vector<visage::Frame*> frames;
        for (auto& button : buttons_)
          frames.push_back(button.get());
        auto cells = grid(takeTop(bounds, 76), 3, 2, 6);
        for (int i = 0; i < frames.size(); ++i)
          frames[i]->setBounds(cells[i]);
        status_.setBounds(takeBottom(bounds, 40));
        swatches_.setBounds(bounds);
      }

    private:
      visage::Palette* current() { return using_b_ ? &palette_b_ : &palette_a_; }

      void switchPalette() {
        using_b_ = !using_b_;
        setPalette(current());
        color_window_.setEditedPalette(current());
        value_window_.setEditedPalette(current());
        buttons_[2]->setText(using_b_ ? "Switch to palette A" : "Switch to palette B");
        status_.setText(using_b_ ? "Editing palette B (orange primary, round corners)" : "Editing palette A");
        redrawAll();
      }

      void addButton(const std::string& text, std::function<void()> callback) {
        buttons_.push_back(makeButton(text, std::move(callback)));
        addChild(buttons_.back().get());
      }

      visage::Palette palette_a_;
      visage::Palette palette_b_;
      bool using_b_ = false;
      PaletteColorWindow color_window_;
      PaletteValueWindow value_window_;
      visage::Frame swatches_;
      Label status_ { "", 13, kDimText };
      std::vector<std::unique_ptr<visage::UiButton>> buttons_;
    };
  }

  void addUiTests(std::vector<TestDefinition>& tests) {
    tests.push_back(
        { "frame_lifecycle",
          "UI",
          "Frame Lifecycle",
          "Frames that delete, remove or move themselves from inside their own event handlers, "
          "plus focus and visibility edge cases.",
          {
              "\"Delete me on mouse up\" disappears when released over it, and also when you press "
              "it, drag outside and release. No crash.",
              "\"Remove me from my parent\" disappears on mouse down. \"Recreate\" brings back "
              "both.",
              "\"Delete my parent\" removes its whole box on mouse down with no crash.",
              "Clicking Mover moves it between A and B. It only ever shows in one container and "
              "the "
              "child counts in the log stay 1 and 0.",
              "Mover and Orphan log onHierarchyChange each time they move.",
              "Focus the editor, click \"Hide this box\", keep typing. After the box hides the log "
              "shows focus lost and your typing goes nowhere. Showing it again works normally.",
              "Moving Orphan into the visible parent draws it immediately. Moving it back hides "
              "it.",
              "At alpha 0.5 the three circles fade as one group: their overlaps aren't darker or "
              "more opaque than the rest.",
          },
          [] { return std::make_unique<FrameLifecyclePage>(); } });

    tests.push_back(
        { "scrolling",
          "UI",
          "Scrolling",
          "A ScrollableFrame with 300 rows that highlight on hover. The readout shows the scroll "
          "position.",
          {
              "Slow trackpad scrolling moves smoothly with fractional positions and no 1-pixel "
              "stepping or jitter.",
              "You can't scroll past the top or the bottom. Flicking at the end doesn't overshoot "
              "and stick.",
              "Dragging the scroll bar thumb keeps it under the mouse with no drift, even when you "
              "drag past the ends and come back.",
              "Hover a row and scroll with the wheel without moving the mouse. The highlight moves "
              "to the row now under the mouse.",
              "\"Delete list in 1 second\", then flick-scroll so it is still moving when deleted. "
              "No crash. \"Recreate list\" works.",
              "\"Add 50 rows\" extends how far you can scroll.",
              "The red \"You should never see this row\" label (added hidden) never shows.",
              "\"setYPosition(1234.6)\" jumps there and rows stay sharp.",
          },
          [] { return std::make_unique<ScrollingPage>(); } });

    tests.push_back(
        { "layout",
          "UI",
          "Flex Layout",
          "A flex container with 14 children. Each child shows its index and size.",
          {
              "With Rows and Wrap on, items fill left to right and wrap onto new lines going down.",
              "Wrap reverse with Rows on stacks the wrapped lines from the bottom up, still left "
              "to "
              "right.",
              "Wrap reverse with Rows off wraps columns from right to left.",
              "Reverse direction reverses the order within each line.",
              "Percent sizes resolve against the container, not the line. Resizing the window "
              "resizes the items in proportion.",
              "Grow makes items fill each line with no gaps at the end.",
              "The gap slider changes the space between items and between lines.",
              "Cycling item alignment moves items to the start, center or end of their line, or "
              "stretches them.",
          },
          [] { return std::make_unique<LayoutPage>(); } });

    tests.push_back(
        { "undo",
          "UI",
          "Undo History",
          "The page has its own undo history. A listener logs every history notification.",
          {
              "Clicking a toggle logs undoActionAdded and toggleValueChanged.",
              "Undo and Redo flip the toggles back and forth and each one logs toggleValueChanged.",
              "Undo and redo don't add new history entries: after undoing everything, Redo can "
              "redo "
              "it all again.",
              "\"Add counter +1\" changes the counter, and undo and redo move it down and up.",
              "\"Clear history\" logs undoHistoryCleared and canUndo and canRedo both turn to no.",
              "Toggle A a few times, delete it, then press Undo several times. No crash, and the "
              "other entries still undo.",
              "After \"Recreate toggles\" the new Toggle A records undo steps normally.",
          },
          [] { return std::make_unique<UndoPage>(); } });

    tests.push_back(
        { "timers",
          "UI",
          "Timers and Threads",
          "Event timers, animations and callbacks posted from background threads.",
          {
              "\"Start A and C\" counts A and C up for 2 seconds, then both stop. The counts in "
              "the "
              "log match the readout and never change afterwards.",
              "\"Toggle animations\" moves Animation<int> through every value from 0 to 100, not "
              "jumping between ends, and the float animation eases.",
              "Toggling again mid-animation reverses smoothly from where it was.",
              "\"Post 1000 callbacks\" brings the count to exactly 1000. Clicking it repeatedly "
              "quickly causes no crash.",
              "Start the 1 second clock, then set the system clock back an hour (or a day). The "
              "clock keeps ticking and animations keep running.",
              "Leaving and coming back to this page while timers run causes no crash.",
          },
          [] { return std::make_unique<TimersPage>(); },
          true,
          "This build has no threads, so \"Post 1000 callbacks\" is disabled. Skip that check." });

    tests.push_back(
        { "widgets",
          "UI",
          "Widgets",
          "GraphLine, HeatMap, BarList, ColorPicker and buttons.",
          {
              "The bottom-fill graph fills down to the bottom, the top-fill graph fills up to the "
              "top "
              "and the center one fills toward the middle.",
              "Unchecking Filled or Active redraws the graphs immediately, even with Animate off.",
              "The heat map and bars animate smoothly with no seams or flicker.",
              "Dragging in the color picker logs color changes.",
              "Typing a full 8-digit hex value with alpha into the picker keeps the alpha. A "
              "partial "
              "hex value doesn't send a change until it's complete.",
              "The icon button toggles and has a soft shadow. The other buttons highlight on hover "
              "and respond to clicks.",
          },
          [] { return std::make_unique<WidgetsPage>(); } });

    tests.push_back(
        { "palette",
          "UI",
          "Palette Editing",
          "The swatches use theme colors and values from the page's palette. The editors open in "
          "their own windows.",
          {
              "Editing ManualSwatchPrimary in the color editor updates the first swatch live.",
              "Editing ManualSwatchRounding in the value editor changes the corner rounding live.",
              "Switch to palette B and edit a color. Palette B changes and switching back shows "
              "palette A unchanged.",
              "Save, change a color, then Load. The saved colors come back.",
              "\"Load garbage palette data\" doesn't crash or hang.",
              "Mouse-moving over the color editor list stays responsive.",
          },
          [] { return std::make_unique<PalettePage>(); },
          false });
  }
}
