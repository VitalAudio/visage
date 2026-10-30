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

#include "test_helpers.h"

using namespace visage::dimension;

namespace manual {
  namespace {
    class MouseDownButton : public visage::Frame {
    public:
      MouseDownButton(std::string text, std::function<void()> callback) :
          text_(std::move(text)), callback_(std::move(callback)) { }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(hover_ ? 0xff4a5568 : kPanelLight);
        canvas.roundedRectangle(0, 0, width(), height(), 6);
        canvas.setColor(kText);
        canvas.text(text_, font(14), visage::Font::kCenter, 0, 0, width(), height());
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
        std::function<void()> callback = callback_;
        callback();
      }

    private:
      std::string text_;
      std::function<void()> callback_;
      bool hover_ = false;
    };

    class TestWindow : public visage::ApplicationWindow {
    public:
      TestWindow(std::string label, unsigned int color) : label_(std::move(label)), color_(color) {
        setTitle(label_);
      }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(color_);
        canvas.fill(0, 0, width(), height());
        canvas.setColor(0xffffffff);
        std::string info = label_ + "\nlogical " + visage::String(width(), 0).toUtf8() + " x " +
                           visage::String(height(), 0).toUtf8() + "   native " +
                           std::to_string(nativeWidth()) + " x " + std::to_string(nativeHeight()) +
                           "\nDPI scale " + visage::String(dpiScale(), 2).toUtf8();
        if (!extra_.empty())
          info += "\n" + extra_;
        drawLines(canvas, info, font(15), visage::Font::kCenter, 0, 0, width(), height() * 0.6f);
        canvas.setColor(0xffffffff);
        canvas.rectangleBorder(0, 0, width(), height(), 2);
      }

      void resized() override {
        visage::Bounds bounds(16, height() * 0.6f, width() - 32, height() * 0.4f - 16);
        std::vector<visage::Frame*> frames;
        for (auto& button : buttons_)
          frames.push_back(button.get());
        if (!frames.empty())
          layoutButtonRow(bounds.reduced(0, 0, bounds.height() * 0.25f, 0), frames);
        redraw();
      }

      void addButton(std::unique_ptr<visage::Frame> button) {
        addChild(button.get());
        buttons_.push_back(std::move(button));
      }

      void setExtra(std::string extra) {
        extra_ = std::move(extra);
        redraw();
      }

    private:
      std::string label_;
      std::string extra_;
      unsigned int color_ = 0;
      std::vector<std::unique_ptr<visage::Frame>> buttons_;
    };

    class WindowsPage : public visage::Frame {
    public:
      WindowsPage() :
          basic_("Basic window", 0xff2f4f6f), refuses_("Refuses to close", 0xff6f2f2f),
          limits_("Min 300x200, optional fixed aspect", 0xff2f6f4f),
          client_("Client decorated", 0xff4f2f6f), on_top_("Always on top", 0xff6f5f2f),
          placed_("Placed at 100, 100", 0xff2f6f6f) {
        client_.setWindowDecoration(visage::Window::Decoration::Client);
        on_top_.setWindowOnTop(true);
        limits_.setMinimumDimensions(300, 200);

        for (TestWindow* window : { &basic_, &refuses_, &limits_, &client_, &on_top_, &placed_ }) {
          window->onShow() += [this, window] { log_.add(window->title() + ": onShow"); };
          window->onHide() += [this, window] { log_.add(window->title() + ": onHide"); };
        }

        refuses_.onCloseRequested() = [this] {
          log_.add(std::string("Refuses to close: close requested, ") +
                   (allow_close_ ? "allowed" : "refused"));
          return allow_close_;
        };
        refuses_.setExtra("Close button does nothing until\n\"Allow close\" is toggled on");

        addButton("Show / hide basic window", [this] { toggle(basic_, 420, 300); });
        addButton("Hide, then re-show from a timer", [this] {
          if (!basic_.isShowing())
            basic_.show(420_px, 300_px);
          basic_.hide();
          reshow_timer_.startTimer(1000);
          log_.add("Basic window hidden, re-showing in 1 second");
        });
        addButton("Hide and show 5 times quickly", [this] {
          for (int i = 0; i < 5; ++i) {
            basic_.show(420_px, 300_px);
            basic_.hide();
          }
          basic_.show(420_px, 300_px);
        });
        addButton("Show refuses-close window", [this] { toggle(refuses_, 380, 260); });
        allow_close_button_ = std::make_unique<visage::ToggleTextButton>("Allow close");
        allow_close_button_->setFont(font(13));
        allow_close_button_->onToggle() = [this](visage::Button*, bool on) { allow_close_ = on; };
        addChild(allow_close_button_.get());
        addButton("Show size-limited window", [this] { toggle(limits_, 420, 300); });
        aspect_button_ = std::make_unique<visage::ToggleTextButton>("Fixed aspect ratio");
        aspect_button_->setFont(font(13));
        aspect_button_->onToggle() = [this](visage::Button*, bool on) {
          limits_.setFixedAspectRatio(on);
          limits_.setExtra(on ? "Aspect ratio is fixed" : "Aspect ratio is free");
        };
        addChild(aspect_button_.get());
        addButton("Show client-decorated window", [this] { toggle(client_, 460, 320); });
        addButton("Show always-on-top window", [this] { toggle(on_top_, 360, 240); });
        addButton("Show window at screen 100, 100", [this] {
          if (placed_.isShowing())
            placed_.hide();
          else
            placed_.show(100_px, 100_px, 380_px, 260_px);
        });
        addButton("Open self-closing window", [this] { openSelfClosing(); });
        addButton("Open window, destroy it in 2 seconds", [this] {
          dynamic_ = std::make_unique<TestWindow>("Destroyed in 2 seconds while open", 0xff5a3a2a);
          dynamic_->show(400_px, 260_px);
          destroy_timer_.startTimer(2000);
        });
        addButton("Open 5 windows at once", [this] {
          many_.clear();
          for (int i = 0; i < 5; ++i) {
            auto window = std::make_unique<TestWindow>("Window " + std::to_string(i + 1),
                                                       0xff303848 + 0x00100800 * i);
            window->show(visage::Dimension::logicalPixels(60 + 50 * i),
                         visage::Dimension::logicalPixels(80 + 40 * i), 320_px, 200_px);
            many_.push_back(std::move(window));
          }
        });
        addButton("Close all", [this] {
          for (TestWindow* window : { &basic_, &limits_, &client_, &on_top_, &placed_ })
            window->hide();
          many_.clear();
          self_closing_ = nullptr;
        });

        reshow_timer_.onTimerCallback() = [this] {
          reshow_timer_.stopTimer();
          basic_.show(420_px, 300_px);
          log_.add("Basic window re-shown from timer");
        };
        destroy_timer_.onTimerCallback() = [this] {
          destroy_timer_.stopTimer();
          dynamic_ = nullptr;
          log_.add("Destroyed the dynamic window while it was open");
        };

        addChild(&log_);
      }

      ~WindowsPage() override {
        for (TestWindow* window : { &basic_, &refuses_, &limits_, &client_, &on_top_, &placed_ })
          window->onCloseRequested().clear();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        std::vector<visage::Frame*> frames;
        for (auto& button : buttons_)
          frames.push_back(button.get());
        frames.insert(frames.begin() + 4, allow_close_button_.get());
        frames.insert(frames.begin() + 6, aspect_button_.get());
        int columns = 2;
        int rows = (frames.size() + columns - 1) / columns;
        visage::Bounds button_area = takeTop(bounds, rows * 40.0f);
        auto cells = grid(button_area, columns, rows, 6);
        for (int i = 0; i < frames.size(); ++i)
          frames[i]->setBounds(cells[i]);
        log_.setBounds(bounds);
      }

    private:
      void addButton(const std::string& text, std::function<void()> callback) {
        buttons_.push_back(makeButton(text, std::move(callback)));
        addChild(buttons_.back().get());
      }

      void toggle(TestWindow& window, int w, int h) {
        if (window.isShowing())
          window.hide();
        else
          window.show(visage::Dimension::logicalPixels(w), visage::Dimension::logicalPixels(h));
      }

      void openSelfClosing() {
        self_closing_ = std::make_unique<TestWindow>("Self-closing window", 0xff3a4a2a);
        self_closing_->setExtra("Each button acts on mouse down");
        self_closing_->addButton(std::make_unique<MouseDownButton>("close()", [this] {
          log_.add("Self-closing: close() from mouse down");
          self_closing_->close();
        }));
        self_closing_->addButton(std::make_unique<MouseDownButton>("hide()", [this] {
          log_.add("Self-closing: hide() from mouse down");
          self_closing_->hide();
        }));
        self_closing_->addButton(std::make_unique<MouseDownButton>("Delete window", [this] {
          log_.add("Self-closing: deleting the window from its own mouse down");
          self_closing_ = nullptr;
        }));
        self_closing_->show(460_px, 260_px);
      }

      LogView log_;
      TestWindow basic_;
      TestWindow refuses_;
      TestWindow limits_;
      TestWindow client_;
      TestWindow on_top_;
      TestWindow placed_;
      std::unique_ptr<TestWindow> self_closing_;
      std::unique_ptr<TestWindow> dynamic_;
      std::vector<std::unique_ptr<TestWindow>> many_;
      visage::EventTimer reshow_timer_;
      visage::EventTimer destroy_timer_;
      bool allow_close_ = false;
      std::vector<std::unique_ptr<visage::UiButton>> buttons_;
      std::unique_ptr<visage::ToggleTextButton> allow_close_button_;
      std::unique_ptr<visage::ToggleTextButton> aspect_button_;
    };

    class DpiPage : public visage::Frame {
    public:
      static constexpr int kStripFrames = 37;

      DpiPage() : editor_box_("Text editor (font must keep its size after DPI changes)") {
        for (int i = 0; i < kStripFrames; ++i) {
          auto frame = std::make_unique<visage::Frame>();
          unsigned int color = (i % 2) ? 0xffffffff : 0xffff3366;
          visage::Frame* raw = frame.get();
          frame->onDraw() = [raw, color](visage::Canvas& canvas) {
            canvas.setColor(color);
            canvas.fill(0, 0, raw->width(), raw->height());
          };
          strip_.addChild(frame.get());
          strip_frames_.push_back(std::move(frame));
        }
        strip_.onResize() += [this] {
          float w = strip_.width() / kStripFrames;
          for (int i = 0; i < kStripFrames; ++i)
            strip_frames_[i]->setBounds(i * w, 0, w, strip_.height());
        };
        strip_.onDraw() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xff00ff00);
          canvas.fill(0, 0, strip_.width(), strip_.height());
        };
        addChild(&strip_);

        flex_strip_.setFlexLayout(true);
        flex_strip_.onDraw() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xff00ff00);
          canvas.fill(0, 0, flex_strip_.width(), flex_strip_.height());
        };
        for (int i = 0; i < kStripFrames; ++i) {
          auto frame = std::make_unique<visage::Frame>();
          unsigned int color = (i % 2) ? 0xffffffff : 0xff3366ff;
          visage::Frame* raw = frame.get();
          frame->onDraw() = [raw, color](visage::Canvas& canvas) {
            canvas.setColor(color);
            canvas.fill(0, 0, raw->width(), raw->height());
          };
          frame->layout().setFlexGrow(1.0f);
          frame->layout().setHeight(100_vh);
          flex_strip_.addChild(frame.get());
          flex_frames_.push_back(std::move(frame));
        }
        addChild(&flex_strip_);

        hairlines_.onDraw() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xff000000);
          canvas.fill(0, 0, hairlines_.width(), hairlines_.height());
          canvas.setNativePixelScale();
          canvas.setColor(0xffffffff);
          for (int x = 0; x < hairlines_.nativeWidth(); x += 4)
            canvas.fill(x, 0, 1, hairlines_.nativeHeight() / 2);
          for (int y = hairlines_.nativeHeight() / 2; y < hairlines_.nativeHeight(); y += 4)
            canvas.fill(0, y, hairlines_.nativeWidth(), 1);
        };
        addChild(&hairlines_);

        editor_.setFont(font(18));
        editor_.setText("Move me to another monitor");
        editor_box_.addChild(&editor_);
        editor_box_.onResize() += [this] { editor_.setBounds(editor_box_.contentBounds()); };
        addChild(&editor_box_);
        addChild(&readout_);
        readout_.setMono(true);
        readout_.setMultiLine(true);
        addChild(&strip_label_);
        addChild(&flex_label_);
        addChild(&hairline_label_);
        onDpiChange() += [this] { ++dpi_changes_; };
      }

      void draw(visage::Canvas& canvas) override {
        readout_.setText("DPI scale: " + visage::String(dpiScale(), 3).toUtf8() +
                         "   DPI changes seen: " + std::to_string(dpi_changes_) +
                         "\nPage logical: " + visage::String(width(), 1).toUtf8() + " x " +
                         visage::String(height(), 1).toUtf8() +
                         "   native: " + std::to_string(nativeWidth()) + " x " +
                         std::to_string(nativeHeight()) + "\nStrip frame width: " +
                         visage::String(strip_.width() / kStripFrames, 3).toUtf8() + " logical px");
        redraw();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        readout_.setBounds(takeTop(bounds, 64));
        strip_label_.setBounds(takeTop(bounds, 20, 4));
        strip_.setBounds(takeTop(bounds, 60));
        flex_label_.setBounds(takeTop(bounds, 20, 4));
        flex_strip_.setBounds(takeTop(bounds, 60));
        hairline_label_.setBounds(takeTop(bounds, 20, 4));
        hairlines_.setBounds(takeTop(bounds, 120));
        editor_box_.setBounds(takeTop(bounds, 80));
      }

    private:
      visage::Frame strip_;
      visage::Frame flex_strip_;
      visage::Frame hairlines_;
      std::vector<std::unique_ptr<visage::Frame>> strip_frames_;
      std::vector<std::unique_ptr<visage::Frame>> flex_frames_;
      Box editor_box_;
      visage::TextEditor editor_;
      Label readout_ { "", 13, kText };
      Label strip_label_ {
        "37 frames with fractional widths over green: no green should show through", 13, kDimText
      };
      Label flex_label_ { "The same with flex layout", 13, kDimText };
      Label hairline_label_ { "1 native pixel lines every 4 pixels: even and sharp", 13, kDimText };
      int dpi_changes_ = 0;
    };

    class LifecyclePage : public visage::Frame {
    public:
      LifecyclePage() {
        status_.setMultiLine(true);
        status_.setText(lastSessionStatus());
        status_.setBackground(kPanel);
        addChild(&status_);

        addButton("visage::closeApplication()", [] { visage::closeApplication(); });
        addButton("Force a crash", [] { visage::forceCrash(); });
        addButton("Show message box", [] {
          visage::showMessageBox("Visage \"message\" box ✓", "This text has 'quotes', \"double "
                                                             "quotes\", <tags>, a backslash \\ and "
                                                             "Unicode: ünïcødé 🎉.\nSecond line.");
        });
        addButton("Install test menu bar", [this] { installMenuBar(false); });
        addButton("Open popup, then quit with Cmd+Q", [this] {
          visage::PopupMenu menu;
          menu.addOption(1, "Leave this open and quit the app");
          menu.addOption(2, "It should close with the app");
          menu.show(buttons_.back().get());
        });
        addChild(&log_);
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        status_.setBounds(takeTop(bounds, std::max(60.0f, status_.heightForWidth(bounds.width()) + 8)));
        std::vector<visage::Frame*> frames;
        for (auto& button : buttons_)
          frames.push_back(button.get());
        auto cells = grid(takeTop(bounds, 3 * 40.0f), 2, 3, 6);
        for (int i = 0; i < frames.size(); ++i)
          frames[i]->setBounds(cells[i]);
        log_.setBounds(bounds);
      }

    private:
      void addButton(const std::string& text, std::function<void()> callback) {
        buttons_.push_back(makeButton(text, std::move(callback)));
        addChild(buttons_.back().get());
      }

      void installMenuBar(bool alternate) {
        visage::PopupMenu menu_bar;
        visage::PopupMenu test_menu(alternate ? "Replaced Menu" : "Manual Test");
        test_menu.addOption(1, "Log hello")
            .withNativeKeyboardShortcut(visage::kModifierCmd | visage::kModifierShift, "h");
        test_menu.addOption(2, alternate ? "Restore original menu bar" : "Replace the menu bar from here");
        test_menu.addBreak();
        test_menu.addOption(3, "Disabled item").enable(false);
        test_menu.addOption(4, "Checked item").select(true);
        visage::PopupMenu sub_menu("Sub menu");
        sub_menu.addOption(5, "Sub item A");
        sub_menu.addOption(6, "Sub item B");
        test_menu.addSubMenu(sub_menu);
        test_menu.onSelection() = [this, alternate](int id) {
          log_.add("Menu bar selection " + std::to_string(id));
          if (id == 2)
            installMenuBar(!alternate);
        };
        menu_bar.addSubMenu(test_menu);
        menu_bar.setAsNativeMenuBar();
        log_.add(alternate ? "Installed replacement menu bar" : "Installed test menu bar");
      }

      Label status_ { "", 15, kText };
      LogView log_;
      std::vector<std::unique_ptr<visage::UiButton>> buttons_;
    };

    class PopupPage : public visage::Frame {
    public:
      PopupPage() {
        addButton("Basic menu", [this](visage::Frame* source) { basicMenu().show(source); });
        addButton("8 levels of sub menus", [this](visage::Frame* source) {
          visage::PopupMenu menu = deepMenu(1, 8);
          menu.show(source);
        });
        addButton("Menu with 150 items", [this](visage::Frame* source) {
          visage::PopupMenu menu;
          for (int i = 0; i < 150; ++i)
            menu.addOption(i, "Long menu item " + std::to_string(i));
          connect(menu);
          menu.show(source);
        });
        addButton("Sub menu with 150 items", [this](visage::Frame* source) {
          visage::PopupMenu menu;
          menu.addOption(1, "First");
          visage::PopupMenu long_sub("Long sub menu");
          for (int i = 0; i < 150; ++i)
            long_sub.addOption(100 + i, "Sub item " + std::to_string(i));
          menu.addSubMenu(long_sub);
          visage::PopupMenu other("Short sub menu");
          other.addOption(2, "Short item");
          menu.addSubMenu(other);
          connect(menu);
          menu.show(source);
        });
        addButton("At a point near the bottom", [this](visage::Frame* source) {
          visage::PopupMenu menu = basicMenu();
          menu.show(this, { width() * 0.5f, height() - 20.0f });
        });

        corner_ = makeButton("Bottom-right corner", [this] { basicMenu().show(corner_.get()); });
        corner_->setToggleOnMouseDown(true);
        addChild(corner_.get());
        addChild(&log_);
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        std::vector<visage::Frame*> frames;
        for (auto& button : buttons_)
          frames.push_back(button.get());
        auto cells = grid(takeTop(bounds, 3 * 40.0f), 2, 3, 6);
        for (int i = 0; i < frames.size(); ++i)
          frames[i]->setBounds(cells[i]);
        corner_->setBounds(width() - 172, height() - 46, 160, 34);
        bounds = bounds.reduced(0, 0, 0, 50);
        log_.setBounds(bounds);
      }

    private:
      void addButton(const std::string& text, std::function<void(visage::Frame*)> callback) {
        auto button = std::make_unique<visage::UiButton>(text);
        button->setName(text);
        button->setFont(font(14));
        button->setToggleOnMouseDown(true);
        visage::UiButton* raw = button.get();
        button->onToggle() = [raw, callback = std::move(callback)](visage::Button*, bool) {
          callback(raw);
        };
        addChild(button.get());
        buttons_.push_back(std::move(button));
      }

      void connect(visage::PopupMenu& menu) {
        menu.onSelection() = [this](int id) { log_.add("Selected id " + std::to_string(id)); };
        menu.onCancel() = [this] { log_.add("Cancelled"); };
      }

      visage::PopupMenu basicMenu() {
        visage::PopupMenu menu;
        menu.addOption(1, "Plain item");
        menu.addOption(2, "Selected item").select(true);
        menu.addOption(3, "Disabled item").enable(false);
        menu.addBreak();
        visage::PopupMenu sub("Sub menu");
        sub.addOption(10, "Sub item 10");
        sub.addOption(11, "Sub item 11");
        visage::PopupMenu sub_sub("Sub sub menu");
        sub_sub.addOption(20, "Sub sub item 20");
        sub.addSubMenu(sub_sub);
        menu.addSubMenu(sub);
        menu.addOption(4, "Item with Unicode ✓ ünï");
        connect(menu);
        return menu;
      }

      visage::PopupMenu deepMenu(int level, int max_level) {
        visage::PopupMenu menu(level == 1 ? "" : "Level " + std::to_string(level));
        menu.addOption(level * 100, "Item at level " + std::to_string(level));
        if (level < max_level)
          menu.addSubMenu(deepMenu(level + 1, max_level));
        if (level == 1)
          connect(menu);
        return menu;
      }

      LogView log_;
      std::vector<std::unique_ptr<visage::UiButton>> buttons_;
      std::unique_ptr<visage::UiButton> corner_;
    };
  }

  void addWindowTests(std::vector<TestDefinition>& tests) {
    tests.push_back(
        { "windows",
          "Windows",
          "Secondary Windows",
          "Each button opens, hides or destroys a window. Window shows and hides are logged.",
          {
              "Show, move and hide the basic window a few times. It reopens where you left it, "
              "with "
              "the same size and title bar.",
              "The native close button hides a secondary window and the app keeps running.",
              "\"Re-show from a timer\" hides the basic window and brings it back after 1 second. "
              "\"5 times quickly\" ends with the window showing. Nothing hangs.",
              "The refuses-close window ignores its close button until \"Allow close\" is on, then "
              "closes.",
              "The size-limited window can't be resized below 300x200. With \"Fixed aspect ratio\" "
              "on, edges and corners keep the ratio.",
              "Dragging the bottom edge of any window keeps its top edge in place.",
              "Self-closing window: close(), hide() and \"Delete window\" all work from inside a "
              "mouse "
              "down with no crash. Reopen it and try each one.",
              "\"Destroy in 2 seconds\" closes the window cleanly while it's open.",
              "The always-on-top window stays above other apps' windows.",
              "Client-decorated window: dragging its title bar moves it. Minimize, "
              "maximize/restore "
              "and close all work, and maximize toggles back.",
              "\"At screen 100, 100\" opens near the top left of the main screen. Moving the main "
              "window to another monitor doesn't change that.",
              "New windows show their content on the very first frame, not blank or black.",
              "\"Open 5 windows\" then \"Close all\" closes them all without errors.",
          },
          [] { return std::make_unique<WindowsPage>(); },
          false });

    tests.push_back(
        { "dpi",
          "Windows",
          "DPI and Resizing",
          "Readouts and pixel patterns for checking scaling. You need two monitors with different "
          "scaling for some checks.",
          {
              "Dragging the window to a monitor with a different scale updates the DPI readout and "
              "keeps the layout the same logical size.",
              "Text stays sharp after the DPI change and the text editor's font keeps its size.",
              "Resize the window slowly. No green shows between the frames in either strip.",
              "The hairline pattern stays crisp and even with no blurry or doubled lines.",
              "Resizing quickly for several seconds causes no black margins, stretching, flicker "
              "or crash.",
              "Maximizing and restoring the window draws correctly at both sizes.",
          },
          [] { return std::make_unique<DpiPage>(); } });

    tests.push_back(
        { "lifecycle",
          "Windows",
          "Lifecycle and Menus",
          "The box at the top says whether the previous session exited cleanly. Quit the app in "
          "different ways, relaunch, and check what it says.",
          {
              "Quit with Cmd+Q (or Alt+F4). After relaunch the status says the last session exited "
              "cleanly.",
              "Mac: quit from the Dock menu. After relaunch the status says it exited cleanly.",
              "Close the main window with its close button. The app quits and the next launch says "
              "it exited cleanly.",
              "\"closeApplication()\" quits and the next launch says it exited cleanly.",
              "\"Force a crash\" crashes, and the next launch says it did NOT exit cleanly.",
              "The message box shows the quotes, tags, backslash and Unicode literally.",
              "Mac: \"Install test menu bar\" adds a Manual Test menu. Cmd+Shift+H logs hello, the "
              "disabled item can't be picked and the checked item has a check mark.",
              "Mac: choosing \"Replace the menu bar from here\" swaps the menu bar with no crash, "
              "and "
              "the replacement can restore the original.",
              "Open the popup and quit with Cmd+Q while it's open. The popup closes with the app.",
          },
          [] { return std::make_unique<LifecyclePage>(); },
          true,
          "Only the message box check applies. The page can't quit, and the menu bar and session "
          "checks "
          "don't exist in a browser. Check that the alert shows the quotes, tags and backslash "
          "literally "
          "instead of running them as JavaScript." });

    tests.push_back(
        { "popups",
          "Windows",
          "Popup Menus",
          "Popup menus with sub menus, long lists and different positions. Selections and cancels "
          "are logged.",
          {
              "Picking an item logs its id. Escape or clicking outside the menu logs Cancelled.",
              "Disabled items can't be picked and the selected item shows a check mark.",
              "All 8 sub menu levels open and an item at the deepest level logs id 800.",
              "The 150-item menu fits on screen and scrolls with the wheel. Hover highlights "
              "follow "
              "the mouse while scrolling.",
              "Scroll the long sub menu down, close everything, reopen it. It starts at the top "
              "again.",
              "Sub menus near the bottom of the screen stay inside the screen.",
              "Press a menu button, drag onto an item and release. Items highlight while you drag "
              "and "
              "the release picks the item.",
              "The bottom-right menu and the bottom point menu open fully on screen, flipping up "
              "or "
              "left as needed.",
          },
          [] { return std::make_unique<PopupPage>(); } });
  }
}
