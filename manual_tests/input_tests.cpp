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

#include <filesystem>
#include <visage_utils/file_system.h>

namespace manual {
  std::string keyCodeName(visage::KeyCode key_code) {
    using visage::KeyCode;
    switch (key_code) {
    case KeyCode::Unknown: return "Unknown";
    case KeyCode::Return: return "Return";
    case KeyCode::Escape: return "Escape";
    case KeyCode::Backspace: return "Backspace";
    case KeyCode::Tab: return "Tab";
    case KeyCode::Space: return "Space";
    case KeyCode::CapsLock: return "CapsLock";
    case KeyCode::Home: return "Home";
    case KeyCode::End: return "End";
    case KeyCode::PageUp: return "PageUp";
    case KeyCode::PageDown: return "PageDown";
    case KeyCode::Delete: return "Delete";
    case KeyCode::Insert: return "Insert";
    case KeyCode::Left: return "Left";
    case KeyCode::Right: return "Right";
    case KeyCode::Up: return "Up";
    case KeyCode::Down: return "Down";
    case KeyCode::NumLock: return "NumLock";
    case KeyCode::KPDivide: return "KP/";
    case KeyCode::KPMultiply: return "KP*";
    case KeyCode::KPMinus: return "KP-";
    case KeyCode::KPPlus: return "KP+";
    case KeyCode::KPEnter: return "KPEnter";
    case KeyCode::KPPeriod: return "KPPeriod";
    case KeyCode::KPEquals: return "KP=";
    case KeyCode::KPComma: return "KPComma";
    case KeyCode::KP0: return "KP0";
    case KeyCode::KP1: return "KP1";
    case KeyCode::KP2: return "KP2";
    case KeyCode::KP3: return "KP3";
    case KeyCode::KP4: return "KP4";
    case KeyCode::KP5: return "KP5";
    case KeyCode::KP6: return "KP6";
    case KeyCode::KP7: return "KP7";
    case KeyCode::KP8: return "KP8";
    case KeyCode::KP9: return "KP9";
    case KeyCode::NonUSBackslash: return "NonUSBackslash";
    case KeyCode::Application: return "Application";
    case KeyCode::Menu: return "Menu";
    case KeyCode::Help: return "Help";
    case KeyCode::Mute: return "Mute";
    case KeyCode::VolumeUp: return "VolumeUp";
    case KeyCode::VolumeDown: return "VolumeDown";
    case KeyCode::PrintScreen: return "PrintScreen";
    case KeyCode::ScrollLock: return "ScrollLock";
    case KeyCode::Pause: return "Pause";
    default: break;
    }

    int code = static_cast<int>(key_code);
    int f1 = static_cast<int>(KeyCode::F1);
    if (code >= f1 && code <= static_cast<int>(KeyCode::F12))
      return "F" + std::to_string(code - f1 + 1);
    int f13 = static_cast<int>(KeyCode::F13);
    if (code >= f13 && code <= static_cast<int>(KeyCode::F24))
      return "F" + std::to_string(code - f13 + 13);
    if (code > 32 && code < 127)
      return std::string(1, static_cast<char>(code));
    if (code & visage::kUnprintableKeycodeMask)
      return "Key#" + std::to_string(code & ~visage::kUnprintableKeycodeMask);
    return "Code#" + std::to_string(code);
  }

  namespace {
    std::string codepoints(const std::string& text) {
      std::u32string utf32 = visage::String(text).toUtf32();
      std::string result;
      for (char32_t c : utf32) {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "U+%04X ", static_cast<unsigned int>(c));
        result += buffer;
      }
      return result;
    }

    class TrackerFrame : public visage::Frame {
    public:
      TrackerFrame(std::string name, unsigned int color, LogView* log, bool* log_moves) :
          Frame(name), color_(color), log_(log), log_moves_(log_moves) { }

      void draw(visage::Canvas& canvas) override {
        unsigned int color = pressed_ ? 0xffffffff : color_;
        canvas.setColor(color);
        canvas.roundedRectangle(0, 0, width(), height(), 8);
        if (hovered_) {
          canvas.setColor(0xffffffff);
          canvas.roundedRectangleBorder(0, 0, width(), height(), 8, 3);
        }
        canvas.setColor(pressed_ ? 0xff000000 : 0xffffffff);
        std::string text = name();
        if (clicks_)
          text += "  clicks: " + std::to_string(clicks_);
        canvas.text(text, font(14), visage::Font::kTopLeft, 10, 8, width() - 20, 20);
      }

      void log(const std::string& type, const visage::MouseEvent& e, const std::string& extra = "") {
        log_->add(name() + " " + type + " " + buttonName(e.button_id) +
                  " held:" + buttonStateString(e.button_state) + " " + pointString(e.position) +
                  " " + modifierString(e.modifiers) + extra);
      }

      void mouseEnter(const visage::MouseEvent& e) override {
        hovered_ = true;
        log("ENTER", e);
        redraw();
      }
      void mouseExit(const visage::MouseEvent& e) override {
        hovered_ = false;
        log("EXIT", e);
        redraw();
      }
      void mouseDown(const visage::MouseEvent& e) override {
        pressed_ = true;
        clicks_ = e.repeatClickCount();
        log("DOWN", e, " count:" + std::to_string(e.repeatClickCount()));
        redraw();
      }
      void mouseUp(const visage::MouseEvent& e) override {
        pressed_ = e.button_state &
                   (visage::kMouseButtonLeft | visage::kMouseButtonRight | visage::kMouseButtonMiddle);
        log("UP", e, e.isCancelled() ? " CANCELLED" : "");
        redraw();
      }
      void mouseMove(const visage::MouseEvent& e) override {
        if (*log_moves_)
          log("move", e);
      }
      void mouseDrag(const visage::MouseEvent& e) override {
        if (*log_moves_)
          log("drag", e);
      }
      bool mouseWheel(const visage::MouseEvent& e) override {
        log_->add(name() + " WHEEL dx:" + visage::String(e.wheel_delta_x, 3).toUtf8() +
                  " dy:" + visage::String(e.wheel_delta_y, 3).toUtf8() +
                  " precise:" + visage::String(e.precise_wheel_delta_x, 2).toUtf8() + "," +
                  visage::String(e.precise_wheel_delta_y, 2).toUtf8() +
                  (e.hasWheelMomentum() ? " momentum" : "") + (e.wheel_reversed ? " reversed" : ""));
        return true;
      }

    private:
      unsigned int color_ = 0;
      LogView* log_ = nullptr;
      bool* log_moves_ = nullptr;
      bool hovered_ = false;
      bool pressed_ = false;
      int clicks_ = 0;
    };

    class RelativeModeFrame : public visage::Frame {
    public:
      explicit RelativeModeFrame(LogView* log) : log_(log) { }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(active_ ? 0xff6a4a9a : 0xff3e3550);
        canvas.roundedRectangle(0, 0, width(), height(), 8);
        canvas.setColor(0xffffffff);
        drawLines(canvas,
                  "Relative mode: hold any button and drag far.\nAccumulated: " +
                      visage::String(accumulated_.x, 0).toUtf8() + ", " +
                      visage::String(accumulated_.y, 0).toUtf8(),
                  font(13), visage::Font::kCenter, 0, 0, width(), height());
      }

      void mouseDown(const visage::MouseEvent& e) override {
        active_ = true;
        accumulated_ = {};
        setMouseRelativeMode(true);
        setCursorVisible(false);
        redraw();
      }
      void mouseDrag(const visage::MouseEvent& e) override {
        accumulated_ = accumulated_ + e.relative_position;
        redraw();
      }
      void mouseUp(const visage::MouseEvent& e) override {
        active_ = false;
        setCursorVisible(true);
        setMouseRelativeMode(false);
        log_->add("Relative mode ended, moved " + pointString(accumulated_));
        redraw();
      }

    private:
      LogView* log_ = nullptr;
      bool active_ = false;
      visage::Point accumulated_;
    };

    class MousePage : public visage::Frame {
    public:
      MousePage() :
          outer_("Outer", 0xff2f5d8a, &log_, &log_moves_),
          inner_("Inner", 0xff3c8a5a, &log_, &log_moves_),
          sibling_("Sibling", 0xff8a5a2f, &log_, &log_moves_), relative_(&log_) {
        addChild(&outer_);
        outer_.addChild(&inner_);
        addChild(&sibling_);
        addChild(&relative_);
        addChild(&log_);
        addChild(&readout_);
        readout_.setMono(true);

        log_moves_button_ = std::make_unique<visage::ToggleTextButton>("Log moves and drags");
        log_moves_button_->setFont(font(13));
        log_moves_button_->onToggle() = [this](visage::Button*, bool on) { log_moves_ = on; };
        addChild(log_moves_button_.get());
        clear_ = makeButton("Clear log", [this] { log_.clear(); });
        addChild(clear_.get());
      }

      void draw(visage::Canvas& canvas) override {
        visage::Point cursor = visage::cursorPosition();
        readout_.setText("visage::cursorPosition(): " + pointString(cursor) +
                         "   double-click speed: " + std::to_string(visage::doubleClickSpeed()) + "ms");
        redraw();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        readout_.setBounds(takeTop(bounds, 22));
        auto columns = splitColumns(bounds, { 1.1f, 1.0f });
        visage::Bounds left = columns[0];
        visage::Bounds controls = takeBottom(left, 34);
        layoutButtonRow(controls, { log_moves_button_.get(), clear_.get() });
        auto rows = splitRows(left, { 2.0f, 1.0f, 1.0f });
        outer_.setBounds(rows[0]);
        inner_.setBounds(outer_.width() * 0.3f, outer_.height() * 0.3f, outer_.width() * 0.5f,
                         outer_.height() * 0.5f);
        sibling_.setBounds(rows[1]);
        relative_.setBounds(rows[2]);
        log_.setBounds(columns[1]);
      }

    private:
      bool log_moves_ = false;
      LogView log_;
      Label readout_ { "", 13, kDimText };
      TrackerFrame outer_;
      TrackerFrame inner_;
      TrackerFrame sibling_;
      RelativeModeFrame relative_;
      std::unique_ptr<visage::ToggleTextButton> log_moves_button_;
      std::unique_ptr<visage::UiButton> clear_;
    };

    class KeyLogFrame : public visage::Frame {
    public:
      explicit KeyLogFrame(LogView* log) : log_(log) { setAcceptsKeystrokes(true); }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(hasKeyboardFocus() ? 0xff2f5d8a : kPanelLight);
        canvas.roundedRectangle(0, 0, width(), height(), 8);
        canvas.setColor(0xffffffff);
        canvas.text(hasKeyboardFocus() ? "Focused: type anything" : "Click here to focus", font(18),
                    visage::Font::kCenter, 0, 0, width(), height());
      }

      void mouseDown(const visage::MouseEvent& e) override { requestKeyboardFocus(); }
      void focusChanged(bool is_focused, bool was_clicked) override {
        log_->add(std::string("FOCUS ") + (is_focused ? "gained" : "lost") +
                  (was_clicked ? " (click)" : ""));
        redraw();
      }

      bool keyPress(const visage::KeyEvent& e) override {
        log_->add("PRESS   " + modifierString(e.modifierMask()) + keyCodeName(e.keyCode()) +
                  (e.isRepeat() ? "  (repeat)" : ""));
        return consume_;
      }

      bool keyRelease(const visage::KeyEvent& e) override {
        log_->add("RELEASE " + modifierString(e.modifierMask()) + keyCodeName(e.keyCode()));
        return consume_;
      }

      bool receivesTextInput() override { return text_input_; }
      void textInput(const std::string& text) override {
        log_->add("TEXT    \"" + text + "\"  " + codepoints(text));
      }

      void setConsume(bool consume) { consume_ = consume; }
      void setTextInput(bool text_input) { text_input_ = text_input; }

    private:
      LogView* log_ = nullptr;
      bool consume_ = true;
      bool text_input_ = true;
    };

    class KeyboardPage : public visage::Frame {
    public:
      KeyboardPage() : key_frame_(&log_) {
        addChild(&key_frame_);
        addChild(&log_);
        consume_ = std::make_unique<visage::ToggleTextButton>("Consume key presses");
        consume_->setFont(font(13));
        consume_->setToggled(true);
        consume_->onToggle() = [this](visage::Button*, bool on) { key_frame_.setConsume(on); };
        addChild(consume_.get());
        text_input_ = std::make_unique<visage::ToggleTextButton>("Receive text input");
        text_input_->setFont(font(13));
        text_input_->setToggled(true);
        text_input_->onToggle() = [this](visage::Button*, bool on) { key_frame_.setTextInput(on); };
        addChild(text_input_.get());
        clear_ = makeButton("Clear log", [this] { log_.clear(); });
        addChild(clear_.get());
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        key_frame_.setBounds(takeTop(bounds, 120));
        layoutButtonRow(takeTop(bounds, 34), { consume_.get(), text_input_.get(), clear_.get() });
        log_.setBounds(bounds);
      }

    private:
      LogView log_;
      KeyLogFrame key_frame_;
      std::unique_ptr<visage::ToggleTextButton> consume_;
      std::unique_ptr<visage::ToggleTextButton> text_input_;
      std::unique_ptr<visage::UiButton> clear_;
    };

    class EditorBox : public Box {
    public:
      EditorBox(const std::string& title, visage::TextEditor* editor) :
          Box(title), editor_(editor) {
        if (editor_)
          addChild(editor_);
      }

      void resized() override {
        if (editor_)
          editor_->setBounds(contentBounds());
      }

      void setEditor(visage::TextEditor* editor) {
        editor_ = editor;
        if (editor_) {
          addChild(editor_);
          editor_->setBounds(contentBounds());
        }
      }

    private:
      visage::TextEditor* editor_ = nullptr;
    };

    class TextEditingPage : public visage::Frame,
                            public visage::UndoHistory {
    public:
      TextEditingPage() :
          single_box_("Single line", &single_), multi_box_("Multi-line, wrapped", &multi_),
          password_box_("Password", &password_),
          number_box_("Number entry: letters and symbols filtered, max 8", &number_),
          inactive_box_("Inactive (setActive(false))", &inactive_),
          delete_box_("Type \"delete\" to delete this editor", nullptr),
          enter_box_("Press Enter to delete this editor", nullptr) {
        setAcceptsKeystrokes(true);
        for (visage::TextEditor* editor : { &single_, &multi_, &password_, &number_, &inactive_ })
          editor->setFont(font(16));

        single_.setDefaultText("Type here");
        single_.setJustification(visage::Font::kLeft);
        multi_.setMultiLine(true);
        multi_.setJustification(visage::Font::kTopLeft);
        multi_.setText("This paragraph wraps across lines so you can test End, Home and clicking "
                       "past the end "
                       "of a wrapped line. Words for navigation: naïve café   Привет   日本語   "
                       "snake_case_word   multiple    spaces    here.\n\nSecond paragraph after a "
                       "blank line.");
        password_.setPassword();
        password_.setDefaultText("Secret");
        number_.setNumberEntry();
        number_.setFilteredCharacters("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                      " !@#$%^&*()_+=[]{};:'\",<>/?\\|`~");
        number_.setMaxCharacters(8);
        number_.setText("42");
        inactive_.setText("You should not be able to change this");
        inactive_.setActive(false);

        for (Box* box : { (Box*)&single_box_, (Box*)&multi_box_, (Box*)&password_box_, (Box*)&number_box_,
                          (Box*)&inactive_box_, (Box*)&delete_box_, (Box*)&enter_box_ })
          addChild(box);

        createDeletableEditors();

        copy_multiline_ = makeButton("Copy 3-line text to clipboard", [this] {
          visage::setClipboardText("first line\nsecond line\nthird line");
          status_.setText("Clipboard now holds three lines of text.");
        });
        recreate_ = makeButton("Recreate deleted editors", [this] { createDeletableEditors(); });
        toggle_ = std::make_unique<visage::ToggleTextButton>("Undoable toggle");
        toggle_->setFont(font(13));
        addChild(copy_multiline_.get());
        addChild(recreate_.get());
        addChild(toggle_.get());
        addChild(&status_);
        status_.setMultiLine(true);
      }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(0x00000000);
        canvas.fill(0, 0, width(), height());
      }

      bool keyPress(const visage::KeyEvent& key) override {
        if (key.isMainModifier() && key.keyCode() == visage::KeyCode::Z) {
          if (key.isShiftDown())
            redo();
          else
            undo();
          status_.setText(std::string("Page undo history handled the key. Toggle is now ") +
                          (toggle_->toggled() ? "on." : "off."));
          return true;
        }
        return false;
      }

      void mouseDown(const visage::MouseEvent& e) override { requestKeyboardFocus(); }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        status_.setBounds(takeBottom(bounds, 40));
        layoutButtonRow(takeBottom(bounds, 34), { copy_multiline_.get(), recreate_.get(), toggle_.get() });
        auto columns = evenColumns(bounds, 2);
        auto left = splitRows(columns[0], { 1.0f, 3.0f, 1.0f });
        single_box_.setBounds(left[0]);
        multi_box_.setBounds(left[1]);
        password_box_.setBounds(left[2]);
        auto right = evenRows(columns[1], 4);
        number_box_.setBounds(right[0]);
        inactive_box_.setBounds(right[1]);
        delete_box_.setBounds(right[2]);
        enter_box_.setBounds(right[3]);
      }

    private:
      void createDeletableEditors() {
        if (!delete_editor_) {
          delete_editor_ = std::make_unique<visage::TextEditor>();
          delete_editor_->setFont(font(16));
          delete_editor_->setDefaultText("delete");
          delete_editor_->onTextChange() = [this] {
            if (delete_editor_->text().toUtf8().find("delete") != std::string::npos) {
              delete_editor_ = nullptr;
              status_.setText("Deleted the editor from inside its own onTextChange.");
            }
          };
          delete_box_.setEditor(delete_editor_.get());
        }
        if (!enter_editor_) {
          enter_editor_ = std::make_unique<visage::TextEditor>();
          enter_editor_->setFont(font(16));
          enter_editor_->setDefaultText("Press Enter");
          enter_editor_->onEnterKey() = [this] {
            enter_editor_ = nullptr;
            status_.setText("Deleted the editor from inside its own onEnterKey.");
          };
          enter_box_.setEditor(enter_editor_.get());
        }
      }

      visage::TextEditor single_;
      visage::TextEditor multi_;
      visage::TextEditor password_;
      visage::TextEditor number_;
      visage::TextEditor inactive_;
      std::unique_ptr<visage::TextEditor> delete_editor_;
      std::unique_ptr<visage::TextEditor> enter_editor_;
      EditorBox single_box_;
      EditorBox multi_box_;
      EditorBox password_box_;
      EditorBox number_box_;
      EditorBox inactive_box_;
      EditorBox delete_box_;
      EditorBox enter_box_;
      std::unique_ptr<visage::UiButton> copy_multiline_;
      std::unique_ptr<visage::UiButton> recreate_;
      std::unique_ptr<visage::ToggleTextButton> toggle_;
      Label status_ { "", 13, kDimText };
    };

    class InternationalPage : public visage::Frame {
    public:
      InternationalPage() :
          editor_box_("Text editor", &editor_), sample_box_("Rendered sample text") {
        editor_.setFont(font(22));
        editor_.setMultiLine(true);
        editor_.setJustification(visage::Font::kTopLeft);
        editor_.setDefaultText("Type accents, IME text and emoji here");
        addChild(&editor_box_);
        addChild(&sample_box_);
        addChild(&log_);
        editor_.onTextChange() = [this] {
          std::string text = editor_.text().toUtf8();
          std::string tail = text.size() > 12 ? text.substr(text.size() - 12) : text;
          log_.add("text changed, length " + std::to_string(editor_.textLength()) +
                   " chars, tail: " + codepoints(visage::String(tail).toUtf8()));
        };
        sample_box_.onDraw() += [this](visage::Canvas& canvas) {
          visage::Bounds content = sample_box_.contentBounds();
          static const char* kLines[] = {
            "Emoji: 😀 🎉 👍🏽 🇺🇸 ❤️ vs ❤ (no extra gap after ❤️)",
            "Latin: àéîõü ñ ç ß ø å æ œ — “quotes” ‘single’ …",
            "Cyrillic: Привет мир   Greek: Γειά σου κόσμε",
            "Combining: e\xCC\x81 a\xCC\x8A n\xCC\x83 (should look like é å ñ)",
          };
          float line = content.height() / 4.0f;
          canvas.setColor(kText);
          for (int i = 0; i < 4; ++i) {
            canvas.text(kLines[i], font(18), visage::Font::kLeft, content.x(),
                        content.y() + i * line, content.width(), line);
          }
        };
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        auto rows = splitRows(bounds, { 2.0f, 1.4f, 1.6f });
        editor_box_.setBounds(rows[0]);
        sample_box_.setBounds(rows[1]);
        log_.setBounds(rows[2]);
      }

    private:
      visage::TextEditor editor_;
      EditorBox editor_box_;
      Box sample_box_;
      LogView log_;
    };

    class CursorZone : public visage::Frame {
    public:
      CursorZone(std::string name, visage::MouseCursor cursor) : Frame(name), cursor_(cursor) { }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(hover_ ? 0xff3c4c66 : kPanelLight);
        canvas.roundedRectangle(0, 0, width(), height(), 6);
        canvas.setColor(kText);
        canvas.text(name(), font(13), visage::Font::kCenter, 0, 0, width(), height());
      }

      void mouseEnter(const visage::MouseEvent& e) override {
        hover_ = true;
        setCursorStyle(cursor_);
        redraw();
      }
      void mouseExit(const visage::MouseEvent& e) override {
        hover_ = false;
        setCursorStyle(visage::MouseCursor::Arrow);
        redraw();
      }

    private:
      visage::MouseCursor cursor_;
      bool hover_ = false;
    };

    class CursorPage : public visage::Frame {
    public:
      CursorPage() {
        static const std::pair<const char*, visage::MouseCursor> kCursors[] = {
          { "Arrow", visage::MouseCursor::Arrow },
          { "IBeam", visage::MouseCursor::IBeam },
          { "Crosshair", visage::MouseCursor::Crosshair },
          { "Pointing", visage::MouseCursor::Pointing },
          { "Dragging", visage::MouseCursor::Dragging },
          { "HorizontalResize", visage::MouseCursor::HorizontalResize },
          { "VerticalResize", visage::MouseCursor::VerticalResize },
          { "TopLeftResize", visage::MouseCursor::TopLeftResize },
          { "TopRightResize", visage::MouseCursor::TopRightResize },
          { "BottomLeftResize", visage::MouseCursor::BottomLeftResize },
          { "BottomRightResize", visage::MouseCursor::BottomRightResize },
          { "MultiDirectionalResize", visage::MouseCursor::MultiDirectionalResize },
          { "Invisible", visage::MouseCursor::Invisible },
        };
        for (const auto& [name, cursor] : kCursors) {
          zones_.push_back(std::make_unique<CursorZone>(name, cursor));
          addChild(zones_.back().get());
        }

        hide_ = makeButton("Hide cursor for 5 seconds", [this] {
          setCursorVisible(false);
          hide_timer_.startTimer(5000);
        });
        hide_timer_.onTimerCallback() = [this] {
          hide_timer_.stopTimer();
          setCursorVisible(true);
        };
        warp_ = makeButton("Warp cursor to target in 1 second",
                           [this] { warp_timer_.startTimer(1000); });
        warp_timer_.onTimerCallback() = [this] {
          warp_timer_.stopTimer();
          visage::setCursorPosition(target_.positionInWindow() +
                                    visage::Point(target_.width() * 0.5f, target_.height() * 0.5f));
        };
        addChild(hide_.get());
        addChild(warp_.get());

        target_.setIgnoresMouseEvents(true, false);
        target_.onDraw() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xff3a2a2a);
          canvas.roundedRectangle(0, 0, target_.width(), target_.height(), 6);
          canvas.setColor(kFail);
          float cx = target_.width() * 0.5f;
          float cy = target_.height() * 0.5f;
          canvas.ring(cx - 20, cy - 20, 40, 2);
          canvas.rectangle(cx - 0.5f, cy - 30, 1, 60);
          canvas.rectangle(cx - 30, cy - 0.5f, 60, 1);
        };
        addChild(&target_);
        addChild(&readout_);
        readout_.setMono(true);
        readout_.setMultiLine(true);
      }

      void draw(visage::Canvas& canvas) override {
        visage::Point cursor = visage::cursorPosition();
        readout_.setText("visage::cursorPosition() (window coords): " + pointString(cursor) +
                         "\nlast mouse event, in window coords:     " + pointString(last_window_position_));
        redraw();
      }

      void mouseMove(const visage::MouseEvent& e) override {
        last_window_position_ = e.windowPosition();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        readout_.setBounds(takeTop(bounds, 40));
        layoutButtonRow(takeTop(bounds, 34), { hide_.get(), warp_.get() });
        target_.setBounds(takeBottom(bounds, 140));
        auto cells = grid(bounds, 4, 4);
        for (int i = 0; i < zones_.size(); ++i)
          zones_[i]->setBounds(cells[i]);
      }

    private:
      std::vector<std::unique_ptr<CursorZone>> zones_;
      std::unique_ptr<visage::UiButton> hide_;
      std::unique_ptr<visage::UiButton> warp_;
      visage::EventTimer hide_timer_;
      visage::EventTimer warp_timer_;
      visage::Frame target_;
      Label readout_ { "", 13, kDimText };
      visage::Point last_window_position_;
    };

    class ClipboardPage : public visage::Frame {
    public:
      ClipboardPage() : editor_box_("Editor: copy, cut and paste here", &editor_) {
        editor_.setFont(font(16));
        editor_.setMultiLine(true);
        editor_.setJustification(visage::Font::kTopLeft);
        addChild(&editor_box_);
        addChild(&log_);

        copy_unicode_ = makeButton("Copy Unicode text", [this] {
          std::string text = "Visage clipboard ✓ ünïcødé Привет 日本語 🎉";
          visage::setClipboardText(text);
          log_.add("Set clipboard to: " + text);
        });
        copy_large_ = makeButton("Copy 1 MB of text", [this] {
          std::string text;
          text.reserve(1024 * 1024);
          for (int i = 0; text.size() < 1024 * 1024 - 64; ++i)
            text += "line " + std::to_string(i) + " of the large clipboard test\n";
          text += "END";
          visage::setClipboardText(text);
          log_.add("Set clipboard to " + std::to_string(text.size()) + " bytes ending in END");
        });
        read_ = makeButton("Read clipboard", [this] {
          std::string text = visage::readClipboardText();
          std::string preview = text.size() > 80 ? text.substr(0, 80) + "..." : text;
          for (char& c : preview) {
            if (c == '\n')
              c = ' ';
          }
          log_.add("Read " + std::to_string(text.size()) + " bytes: \"" + preview + "\"" +
                   (text.size() > 80 ? " ends with \"" + text.substr(text.size() - 3) + "\"" : ""));
        });
        clear_ = makeButton("Copy empty string", [this] {
          visage::setClipboardText("");
          log_.add("Set clipboard to an empty string");
        });
        for (auto* button : { copy_unicode_.get(), copy_large_.get(), read_.get(), clear_.get() })
          addChild(button);
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        layoutButtonRow(takeTop(bounds, 34),
                        { copy_unicode_.get(), copy_large_.get(), read_.get(), clear_.get() });
        auto rows = evenRows(bounds, 2);
        editor_box_.setBounds(rows[0]);
        log_.setBounds(rows[1]);
      }

    private:
      visage::TextEditor editor_;
      EditorBox editor_box_;
      LogView log_;
      std::unique_ptr<visage::UiButton> copy_unicode_;
      std::unique_ptr<visage::UiButton> copy_large_;
      std::unique_ptr<visage::UiButton> read_;
      std::unique_ptr<visage::UiButton> clear_;
    };

    class DragSource : public visage::Frame {
    public:
      DragSource(std::string name, std::string file_name, LogView* log) :
          Frame(std::move(name)), file_name_(std::move(file_name)), log_(log) { }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(dragging_ ? 0xff2f5d8a : 0xff2a4a3a);
        canvas.roundedRectangle(0, 0, width(), height(), 8);
        canvas.setColor(0xffffffff);
        std::string text = dragging_ ? "Dragging \"" + file_name_ + "\"" : name();
        text += "\ncleanups: " + std::to_string(cleanups_);
        drawLines(canvas, text, font(14), visage::Font::kCenter, 0, 0, width(), height());
      }

      bool isDragDropSource() override { return true; }

      std::string startDragDropSource() override {
        dragging_ = true;
        visage::File directory = std::filesystem::temp_directory_path() / "visage_manual_drag";
        visage::createDirectory(directory);
        file_ = directory / pathFromUtf8(file_name_);
        visage::replaceFileWithText(file_, "Visage manual test drag file: " + file_name_ + "\n");
        log_->add(name() + ": startDragDropSource -> " + pathString(file_));
        redraw();
        return pathString(file_);
      }

      void cleanupDragDropSource() override {
        dragging_ = false;
        ++cleanups_;
        std::error_code error;
        std::filesystem::remove(file_, error);
        log_->add(name() + ": cleanupDragDropSource");
        redraw();
      }

      void mouseUp(const visage::MouseEvent& e) override { log_->add(name() + ": mouse up"); }

    private:
      std::string file_name_;
      LogView* log_ = nullptr;
      visage::File file_;
      bool dragging_ = false;
      int cleanups_ = 0;
    };

    class DropTarget : public visage::Frame {
    public:
      DropTarget(std::string name, std::string regex, bool multiple, LogView* log) :
          Frame(std::move(name)), regex_(std::move(regex)), multiple_(multiple), log_(log) { }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(hovering_ ? 0xff2f5d8a : kPanelLight);
        canvas.roundedRectangle(0, 0, width(), height(), 8);
        if (hovering_) {
          canvas.setColor(0xffffffff);
          canvas.roundedRectangleBorder(0, 0, width(), height(), 8, 3);
        }
        canvas.setColor(kText);
        std::string text = name() + "\nregex: " + regex_ + (multiple_ ? "  (multiple)" : "  (single)");
        if (!last_.empty())
          text += "\n" + last_;
        drawLines(canvas, text, font(13), visage::Font::kTop, 6, 8, width() - 12, height() - 16);
      }

      bool receivesDragDropFiles() override { return true; }
      bool receivesMultipleDragDropFiles() override { return multiple_; }
      std::string dragDropFileExtensionRegex() override { return regex_; }

      void dragFilesEnter(const std::vector<std::string>& paths) override {
        hovering_ = true;
        log_->add(name() + ": drag enter, " + std::to_string(paths.size()) + " file(s)");
        redraw();
      }

      void dragFilesExit() override {
        hovering_ = false;
        log_->add(name() + ": drag exit");
        redraw();
      }

      void dropFiles(const std::vector<std::string>& paths) override {
        hovering_ = false;
        last_ = "Dropped " + std::to_string(paths.size()) + ":";
        for (const auto& path : paths) {
          last_ += "\n" + visage::fileName(path);
          log_->add(name() + ": dropped " + path);
        }
        redraw();
      }

    private:
      std::string regex_;
      bool multiple_ = false;
      LogView* log_ = nullptr;
      bool hovering_ = false;
      std::string last_;
    };

    class DragDropPage : public visage::Frame {
    public:
      DragDropPage() :
          source_("Drag me to Finder or a target", "visage-drag.txt", &log_),
          unicode_source_("Drag me: Unicode file name", "visage drag ünïcødé файл.txt", &log_),
          any_("Any single file", ".*", false, &log_), multiple_("Multiple files", ".*", true, &log_),
          text_only_("Only .txt files", "^\\.txt$", false, &log_),
          images_("Only .png/.jpg (nested in 'Any single file')", "^\\.(png|jpg|jpeg)$", false, &log_),
          invalid_("Invalid regex, never accepts", "(txt", false, &log_) {
        addChild(&source_);
        addChild(&unicode_source_);
        addChild(&any_);
        any_.addChild(&images_);
        addChild(&multiple_);
        addChild(&text_only_);
        addChild(&invalid_);
        addChild(&log_);
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        auto rows = splitRows(bounds, { 1.0f, 3.0f, 2.0f });
        auto sources = evenColumns(rows[0], 2);
        source_.setBounds(sources[0]);
        unicode_source_.setBounds(sources[1]);
        auto targets = evenColumns(rows[1], 2);
        any_.setBounds(targets[0]);
        images_.setBounds(20, any_.height() * 0.45f, any_.width() - 40, any_.height() * 0.5f);
        auto right = evenRows(targets[1], 3);
        multiple_.setBounds(right[0]);
        text_only_.setBounds(right[1]);
        invalid_.setBounds(right[2]);
        log_.setBounds(rows[2]);
      }

    private:
      LogView log_;
      DragSource source_;
      DragSource unicode_source_;
      DropTarget any_;
      DropTarget multiple_;
      DropTarget text_only_;
      DropTarget images_;
      DropTarget invalid_;
    };
  }

  void addInputTests(std::vector<TestDefinition>& tests) {
    tests.push_back(
        { "mouse",
          "Input",
          "Mouse Events",
          "Outer contains Inner. Sibling is separate. Every event is logged on the right. Turn on "
          "\"Log moves and drags\" when you need positions.",
          {
              "Moving Outer > Inner > Outer > Sibling logs exactly one EXIT per ENTER, in order, "
              "and only one frame shows the hover border at a time.",
              "Press in Inner, drag outside the window and release. Drags keep coming outside the "
              "window, UP goes to Inner, then the frame under the mouse gets ENTER with a position "
              "relative to itself.",
              "Double and triple clicks count 2 and 3. A left click then a quick right click does "
              "not count as a double click. A tiny move between clicks still counts.",
              "Hold left in Inner, move over Sibling, press and release right, then release left. "
              "Every DOWN has a matching UP and all of them go to Inner.",
              "Hold the button in a frame and Cmd+Tab / Alt+Tab away. The frame does not stay "
              "pressed (white) when you come back, and nothing clicks.",
              "With another app focused, click once on Sibling. The click reaches Sibling (DOWN "
              "and "
              "UP are logged) instead of only focusing the window.",
              "Move the mouse out of the window quickly. The hovered frame gets EXIT with a "
              "position "
              "near the window edge.",
              "Scroll wheel and trackpad log WHEEL. Trackpad deltas are fractional and a flick "
              "shows "
              "momentum.",
              "Relative mode: holding a button in the purple box hides the cursor and lets you "
              "drag "
              "endlessly. Releasing shows the cursor where you started.",
              "cursorPosition() at the top tracks the mouse in window coordinates (0,0 is the top "
              "left of the content area).",
          },
          [] { return std::make_unique<MousePage>(); } });

    tests.push_back({ "keyboard",
                      "Input",
                      "Keyboard Events",
                      "Click the box to focus it, then type. Every press, release, focus change "
                      "and text input "
                      "is logged.",
                      {
                          "A letter logs PRESS, then TEXT, then RELEASE, in that order.",
                          "Shift, Ctrl, Option/Alt and Cmd alone each log a PRESS and a RELEASE.",
                          "Holding a key logs repeat presses marked (repeat), then one RELEASE. "
                          "Holding Shift+key "
                          "repeats the same way.",
                          "Arrows, Home, End, PageUp, PageDown, Escape, Tab, Backspace and Delete "
                          "log their names "
                          "and macOS does not beep.",
                          "Numpad digits log KP0 to KP9, the numpad decimal logs KPPeriod and "
                          "numpad Enter logs "
                          "KPEnter, with NumLock on and off.",
                          "Cmd+digit and Ctrl+digit shortcuts log a PRESS but no TEXT.",
                          "Mac: Option+E then E logs TEXT \"é\" (U+00E9). Windows and Linux: "
                          "Alt+letter logs no TEXT.",
                          "With \"Consume key presses\" off, unhandled keys don't arrive twice and "
                          "the log still "
                          "pairs presses and releases.",
                          "Clicking elsewhere logs FOCUS lost. Switching apps and coming back logs "
                          "focus changes.",
                      },
                      [] { return std::make_unique<KeyboardPage>(); } });

    tests.push_back(
        { "text_editing",
          "Input",
          "Text Editing",
          "Several text editors with different settings. The page has its own undo history for "
          "the toggle button.",
          {
              "Typing, Shift+arrow selection, double-click word selection, triple-click line "
              "selection "
              "and Cmd/Ctrl+A all work in the single-line editor.",
              "Click \"Copy 3-line text\" and paste into the single-line editor. No newlines are "
              "inserted and Enter doesn't add one.",
              "Multi-line: on a wrapped line, End (or Cmd+Right) puts the caret at the end of that "
              "visual line, not the start of the next one. Clicking past the end of a wrapped line "
              "does the same.",
              "Option+Left/Right (Ctrl on Windows/Linux) moves by word across naïve, café, Привет "
              "and 日本語. Runs of spaces are crossed in one jump. Option+Backspace deletes a "
              "word.",
              "Undo (Cmd+Z) and redo (Cmd+Shift+Z) restore typed text. Backspace in an empty "
              "editor "
              "adds no undo step and doesn't clear redo.",
              "Click \"Undoable toggle\", then click an empty part of the page and press Cmd+Z / "
              "Cmd+Shift+Z. The toggle undoes and redoes.",
              "Focus an editor with nothing to redo and press Cmd+Shift+Z. The page toggle does "
              "not "
              "change.",
              "The inactive editor can't be typed into, pasted into, deleted from or undone.",
              "Typing \"delete\" removes that editor and pressing Enter removes the other one, "
              "with no "
              "crash. \"Recreate\" brings them back and they work again.",
              "The password editor shows dots. The number editor accepts digits, '.' and '-', "
              "drops "
              "letters and symbols (typed or pasted), stops at 8 characters and selects all on "
              "focus.",
              "Tab and Shift+Tab move focus between editors.",
          },
          [] { return std::make_unique<TextEditingPage>(); } });

    tests.push_back(
        { "international",
          "Input",
          "IME, Accents and Emoji",
          "Use the system input methods in the editor. The log shows the code points of the end of "
          "the text after each change.",
          {
              "Mac: press and hold E, then pick an accent from the popup. The accented letter "
              "replaces "
              "the plain one.",
              "Dead keys: Option+U then U gives ü, Option+N then N gives ñ. A dead key followed by "
              "a "
              "letter that can't take the accent inserts both characters, with nothing corrupted.",
              "Switch to a Japanese or Chinese input source. The composition is drawn while you "
              "type "
              "and committing inserts the right characters once.",
              "Insert emoji with the emoji picker (Ctrl+Cmd+Space or Win+.). They render in color "
              "with "
              "no dark ring around them.",
              "In the sample text, ❤️ (with variation selector) has no extra gap after it compared "
              "to ❤.",
              "Text edges in the sample look smooth and not too dark or bold.",
          },
          [] { return std::make_unique<InternationalPage>(); } });

    tests.push_back({ "cursor",
                      "Input",
                      "Mouse Cursor",
                      "Hover each zone to set its cursor style. Two buttons test hiding and "
                      "warping the cursor.",
                      {
                          "Each zone shows the named cursor, and Invisible hides it.",
                          "Leaving a zone restores the arrow.",
                          "Click \"Hide cursor\", then move across several zones. The cursor stays "
                          "hidden for 5 "
                          "seconds even though the style keeps changing, then reappears.",
                          "Click \"Warp cursor\". After 1 second the cursor jumps to the center of "
                          "the red target. "
                          "Repeat with the window on another monitor.",
                          "The two readouts at the top agree while the mouse is in the window.",
                          "Hover Crosshair, switch to another app, then click into the window on "
                          "the Crosshair "
                          "zone. The cursor stays a crosshair after the click.",
                      },
                      [] { return std::make_unique<CursorPage>(); },
                      true,
                      "Browsers can't hide or move the cursor from here, so skip the Hide cursor "
                      "and Warp cursor "
                      "checks. Cursor styles still apply." });

    tests.push_back(
        { "clipboard",
          "Input",
          "Clipboard",
          "Buttons write to and read from the system clipboard. The editor uses the normal "
          "copy, cut and paste shortcuts.",
          {
              "\"Copy Unicode text\" then pasting into another app gives exactly the same text, "
              "emoji "
              "included.",
              "Copying Unicode text in another app, then \"Read clipboard\", logs the same text.",
              "\"Copy 1 MB\" then \"Read clipboard\" logs about 1048000 bytes ending in END. "
              "Pasting it "
              "into the editor works.",
              "Copy and cut from the editor paste correctly into other apps, and paste from other "
              "apps "
              "works in the editor.",
              "Reading right after \"Copy empty string\" returns 0 bytes with no error.",
          },
          [] { return std::make_unique<ClipboardPage>(); } });

    tests.push_back(
        { "drag_drop",
          "Input",
          "File Drag and Drop",
          "The two green boxes are drag sources that create a temporary file. The other boxes "
          "accept "
          "dropped files that match their extension regex.",
          {
              "Dragging \"Drag me\" to Finder or Explorer creates visage-drag.txt with its text. "
              "When "
              "the drag ends the source logs a cleanup and goes back to idle.",
              "Dragging the Unicode source to Finder keeps the name \"visage drag ünïcødé "
              "файл.txt\".",
              "After dragging a source out, the source isn't stuck pressed: hover and clicks work "
              "normally.",
              "Dragging one file from the desktop over \"Any single file\" highlights it, leaving "
              "removes the highlight, and dropping shows the file name.",
              "Several files: \"Multiple files\" accepts them and lists every name. \"Any single "
              "file\" does not highlight.",
              "A .txt file is accepted by \"Only .txt\". A .png is accepted by the nested images "
              "box. "
              "A .txt over the images box falls through to its parent \"Any single file\".",
              "The invalid regex box never highlights and nothing crashes.",
              "Drag a source over a target, then back over the source. The target's highlight goes "
              "away.",
              "Dropped files with spaces and Unicode in their names show the right names.",
          },
          [] { return std::make_unique<DragDropPage>(); },
          false });
  }
}
