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

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <map>
#include <sstream>
#include <visage_utils/file_system.h>

#if VISAGE_EMSCRIPTEN
#include <emscripten.h>
#endif

using namespace visage::dimension;

namespace manual {
  namespace {
    std::string g_last_session_status = "This is the first recorded session.";

    visage::File resultsDirectory() {
#if VISAGE_EMSCRIPTEN
      return visage::File("/VisageManualTests");
#else
      visage::File directory = visage::appDataDirectory() / "VisageManualTests";
      visage::createDirectory(directory);
      return directory;
#endif
    }

#if VISAGE_EMSCRIPTEN
    void mountWebStorage() {
      EM_ASM({
        var directory = '/VisageManualTests';
        try {
          FS.mkdir(directory);
        }
        catch (e) {
        }
        FS.mount(IDBFS, {}, directory);
        Module.manualStorageReady = false;
        Module.manualSyncing = false;
        Module.manualSyncDirty = false;
        Module.manualSync = function() {
          if (Module.manualSyncing) {
            Module.manualSyncDirty = true;
            return;
          }
          Module.manualSyncing = true;
          FS.syncfs(
              false, function(error) {
                Module.manualSyncing = false;
                if (Module.manualSyncDirty) {
                  Module.manualSyncDirty = false;
                  Module.manualSync();
                }
              });
        };
        FS.syncfs(true, function(error) { Module.manualStorageReady = true; });
      });
    }

    bool webStorageReady() {
      return EM_ASM_INT({ return Module.manualStorageReady ? 1 : 0; });
    }
#endif

    void writeResultFile(const visage::File& file, const std::string& text) {
      visage::replaceFileWithText(file, text);
#if VISAGE_EMSCRIPTEN
      EM_ASM({
        if (Module.manualStorageReady)
          Module.manualSync();
      });
#endif
    }

    std::string escape(const std::string& text) {
      std::string result;
      for (char c : text) {
        if (c == '\n')
          result += "\\n";
        else if (c == '\t')
          result += "\\t";
        else if (c == '\\')
          result += "\\\\";
        else
          result += c;
      }
      return result;
    }

    std::string unescape(const std::string& text) {
      std::string result;
      for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\\' && i + 1 < text.size()) {
          char next = text[++i];
          if (next == 'n')
            result += '\n';
          else if (next == 't')
            result += '\t';
          else
            result += next;
        }
        else
          result += text[i];
      }
      return result;
    }

    std::string timestamp() {
      std::time_t now = std::time(nullptr);
      std::tm local {};
#if defined(_MSC_VER)
      localtime_s(&local, &now);
#elif VISAGE_WINDOWS
      local = *std::localtime(&now);
#else
      localtime_r(&now, &local);
#endif
      std::ostringstream stream;
      stream << std::put_time(&local, "%Y-%m-%d %H:%M:%S");
      return stream.str();
    }
  }

  std::string lastSessionStatus() {
    return g_last_session_status;
  }

  enum class Status {
    Untested,
    Pass,
    Fail,
    Skip
  };

  static const char* statusName(Status status) {
    switch (status) {
    case Status::Pass: return "pass";
    case Status::Fail: return "fail";
    case Status::Skip: return "skip";
    default: return "untested";
    }
  }

  static unsigned int statusColor(Status status) {
    switch (status) {
    case Status::Pass: return kPass;
    case Status::Fail: return kFail;
    case Status::Skip: return kSkip;
    default: return kPanelLight;
    }
  }

  class Results {
  public:
    Results() : path_(resultsDirectory() / "results.txt") { load(); }

    void load() {
      if (!visage::fileExists(path_))
        return;

      std::istringstream stream(visage::loadFileAsString(path_));
      std::string line;
      while (std::getline(stream, line)) {
        size_t tab = line.find('\t');
        if (tab == std::string::npos)
          continue;
        std::string key = line.substr(0, tab);
        std::string value = line.substr(tab + 1);
        if (key.rfind("note:", 0) == 0)
          notes_[key.substr(5)] = unescape(value);
        else if (value == "pass")
          statuses_[key] = Status::Pass;
        else if (value == "fail")
          statuses_[key] = Status::Fail;
        else if (value == "skip")
          statuses_[key] = Status::Skip;
      }
    }

    void save() const {
      std::string text;
      for (const auto& [key, status] : statuses_) {
        if (status != Status::Untested)
          text += key + "\t" + statusName(status) + "\n";
      }
      for (const auto& [key, note] : notes_) {
        if (!note.empty())
          text += "note:" + key + "\t" + escape(note) + "\n";
      }
      writeResultFile(path_, text);
    }

    void reload() {
      statuses_.clear();
      notes_.clear();
      load();
    }

    Status status(const std::string& id) const {
      auto it = statuses_.find(id);
      return it == statuses_.end() ? Status::Untested : it->second;
    }

    void setStatus(const std::string& id, Status status) {
      statuses_[id] = status;
      save();
    }

    std::string note(const std::string& id) const {
      auto it = notes_.find(id);
      return it == notes_.end() ? "" : it->second;
    }

    void setNote(const std::string& id, const std::string& note) {
      notes_[id] = note;
      save();
    }

    void clear() {
      statuses_.clear();
      notes_.clear();
      save();
    }

    const visage::File& path() const { return path_; }

  private:
    visage::File path_;
    std::map<std::string, Status> statuses_;
    std::map<std::string, std::string> notes_;
  };

  static std::string checkId(const TestDefinition& test, int index) {
    return test.id + "." + std::to_string(index);
  }

  class SidebarItem : public visage::Frame {
  public:
    SidebarItem(const TestDefinition* test, Results* results) : test_(test), results_(results) { }

    auto& onSelect() { return on_select_; }

    void draw(visage::Canvas& canvas) override {
      if (selected_)
        canvas.setColor(kPanelLight);
      else if (hover_)
        canvas.setColor(0xff2c323a);
      else
        canvas.setColor(0x00000000);
      canvas.roundedRectangle(0, 0, width(), height(), 4);

      int pass = 0, fail = 0, skip = 0;
      for (int i = 0; i < test_->checks.size(); ++i) {
        Status status = results_->status(checkId(*test_, i));
        pass += status == Status::Pass;
        fail += status == Status::Fail;
        skip += status == Status::Skip;
      }
      int done = pass + fail + skip;
      int total = test_->checks.size();

      canvas.setColor(selected_ ? 0xffffffff : kText);
      canvas.text(test_->title, font(14), visage::Font::kLeft, 10, 0, width() - 70, height());

      unsigned int count_color = fail ? kFail : (done == total ? kPass : kDimText);
      canvas.setColor(count_color);
      canvas.text(std::to_string(done) + "/" + std::to_string(total), font(12),
                  visage::Font::kRight, 0, 0, width() - 10, height());
    }

    void mouseEnter(const visage::MouseEvent& e) override {
      hover_ = true;
      redraw();
    }
    void mouseExit(const visage::MouseEvent& e) override {
      hover_ = false;
      redraw();
    }
    void mouseDown(const visage::MouseEvent& e) override { on_select_.callback(); }

    void setSelected(bool selected) {
      selected_ = selected;
      redraw();
    }

  private:
    const TestDefinition* test_ = nullptr;
    Results* results_ = nullptr;
    bool hover_ = false;
    bool selected_ = false;
    visage::CallbackList<void()> on_select_;
  };

  class Sidebar : public visage::ScrollableFrame {
  public:
    static constexpr float kItemHeight = 26.0f;
    static constexpr float kHeaderHeight = 30.0f;

    Sidebar(const std::vector<TestDefinition>& tests, Results* results) {
      summary_.setMultiLine(true);
      addScrolledChild(&summary_);

      std::string last_category;
      for (int i = 0; i < tests.size(); ++i) {
        if (tests[i].category != last_category) {
          last_category = tests[i].category;
          auto header = std::make_unique<Label>(last_category, 12, kAccent);
          addScrolledChild(header.get());
          headers_.push_back({ i, std::move(header) });
        }
        auto item = std::make_unique<SidebarItem>(&tests[i], results);
        item->onSelect() = [this, i] { on_select_.callback(i); };
        addScrolledChild(item.get());
        items_.push_back(std::move(item));
      }
    }

    auto& onSelect() { return on_select_; }

    void draw(visage::Canvas& canvas) override {
      canvas.setColor(0xff15181c);
      canvas.fill(0, 0, width(), height());
    }

    void resized() override {
      ScrollableFrame::resized();
      float y = 10.0f;
      float w = width() - 16.0f;
      summary_.setBounds(8, y, w, 54);
      y += 62;
      int header_index = 0;
      for (int i = 0; i < items_.size(); ++i) {
        if (header_index < headers_.size() && headers_[header_index].first == i) {
          headers_[header_index].second->setBounds(10, y + 6, w, kHeaderHeight - 6);
          y += kHeaderHeight;
          ++header_index;
        }
        items_[i]->setBounds(8, y, w, kItemHeight);
        y += kItemHeight + 1;
      }
      setScrollableHeight(y + 12);
    }

    void setSelected(int index) {
      for (int i = 0; i < items_.size(); ++i)
        items_[i]->setSelected(i == index);
    }

    void refresh(const std::string& summary) {
      summary_.setText(summary);
      for (auto& item : items_)
        item->redraw();
    }

  private:
    Label summary_ { "", 13, kDimText };
    std::vector<std::pair<int, std::unique_ptr<Label>>> headers_;
    std::vector<std::unique_ptr<SidebarItem>> items_;
    visage::CallbackList<void(int)> on_select_;
  };

  class CheckRow : public visage::Frame {
  public:
    static constexpr float kChipWidth = 44.0f;

    explicit CheckRow(int number, const std::string& text) : number_(number), label_(text, 14) {
      label_.setMultiLine(true);
      addChild(&label_);
    }

    auto& onStatusChange() { return on_status_change_; }

    void setStatus(Status status) {
      status_ = status;
      redraw();
    }

    float heightForWidth(float w) const {
      return std::max(30.0f, label_.heightForWidth(w - kChipWidth - 14) + 10.0f);
    }

    void resized() override {
      label_.setBounds(kChipWidth + 10, 5, width() - kChipWidth - 14, height() - 5);
    }

    void draw(visage::Canvas& canvas) override {
      if (hover_) {
        canvas.setColor(0xff2c323a);
        canvas.roundedRectangle(0, 0, width(), height(), 4);
      }

      canvas.setColor(statusColor(status_));
      canvas.roundedRectangle(4, 5, kChipWidth, 20, 10);
      canvas.setColor(status_ == Status::Untested ? kDimText : 0xffffffff);
      std::string chip;
      if (status_ == Status::Pass)
        chip = "PASS";
      else if (status_ == Status::Fail)
        chip = "FAIL";
      else if (status_ == Status::Skip)
        chip = "SKIP";
      else
        chip = std::to_string(number_);
      canvas.text(chip, font(11), visage::Font::kCenter, 4, 5, kChipWidth, 20);
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
      if (e.shouldTriggerPopup()) {
        visage::PopupMenu menu;
        menu.addOption(1, "Pass").select(status_ == Status::Pass);
        menu.addOption(2, "Fail").select(status_ == Status::Fail);
        menu.addOption(3, "Skip / not applicable").select(status_ == Status::Skip);
        menu.addBreak();
        menu.addOption(0, "Untested").select(status_ == Status::Untested);
        menu.onSelection() = [this](int id) { on_status_change_.callback(static_cast<Status>(id)); };
        menu.show(this, e.position);
        return;
      }

      Status next = Status::Pass;
      if (status_ == Status::Pass)
        next = Status::Fail;
      else if (status_ == Status::Fail)
        next = Status::Skip;
      else if (status_ == Status::Skip)
        next = Status::Untested;
      on_status_change_.callback(next);
    }

  private:
    int number_ = 0;
    Status status_ = Status::Untested;
    bool hover_ = false;
    Label label_;
    visage::CallbackList<void(Status)> on_status_change_;
  };

  class ChecklistPanel : public visage::ScrollableFrame {
  public:
    ChecklistPanel() {
      title_.setMultiLine(true);
      setup_.setMultiLine(true);
      hint_.setMultiLine(true);
      status_.setMultiLine(true);
      addScrolledChild(&title_);
      addScrolledChild(&setup_);
      addScrolledChild(&hint_);
      addScrolledChild(&notes_label_);

      notes_.setMultiLine(true);
      notes_.setJustification(visage::Font::kTopLeft);
      notes_.setFont(font(14));
      notes_.setDefaultText("Notes for this page: what failed, steps to reproduce...");
      notes_.onTextChange() = [this] { on_notes_change_.callback(notes_.text().toUtf8()); };
      addScrolledChild(&notes_);

      prev_ = makeButton("< Prev", [this] { on_navigate_.callback(-1); });
      next_ = makeButton("Next >", [this] { on_navigate_.callback(1); });
      export_ = makeButton("Export Report", [this] { on_export_.callback(); });
      addScrolledChild(prev_.get());
      addScrolledChild(next_.get());
      addScrolledChild(export_.get());
      addScrolledChild(&status_);
    }

    auto& onStatusChange() { return on_status_change_; }
    auto& onNotesChange() { return on_notes_change_; }
    auto& onNavigate() { return on_navigate_; }
    auto& onExport() { return on_export_; }

    void draw(visage::Canvas& canvas) override {
      canvas.setColor(0xff15181c);
      canvas.fill(0, 0, width(), height());
    }

    void setTest(const TestDefinition& test, const Results& results) {
      for (auto& row : rows_)
        removeScrolledChild(row.get());
      rows_.clear();

      title_.setText(test.category + "  /  " + test.title);
      setup_.setText(kWebBuild && !test.web_note.empty() ?
                         test.setup + "\n\nIn the browser: " + test.web_note :
                         test.setup);
      for (int i = 0; i < test.checks.size(); ++i) {
        auto row = std::make_unique<CheckRow>(i + 1, test.checks[i]);
        row->setStatus(results.status(checkId(test, i)));
        row->onStatusChange() = [this, i](Status status) { on_status_change_.callback(i, status); };
        addScrolledChild(row.get());
        rows_.push_back(std::move(row));
      }
      notes_.setText(results.note(test.id));
      setYPosition(0);
      resized();
    }

    void setRowStatus(int index, Status status) {
      if (index >= 0 && index < rows_.size())
        rows_[index]->setStatus(status);
    }

    void setStatusMessage(const std::string& message) {
      status_.setText(message);
      resized();
    }

    void resized() override {
      ScrollableFrame::resized();
      float x = 12.0f;
      float w = width() - 24.0f;
      float y = 12.0f;
      title_.setBounds(x, y, w, title_.heightForWidth(w));
      y = title_.bottom() + 6;
      setup_.setBounds(x, y, w, setup_.heightForWidth(w));
      y = setup_.bottom() + 8;
      hint_.setBounds(x, y, w, hint_.heightForWidth(w));
      y = hint_.bottom() + 8;
      for (auto& row : rows_) {
        float h = row->heightForWidth(w);
        row->setBounds(x - 4, y, w + 8, h);
        y += h + 2;
      }
      y += 10;
      notes_label_.setBounds(x, y, w, 20);
      y += 22;
      notes_.setBounds(x, y, w, 110);
      y += 118;
      layoutButtonRow({ x, y, w, 34 }, { prev_.get(), next_.get(), export_.get() });
      y += 42;
      status_.setBounds(x, y, w, status_.heightForWidth(w));
      y = status_.bottom() + 16;
      setScrollableHeight(y);
    }

  private:
    Label title_ { "", 19, 0xffffffff };
    Label setup_ { "", 14, kText };
    Label hint_ { "Click a check to cycle Pass, Fail, Skip, Untested. Right-click to pick one. "
                  "Results save automatically.",
                  12, kDimText };
    Label notes_label_ { "Notes", 13, kDimText };
    Label status_ { "", 12, kDimText };
    visage::TextEditor notes_;
    std::vector<std::unique_ptr<CheckRow>> rows_;
    std::unique_ptr<visage::UiButton> prev_;
    std::unique_ptr<visage::UiButton> next_;
    std::unique_ptr<visage::UiButton> export_;
    visage::CallbackList<void(int, Status)> on_status_change_;
    visage::CallbackList<void(const std::string&)> on_notes_change_;
    visage::CallbackList<void(int)> on_navigate_;
    visage::CallbackList<void()> on_export_;
  };

  class PageHolder : public visage::Frame {
  public:
    visage::Frame* page() const { return page_.get(); }

    void draw(visage::Canvas& canvas) override {
      canvas.setColor(kBackground);
      canvas.fill(0, 0, width(), height());
    }

    void setPage(std::unique_ptr<visage::Frame> page) {
      if (page_)
        removeChild(page_.get());
      page_ = nullptr;
      page_ = std::move(page);
      if (page_) {
        addChild(page_.get());
        page_->setBounds(localBounds());
      }
    }

    void resized() override {
      if (page_)
        page_->setBounds(localBounds());
    }

  private:
    std::unique_ptr<visage::Frame> page_;
  };

  class ManualTestsWindow : public visage::ApplicationWindow {
  public:
    static constexpr float kSidebarWidth = 230.0f;
    static constexpr float kChecklistWidth = 400.0f;

    ManualTestsWindow() {
      palette_.initWithDefaults();
      setPalette(&palette_);

      addInputTests(tests_);
      addWindowTests(tests_);
      addUiTests(tests_);
      addGraphicsTests(tests_);
      addSvgTests(tests_);
      addEffectsTests(tests_);
      addSystemTests(tests_);
      if (kWebBuild) {
        tests_.erase(std::remove_if(tests_.begin(), tests_.end(),
                                    [](const TestDefinition& test) { return !test.on_web; }),
                     tests_.end());
      }

      sidebar_ = std::make_unique<Sidebar>(tests_, &results_);
      sidebar_->onSelect() = [this](int index) { selectTest(index); };
      addChild(sidebar_.get());
      addChild(&page_holder_);
      addChild(&checklist_);

      checklist_.onStatusChange() = [this](int index, Status status) {
        results_.setStatus(checkId(tests_[current_], index), status);
        checklist_.setRowStatus(index, status);
        refreshSummary();
      };
      checklist_.onNotesChange() = [this](const std::string& note) {
        results_.setNote(tests_[current_].id, note);
      };
      checklist_.onNavigate() = [this](int delta) {
        int next = (current_ + delta + tests_.size()) % tests_.size();
        selectTest(next);
      };
      checklist_.onExport() = [this] { exportReport(); };

      selectTest(lastTestIndex());
      refreshSummary();
      checklist_.setStatusMessage("Results file: " + pathString(results_.path()));

#if VISAGE_EMSCRIPTEN
      storage_timer_.onTimerCallback() = [this] {
        if (!webStorageReady())
          return;
        storage_timer_.stopTimer();
        results_.reload();
        selectTest(lastTestIndex());
        refreshSummary();
        checklist_
            .setStatusMessage("Results are kept in this browser's IndexedDB storage for this site. "
                              "Export Report copies them to the clipboard.");
      };
      storage_timer_.startTimer(50);
#endif
    }

    int lastTestIndex() const {
      visage::File last_test_file = resultsDirectory() / "last_page.txt";
      if (!visage::fileExists(last_test_file))
        return 0;
      std::string last_test = visage::loadFileAsString(last_test_file);
      for (int i = 0; i < tests_.size(); ++i) {
        if (tests_[i].id == last_test)
          return i;
      }
      return 0;
    }

    void draw(visage::Canvas& canvas) override {
      canvas.setColor(kBackground);
      canvas.fill(0, 0, width(), height());
    }

    void resized() override {
      float sidebar_width = std::min(kSidebarWidth, width() * 0.2f);
      float checklist_width = std::min(kChecklistWidth, width() * 0.3f);
      sidebar_->setBounds(0, 0, sidebar_width, height());
      checklist_.setBounds(width() - checklist_width, 0, checklist_width, height());
      page_holder_.setBounds(sidebar_width, 0, width() - sidebar_width - checklist_width, height());
    }

    void selectTest(int index) {
      if (index < 0 || index >= tests_.size())
        return;

      current_ = index;
      page_holder_.setPage(nullptr);
      page_holder_.setPage(tests_[index].create());
      sidebar_->setSelected(index);
      checklist_.setTest(tests_[index], results_);
      writeResultFile(resultsDirectory() / "last_page.txt", tests_[index].id);
      setTitle("Visage Manual Tests - " + tests_[index].title);
    }

    int numTests() const { return tests_.size(); }
    visage::Frame* currentPage() const { return page_holder_.page(); }
    const TestDefinition& test(int index) const { return tests_[index]; }

  private:
    void refreshSummary() {
      int pass = 0, fail = 0, skip = 0, total = 0;
      for (const auto& test : tests_) {
        for (int i = 0; i < test.checks.size(); ++i) {
          Status status = results_.status(checkId(test, i));
          pass += status == Status::Pass;
          fail += status == Status::Fail;
          skip += status == Status::Skip;
          ++total;
        }
      }
      int left = total - pass - fail - skip;
      sidebar_->refresh(std::to_string(pass) + " pass   " + std::to_string(fail) + " fail   " +
                        std::to_string(skip) + " skip\n" + std::to_string(left) + " of " +
                        std::to_string(total) + " checks left");
    }

    void exportReport() {
      std::ostringstream report;
      report << "# Visage manual test report\n\n";
      report << "Generated " << timestamp() << "\n\n";

      int pass = 0, fail = 0, skip = 0, total = 0;
      std::ostringstream failures;
      std::ostringstream details;
      for (const auto& test : tests_) {
        details << "## " << test.category << " / " << test.title << "\n\n";
        for (int i = 0; i < test.checks.size(); ++i) {
          Status status = results_.status(checkId(test, i));
          pass += status == Status::Pass;
          fail += status == Status::Fail;
          skip += status == Status::Skip;
          ++total;
          std::string mark = "[ ]";
          if (status == Status::Pass)
            mark = "PASS";
          else if (status == Status::Fail)
            mark = "**FAIL**";
          else if (status == Status::Skip)
            mark = "skip";
          details << "- " << mark << " " << test.checks[i] << "\n";
          if (status == Status::Fail)
            failures << "- " << test.title << ": " << test.checks[i] << "\n";
        }
        std::string note = results_.note(test.id);
        if (!note.empty())
          details << "\nNotes:\n\n> " << note << "\n";
        details << "\n";
      }

      report << pass << " pass, " << fail << " fail, " << skip << " skip, "
             << (total - pass - fail - skip) << " untested of " << total << " checks\n\n";
      if (fail) {
        report << "## Failures\n\n" << failures.str() << "\n";
      }
      report << details.str();

      visage::File report_file = resultsDirectory() / "report.md";
      writeResultFile(report_file, report.str());
      visage::setClipboardText(report.str());
      checklist_.setStatusMessage("Report written to " + pathString(report_file) +
                                  " and copied to the clipboard.");
    }

    visage::Palette palette_;
    std::vector<TestDefinition> tests_;
    Results results_;
    int current_ = 0;
    std::unique_ptr<Sidebar> sidebar_;
    PageHolder page_holder_;
    ChecklistPanel checklist_;
#if VISAGE_EMSCRIPTEN
    visage::EventTimer storage_timer_;
#endif
  };

  class ScreenshotTour : public visage::EventTimer {
  public:
    ScreenshotTour(ManualTestsWindow* window, visage::File directory) :
        window_(window), directory_(std::move(directory)) {
      visage::createDirectory(directory_);
      if (const char* from = std::getenv("VISAGE_MANUAL_TESTS_FROM")) {
        for (int i = 0; i < window_->numTests(); ++i) {
          if (window_->test(i).id == from)
            index_ = i;
        }
      }
      startTimer(1500);
      window_->selectTest(index_);
    }

    void timerCallback() override {
      window_->takeScreenshot().save(pathString(directory_ / (window_->test(index_).id + ".png")));
      ++index_;
      if (index_ >= window_->numTests()) {
        stopTimer();
        window_->close();
        return;
      }
      window_->selectTest(index_);
    }

  private:
    ManualTestsWindow* window_ = nullptr;
    visage::File directory_;
    int index_ = 0;
  };
  class MonkeyTour : public visage::EventTimer {
  public:
    static constexpr int kIntervalMs = 120;
    static constexpr int kPageSettleTicks = 6;

    MonkeyTour(ManualTestsWindow* window, int repeats) : window_(window), repeats_(repeats) {
      if (const char* from = std::getenv("VISAGE_MANUAL_TESTS_FROM")) {
        for (int i = 0; i < window_->numTests(); ++i) {
          if (window_->test(i).id == from)
            page_ = i;
        }
      }
      window_->selectTest(page_);
      log("PAGE " + window_->test(page_).id);
      startTimer(kIntervalMs);
    }

    void timerCallback() override {
      if (settle_ > 0) {
        --settle_;
        return;
      }

      std::vector<visage::Frame*> targets;
      collect(window_->currentPage(), targets);
      if (action_ < targets.size()) {
        act(targets[action_]);
        ++action_;
        settle_ = isHeavyPage() ? 8 : 1;
        return;
      }

      if (++pass_ < repeats_) {
        action_ = 0;
        return;
      }

      pass_ = 0;
      action_ = 0;
      ++page_;
      if (page_ >= window_->numTests()) {
        log("MONKEY DONE");
        stopTimer();
        window_->close();
        return;
      }
      window_->selectTest(page_);
      log("PAGE " + window_->test(page_).id);
      settle_ = kPageSettleTicks;
    }

  private:
    static bool skipped(const std::string& name) {
      static const char* kSkipped[] = { "closeApplication", "crash",           "message box",
                                        "Warp cursor",      "Export all SVGs", "Hide cursor",
                                        "then quit" };
      for (const char* skip : kSkipped) {
        if (name.find(skip) != std::string::npos)
          return true;
      }
      return false;
    }

    static void collect(visage::Frame* frame, std::vector<visage::Frame*>& targets) {
      if (frame == nullptr || !frame->isVisible())
        return;
      if (dynamic_cast<visage::Button*>(frame) || dynamic_cast<Slider*>(frame)) {
        if (!skipped(frame->name()))
          targets.push_back(frame);
        return;
      }
      for (visage::Frame* child : frame->children())
        collect(child, targets);
    }

    bool isHeavyPage() const { return window_->test(page_).id == "stress"; }

    void act(visage::Frame* target) {
      if (auto* slider = dynamic_cast<Slider*>(target)) {
        float t = (pass_ % 3 == 0) ? 1.0f : (pass_ % 3 == 1 ? 0.0f : 0.5f);
        log("  slider " + target->name() + " -> " + std::to_string(t));
        slider->setFraction(t);
        return;
      }
      if (auto* toggle = dynamic_cast<visage::ToggleButton*>(target)) {
        log("  toggle " + target->name());
        toggle->setToggledAndNotify(!toggle->toggled());
        return;
      }
      auto* button = dynamic_cast<visage::Button*>(target);
      log("  button " + target->name());
      button->notify(true);
    }

    static void log(const std::string& line) {
      printf("%s\n", line.c_str());
      fflush(stdout);
    }

    ManualTestsWindow* window_ = nullptr;
    int repeats_ = 2;
    int page_ = 0;
    int pass_ = 0;
    size_t action_ = 0;
    int settle_ = kPageSettleTicks;
  };
}

int runManualTests() {
#if VISAGE_EMSCRIPTEN
  manual::mountWebStorage();
  manual::g_last_session_status = "Session tracking isn't available in the browser.";
  static manual::ManualTestsWindow* window = new manual::ManualTestsWindow();
  window->setTitle("Visage Manual Tests");
  window->showMaximized();
  window->runEventLoop();
  return 0;
#endif

  visage::File session_file = manual::resultsDirectory() / "session.txt";
  if (visage::fileExists(session_file)) {
    std::string previous = visage::loadFileAsString(session_file);
    if (previous.rfind("clean", 0) == 0)
      manual::g_last_session_status = "Last session exited cleanly (" + previous.substr(6) + ").";
    else
      manual::g_last_session_status =
          "Last session did NOT exit cleanly: it crashed, was killed, or "
          "the event loop never returned (started " +
          previous.substr(std::min<size_t>(previous.size(), 8)) + ").";
  }
  visage::replaceFileWithText(session_file, "running " + manual::timestamp());

  {
    manual::ManualTestsWindow window;
    std::unique_ptr<manual::ScreenshotTour> tour;
    if (const char* screenshot_dir = std::getenv("VISAGE_MANUAL_TESTS_SCREENSHOTS"))
      tour = std::make_unique<manual::ScreenshotTour>(&window, visage::File(screenshot_dir));
    std::unique_ptr<manual::MonkeyTour> monkey;
    if (const char* repeats = std::getenv("VISAGE_MANUAL_TESTS_MONKEY"))
      monkey = std::make_unique<manual::MonkeyTour>(&window, std::max(1, std::atoi(repeats)));

    window.setTitle("Visage Manual Tests");
    window.setMinimumDimensions(900, 600);
    window.show(1500_px, 940_px);
    window.runEventLoop();
  }

  visage::replaceFileWithText(session_file, "clean " + manual::timestamp());
  return 0;
}
