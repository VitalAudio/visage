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

#include <chrono>
#include <clocale>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <thread>
#include <visage_utils/child_process.h>
#include <visage_utils/file_system.h>

namespace manual {
  namespace {
    struct ProcessCase {
      std::string name;
      std::string command;
      std::string arguments;
      int timeout_ms;
      std::function<bool(bool, const std::string&, double)> passes;
    };

    std::string printable(const std::string& text) {
      std::string result;
      for (char c : text) {
        if (c == '\0')
          result += "\\0";
        else if (c == '\n')
          result += "\\n";
        else if (c == '\r')
          result += "\\r";
        else
          result += c;
        if (result.size() > 90) {
          result += "...";
          break;
        }
      }
      return result;
    }

    std::vector<ProcessCase> processCases() {
#if VISAGE_WINDOWS
      return {
        { "PATH lookup and quotes", "cmd", "/c echo hello \"two words\"", 5000,
          [](bool ok, const std::string& out, double) {
            return ok && out.find("hello") != std::string::npos;
          } },
        { "Environment", "cmd", "/c echo %USERPROFILE%", 5000,
          [](bool ok, const std::string& out, double) {
            return ok && out.find('%') == std::string::npos;
          } },
        { "Command path with spaces", "C:\\Windows\\System32\\cmd.exe", "/c echo spaces ok", 5000,
          [](bool ok, const std::string& out, double) {
            return ok && out.find("spaces ok") != std::string::npos;
          } },
        { "Timeout", "cmd", "/c ping -n 30 127.0.0.1", 1000,
          [](bool ok, const std::string&, double seconds) { return !ok && seconds < 5.0; } },
        { "Missing command", "visage-command-that-does-not-exist", "", 5000,
          [](bool ok, const std::string&, double) { return !ok; } },
      };
#else
      return {
        { "PATH lookup and quotes", "echo", R"(hello "two words" "a \"quoted\" word")", 5000,
          [](bool ok, const std::string& out, double) {
            return ok && out.find("hello two words a \"quoted\" word") != std::string::npos;
          } },
        { "Environment", "sh", R"(-c "echo $HOME")", 5000,
          [](bool ok, const std::string& out, double) {
            return ok && out.size() > 2 && out[0] == '/';
          } },
        { "NUL bytes in output", "printf", R"(a\000b\000c)", 5000,
          [](bool ok, const std::string& out, double) {
            return ok && out.size() == 5 && out[1] == '\0';
          } },
        { "2 MB of output is capped", "sh", R"(-c "head -c 2000000 /dev/zero | tr '\000' x")", 10000,
          [](bool ok, const std::string& out, double) {
            return out.size() > 1000 && out.size() <= visage::kMaxOutputSize;
          } },
        { "Timeout", "sleep", "30", 1000,
          [](bool ok, const std::string&, double seconds) { return !ok && seconds < 5.0; } },
        { "Timeout when SIGTERM is ignored", "sh", R"(-c "trap '' TERM; sleep 30")", 1000,
          [](bool ok, const std::string&, double seconds) {
            return !ok && seconds > 0.5 && seconds < 10.0;
          } },
        { "Missing command", "visage-command-that-does-not-exist", "", 5000,
          [](bool ok, const std::string&, double) { return !ok; } },
      };
#endif
    }

    class SystemPage : public visage::Frame {
    public:
      SystemPage() {
        run_processes_ = makeButton("Run child process checks", [this] { runProcesses(); });
        run_files_ = makeButton("Run file system checks", [this] { runFileChecks(); });
        run_strings_ = makeButton("Run number formatting checks", [this] { runStringChecks(); });
        clear_ = makeButton("Clear log", [this] { log_.clear(); });
        for (auto* button : { run_processes_.get(), run_files_.get(), run_strings_.get(), clear_.get() })
          addChild(button);
        addChild(&log_);
        if (std::getenv("VISAGE_MANUAL_TESTS_AUTORUN")) {
          print_ = true;
          runFileChecks();
          runStringChecks();
          runProcesses();
        }
      }

      ~SystemPage() override { *alive_ = false; }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        layoutButtonRow(takeTop(bounds, 34),
                        { run_processes_.get(), run_files_.get(), run_strings_.get(), clear_.get() });
        log_.setBounds(bounds);
      }

    private:
      void result(bool pass, const std::string& text) {
        std::string line = std::string(pass ? "PASS  " : "FAIL  ") + text;
        log_.add(line);
        if (print_)
          printf("%s\n", line.c_str()), fflush(stdout);
      }

      void runProcesses() {
        if (kWebBuild) {
          log_.add("Child processes can't run in a browser.");
          return;
        }
        log_.add("Running child processes on a background thread. The UI should stay responsive.");
        std::shared_ptr<bool> alive = alive_;
        std::thread([this, alive] {
          for (const ProcessCase& process : processCases()) {
            std::string output;
            auto start = std::chrono::steady_clock::now();
            bool ok = visage::spawnChildProcess(process.command, process.arguments, output,
                                                process.timeout_ms);
            double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            bool pass = process.passes(ok, output, seconds);
            std::string line = process.name + ": returned " + (ok ? "true" : "false") + " in " +
                               visage::String(static_cast<float>(seconds), 2).toUtf8() + "s, " +
                               std::to_string(output.size()) + " bytes \"" + printable(output) + "\"";
            visage::runOnEventThread([this, alive, pass, line] {
              if (*alive)
                result(pass, line);
            });
          }
          visage::runOnEventThread([this, alive] {
            if (*alive)
              log_.add("Child process checks done.");
          });
        }).detach();
      }

      void runFileChecks() {
        if (kWebBuild)
          log_.add("The browser has no app data, documents or executable paths, so those checks "
                   "are skipped.");
        else {
          visage::File app_data = visage::appDataDirectory();
          visage::File documents = visage::userDocumentsDirectory();
          visage::File executable = visage::hostExecutable();
          result(!app_data.empty() && pathString(app_data).find('~') == std::string::npos &&
                     visage::isDirectory(app_data),
                 "appDataDirectory: " + pathString(app_data));
          result(!documents.empty() && pathString(documents).find('~') == std::string::npos,
                 "userDocumentsDirectory: " + pathString(documents));
          result(visage::fileExists(executable), "hostExecutable: " + pathString(executable));
        }

        visage::File first = visage::createTemporaryFile("txt");
        visage::File second = visage::createTemporaryFile("txt");
        result(first != second, "Two temporary files made back to back differ: " +
                                    visage::fileName(first) + " / " + visage::fileName(second));

        visage::File missing = visage::File(std::filesystem::temp_directory_path()) /
                               "visage-manual-missing" / "nothing.txt";
        result(!visage::hasWriteAccess(missing), "hasWriteAccess on a missing file returns false");
        result(!visage::fileExists(missing), "fileExists on a missing file returns false");

        visage::File directory = std::filesystem::temp_directory_path();
        std::string as_string;
        bool threw = false;
        try {
          as_string = visage::loadFileAsString(directory);
          size_t size = 0;
          visage::loadFileData(directory, size);
        }
        catch (...) {
          threw = true;
        }
        result(!threw && as_string.empty(),
               "Loading a directory as a file returns nothing without throwing");

        visage::File unicode = std::filesystem::temp_directory_path() /
                               pathFromUtf8("visage ünïcødé файл 日本.txt");
        std::string text = "Unicode round trip ✓ 🎉";
        bool wrote = visage::replaceFileWithText(unicode, text);
        std::string read = visage::loadFileAsString(unicode);
        result(wrote && read == text,
               "Unicode file name write and read back: " + visage::fileName(unicode));
        bool appended = visage::appendTextToFile(unicode, "!");
        result(appended && visage::loadFileAsString(unicode) == text + "!", "appendTextToFile");
        std::error_code error;
        std::filesystem::remove(unicode, error);
        std::filesystem::remove(first, error);
        std::filesystem::remove(second, error);

        bool search_threw = false;
        size_t found = 0;
        try {
          found = visage::searchForFiles(std::filesystem::temp_directory_path(), ".*\\.txt").size();
        }
        catch (...) {
          search_threw = true;
        }
        result(!search_threw, "searchForFiles over the temp directory found " +
                                  std::to_string(found) + " .txt files without throwing");
        if (!kWebBuild)
          result(!visage::hostName().empty(), "hostName: " + visage::hostName());
      }

      void runStringChecks() {
        auto check = [this](const visage::String& value, const std::string& expected,
                            const std::string& what) {
          std::string actual = value.toUtf8();
          result(actual == expected, what + " -> \"" + actual + "\" (expected \"" + expected + "\")");
        };

        auto checkAll = [&](const std::string& locale_name) {
          log_.add("Locale: " + locale_name);
          check(visage::String(3.14159f, 3), "3.142", "String(3.14159f, 3)");
          check(visage::String(-2.5f, 1), "-2.5", "String(-2.5f, 1)");
          check(visage::String(-0.25f, 1), "-0.3", "String(-0.25f, 1)");
          check(visage::String(1.23456789, 8), "1.23456789", "String(1.23456789, 8)");
          check(visage::String(2.0f, 0), "2", "String(2.0f, 0)");
          check(visage::String(1234.5f), "1234.5", "String(1234.5f)");
          float parsed = visage::String("1.5").toFloat();
          result(parsed == 1.5f, "String(\"1.5\").toFloat() -> " + std::to_string(parsed));
        };

        checkAll("default");
        const char* previous = std::setlocale(LC_ALL, nullptr);
        std::string saved = previous ? previous : "C";
        const char* comma_locales[] = { "de_DE.UTF-8", "de_DE", "fr_FR.UTF-8", "German" };
        bool switched = false;
        for (const char* name : comma_locales) {
          if (std::setlocale(LC_ALL, name)) {
            switched = true;
            checkAll(std::string(name) + " (comma decimal point)");
            break;
          }
        }
        if (!switched)
          log_.add("No comma decimal locale is installed, so the locale checks were skipped.");
        std::setlocale(LC_ALL, saved.c_str());
      }

      std::shared_ptr<bool> alive_ = std::make_shared<bool>(true);
      bool print_ = false;
      LogView log_;
      std::unique_ptr<visage::UiButton> run_processes_;
      std::unique_ptr<visage::UiButton> run_files_;
      std::unique_ptr<visage::UiButton> run_strings_;
      std::unique_ptr<visage::UiButton> clear_;
    };
  }

  void addSystemTests(std::vector<TestDefinition>& tests) {
    tests.push_back(
        { "system",
          "System",
          "Processes, Files and Strings",
          "Each button runs a set of checks and logs PASS or FAIL. Run them on every platform.",
          {
              "Every child process check logs PASS, and the window stays responsive while they "
              "run.",
              "The timeout checks return in about a second, even when the child ignores SIGTERM.",
              "Every file system check logs PASS. The directories are real paths with no ~ in "
              "them.",
              "Every number formatting check logs PASS in the default locale.",
              "If a comma decimal locale is installed, the formatting checks also pass in it.",
              "Leaving this page while child processes are still running causes no crash.",
          },
          [] { return std::make_unique<SystemPage>(); },
          true,
          "Child processes can't run in a browser and there's no app data or documents directory, "
          "so only "
          "the number formatting checks and the temporary and Unicode file checks apply." });
  }
}
