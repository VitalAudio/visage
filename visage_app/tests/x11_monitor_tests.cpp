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
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#if VISAGE_LINUX
#include "visage_windowing/linux/windowing_x11.h"

#include <csignal>
#include <fcntl.h>
#include <poll.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>
#include <X11/Xlib.h>

extern char** environ;

using namespace visage;

namespace {
  MonitorInfo monitor(int x, int y, int width, int height, float dpi) {
    MonitorInfo result;
    result.bounds = IBounds(x, y, width, height);
    result.dpi = dpi;
    return result;
  }

  class Steps : public EventTimer {
  public:
    explicit Steps(std::vector<std::function<bool()>> steps) : steps_(std::move(steps)) {
      startTimer(20);
    }

    void timerCallback() override {
      if (next_ < steps_.size() && steps_[next_]())
        next_++;
    }

  private:
    std::vector<std::function<bool()>> steps_;
    size_t next_ = 0;
  };

  pid_t spawn(const std::vector<std::string>& arguments,
              const std::vector<std::string>& environment, int inherited_fd = -1) {
    std::vector<char*> argv;
    for (const std::string& argument : arguments)
      argv.push_back(const_cast<char*>(argument.c_str()));
    argv.push_back(nullptr);

    std::vector<char*> envp;
    for (const std::string& variable : environment)
      envp.push_back(const_cast<char*>(variable.c_str()));
    envp.push_back(nullptr);

    pid_t parent = getpid();
    pid_t pid = fork();
    if (pid == 0) {
      prctl(PR_SET_PDEATHSIG, SIGKILL);
      if (getppid() != parent)
        _exit(127);

      int null_fd = open("/dev/null", O_WRONLY);
      dup2(null_fd, STDOUT_FILENO);
      dup2(null_fd, STDERR_FILENO);
      if (inherited_fd >= 0)
        fcntl(inherited_fd, F_SETFD, 0);
      execvpe(argv[0], argv.data(), envp.data());
      _exit(127);
    }
    return pid;
  }

  int readDisplayNumber(int fd, std::chrono::milliseconds timeout) {
    std::string number;
    auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      pollfd poll_fd = { fd, POLLIN, 0 };
      if (poll(&poll_fd, 1, 50) <= 0)
        continue;

      char character = 0;
      if (read(fd, &character, 1) != 1)
        return -1;
      if (character == '\n')
        return number.empty() ? -1 : std::stoi(number);
      number.push_back(character);
    }
    return -1;
  }

  int waitForExit(pid_t pid, std::chrono::seconds timeout) {
    auto end = std::chrono::steady_clock::now() + timeout;
    int status = 0;
    while (waitpid(pid, &status, WNOHANG) == 0) {
      if (std::chrono::steady_clock::now() > end) {
        kill(pid, SIGKILL);
        waitpid(pid, &status, 0);
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  }
}

TEST_CASE("X11 nearest monitor", "[integration]") {
  std::vector<MonitorInfo> monitors = { monitor(0, 0, 1024, 768, 96.0f),
                                        monitor(1024, 0, 1024, 768, 192.0f) };

  CHECK(nearestMonitor(monitors, { 100, 100 }).bounds == monitors[0].bounds);
  CHECK(nearestMonitor(monitors, { 1023, 767 }).bounds == monitors[0].bounds);
  CHECK(nearestMonitor(monitors, { 1024, 0 }).bounds == monitors[1].bounds);
  CHECK(nearestMonitor(monitors, { -300, 100 }).bounds == monitors[0].bounds);
  CHECK(nearestMonitor(monitors, { 3000, 100 }).bounds == monitors[1].bounds);
  CHECK(nearestMonitor(monitors, { 1500, 2000 }).bounds == monitors[1].bounds);
  CHECK(nearestMonitor(monitors, { 900, -50 }).bounds == monitors[0].bounds);
}

TEST_CASE("X11 overlapping monitors pick the first that contains the point", "[integration]") {
  std::vector<MonitorInfo> monitors = { monitor(0, 0, 1024, 768, 96.0f),
                                        monitor(512, 0, 1024, 768, 192.0f) };

  CHECK(nearestMonitor(monitors, { 700, 100 }).bounds == monitors[0].bounds);
  CHECK(nearestMonitor(monitors, { 1100, 100 }).bounds == monitors[1].bounds);
  CHECK(monitorForWindow(monitors, monitors[0], IBounds(500, 100, 400, 300)).bounds == monitors[0].bounds);
}

TEST_CASE("X11 windows keep their monitor when there are no monitors", "[integration]") {
  MonitorInfo current = monitor(0, 0, 1920, 1080, 144.0f);
  MonitorInfo result = monitorForWindow({}, current, IBounds(3000, 100, 400, 300));
  CHECK(result.bounds == current.bounds);
  CHECK(result.dpi == 144.0f);
}

TEST_CASE("X11 windows keep their monitor when their center leaves the screen", "[integration]") {
  std::vector<MonitorInfo> monitors = { monitor(0, 0, 1920, 1080, 192.0f) };

  MonitorInfo result = monitorForWindow(monitors, monitors[0], IBounds(-500, 100, 800, 600));
  CHECK(result.bounds == monitors[0].bounds);
  CHECK(result.dpi == 192.0f);

  result = monitorForWindow(monitors, monitors[0], IBounds(1700, 900, 800, 600));
  CHECK(result.bounds == monitors[0].bounds);
  CHECK(result.dpi == 192.0f);
}

TEST_CASE("X11 windows switch monitors only when the rescaled window stays there",
          "[integration]") {
  MonitorInfo low_left = monitor(0, 0, 1024, 768, 96.0f);
  MonitorInfo high_right = monitor(1024, 0, 1024, 768, 192.0f);
  MonitorInfo high_left = monitor(0, 0, 1024, 768, 192.0f);
  MonitorInfo low_right = monitor(1024, 0, 1024, 768, 96.0f);
  MonitorInfo high_top = monitor(0, 0, 1024, 768, 192.0f);
  MonitorInfo low_bottom = monitor(0, 768, 1024, 768, 96.0f);

  SECTION("Higher DPI on the right") {
    std::vector<MonitorInfo> monitors = { low_left, high_right };
    CHECK(monitorForWindow(monitors, low_left, IBounds(700, 100, 400, 300)).bounds == low_left.bounds);
    CHECK(monitorForWindow(monitors, low_left, IBounds(850, 100, 400, 300)).bounds == high_right.bounds);
    CHECK(monitorForWindow(monitors, high_right, IBounds(600, 100, 800, 600)).bounds == low_left.bounds);
  }

  SECTION("Higher DPI on the left") {
    std::vector<MonitorInfo> monitors = { high_left, low_right };
    CHECK(monitorForWindow(monitors, high_left, IBounds(700, 100, 800, 600)).bounds == high_left.bounds);
    CHECK(monitorForWindow(monitors, high_left, IBounds(900, 100, 800, 600)).bounds == low_right.bounds);
    CHECK(monitorForWindow(monitors, low_right, IBounds(900, 100, 400, 300)).bounds == low_right.bounds);
    CHECK(monitorForWindow(monitors, low_right, IBounds(700, 100, 400, 300)).bounds == low_right.bounds);
    CHECK(monitorForWindow(monitors, low_right, IBounds(500, 100, 400, 300)).bounds == high_left.bounds);
  }

  SECTION("Higher DPI above") {
    std::vector<MonitorInfo> monitors = { high_top, low_bottom };
    CHECK(monitorForWindow(monitors, high_top, IBounds(100, 500, 800, 600)).bounds == high_top.bounds);
    CHECK(monitorForWindow(monitors, high_top, IBounds(100, 700, 800, 600)).bounds == low_bottom.bounds);
    CHECK(monitorForWindow(monitors, low_bottom, IBounds(100, 500, 400, 300)).bounds == low_bottom.bounds);
    CHECK(monitorForWindow(monitors, low_bottom, IBounds(100, 400, 400, 300)).bounds == high_top.bounds);
  }

  SECTION("Same DPI") {
    MonitorInfo right = monitor(1024, 0, 1024, 768, 96.0f);
    std::vector<MonitorInfo> monitors = { low_left, right };
    CHECK(monitorForWindow(monitors, low_left, IBounds(850, 100, 400, 300)).bounds == right.bounds);
    CHECK(monitorForWindow(monitors, right, IBounds(800, 100, 400, 300)).bounds == low_left.bounds);
  }
}

TEST_CASE("X11 windows partly off a HiDPI screen child", "[.x11_offscreen_dpi_child]") {
  if (defaultDpiScale() != 2.0f)
    SKIP("The display isn't 192 dpi");

  ApplicationWindow app;
  app.show(Dimension::nativePixels(100), Dimension::nativePixels(100),
           Dimension::logicalPixels(200), Dimension::logicalPixels(150));

  Display* display = XOpenDisplay(nullptr);
  REQUIRE(display);
  ::Window window = (::Window)app.window()->nativeHandle();
  std::vector<float> scales;
  std::vector<int> widths;
  auto move = [&](int x, int y) {
    XMoveWindow(display, window, x, y);
    XFlush(display);
    return true;
  };
  auto record = [&, ticks = 0]() mutable {
    if (++ticks < 10)
      return false;
    ticks = 0;
    scales.push_back(app.window()->dpiScale());
    widths.push_back(app.window()->clientWidth());
    return true;
  };
  Steps steps({ [&] { return move(-300, 100); }, record, [&] { return move(100, 100); }, record,
                [&] {
                  app.close();
                  return true;
                } });
  app.runEventLoop();
  XCloseDisplay(display);

  CHECK(scales == std::vector<float> { 2.0f, 2.0f });
  CHECK(widths == std::vector<int> { 400, 400 });
}

TEST_CASE("X11 windows keep their DPI when dragged partly off a HiDPI screen", "[integration]") {
  std::string executable = std::filesystem::read_symlink("/proc/self/exe").string();
  std::vector<std::string> environment;
  for (char** variable = environ; *variable; ++variable) {
    std::string entry = *variable;
    if (entry.rfind("DISPLAY=", 0) != 0 && entry.rfind("ASAN_OPTIONS=", 0) != 0)
      environment.push_back(entry);
  }

  int display_fds[2];
  REQUIRE(pipe2(display_fds, O_CLOEXEC) == 0);
  pid_t xvfb = spawn({ "Xvfb", "-displayfd", std::to_string(display_fds[1]), "-screen", "0",
                       "1920x1080x24", "-dpi", "192", "-nolisten", "tcp", "-noreset" },
                     environment, display_fds[1]);
  close(display_fds[1]);
  int display_number = xvfb < 0 ? -1 : readDisplayNumber(display_fds[0], std::chrono::seconds(10));
  close(display_fds[0]);
  if (display_number < 0) {
    if (xvfb > 0) {
      kill(xvfb, SIGTERM);
      waitpid(xvfb, nullptr, 0);
    }
    SKIP("Xvfb isn't available or didn't start");
  }

  const char* asan_options = std::getenv("ASAN_OPTIONS");
  environment.push_back(std::string("ASAN_OPTIONS=") + (asan_options ? asan_options : "") + ":detect_leaks=0");
  environment.push_back("DISPLAY=:" + std::to_string(display_number));
  pid_t child = spawn({ executable, "[.x11_offscreen_dpi_child]" }, environment);
  int status = child < 0 ? -1 : waitForExit(child, std::chrono::seconds(60));
  kill(xvfb, SIGTERM);
  waitpid(xvfb, nullptr, 0);

  if (status == 4)
    SKIP("Xvfb doesn't report 192 dpi");
  REQUIRE(status == 0);
}

#endif
