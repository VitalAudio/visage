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

#include "child_process.h"

#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#if VISAGE_MAC
#include <crt_externs.h>
#else
extern char** environ;
#endif

namespace visage {
  static char** environment() {
#if VISAGE_MAC
    return *_NSGetEnviron();
#else
    return environ;
#endif
  }

  static void closePipes(int out_pipe, int err_pipe) {
    close(out_pipe);
    close(err_pipe);
  }

  static void terminateAndCleanup(pid_t pid, int out_pipe, int err_pipe, int* status) {
    static constexpr int kTerminateTimeoutMs = 100;

    kill(pid, SIGTERM);
    closePipes(out_pipe, err_pipe);

    auto start_time = std::chrono::steady_clock::now();
    while (waitpid(pid, status, WNOHANG) == 0) {
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > kTerminateTimeoutMs) {
        kill(pid, SIGKILL);
        waitpid(pid, status, 0);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  static std::vector<std::string> splitArguments(const std::string& arguments) {
    std::vector<std::string> result;
    std::string argument;
    bool has_argument = false;
    bool quoted = false;
    size_t backslashes = 0;

    for (char c : arguments) {
      if (c == '\\') {
        backslashes++;
        has_argument = true;
        continue;
      }

      if (c == '"') {
        argument.append(backslashes / 2, '\\');
        if (backslashes % 2)
          argument.push_back(c);
        else
          quoted = !quoted;
        has_argument = true;
      }
      else {
        argument.append(backslashes, '\\');
        if ((c == ' ' || c == '\t') && !quoted) {
          if (has_argument)
            result.push_back(argument);
          argument.clear();
          has_argument = false;
        }
        else {
          argument.push_back(c);
          has_argument = true;
        }
      }
      backslashes = 0;
    }

    argument.append(backslashes, '\\');
    if (has_argument)
      result.push_back(argument);
    return result;
  }

  static std::vector<char*> parseArguments(const std::string& command, const std::string& arguments,
                                           std::vector<std::string>& storage) {
    storage = splitArguments(arguments);
    storage.insert(storage.begin(), command);

    std::vector<char*> args;
    for (auto& str : storage)
      args.push_back(&str[0]);

    args.push_back(nullptr);
    return args;
  }

  static bool setupPipes(int out_pipe[2], int err_pipe[2], posix_spawn_file_actions_t* file_actions) {
    if (pipe(out_pipe) == -1 || pipe(err_pipe) == -1)
      return false;

    posix_spawn_file_actions_init(file_actions);
    posix_spawn_file_actions_adddup2(file_actions, out_pipe[1], STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(file_actions, err_pipe[1], STDERR_FILENO);
    posix_spawn_file_actions_addclose(file_actions, out_pipe[0]);
    posix_spawn_file_actions_addclose(file_actions, err_pipe[0]);
    posix_spawn_file_actions_addclose(file_actions, out_pipe[1]);
    posix_spawn_file_actions_addclose(file_actions, err_pipe[1]);
    return true;
  }

  static bool readPipeWithSizeCheck(int pipe_fd, char* buffer, std::string& output) {
    ssize_t count;
    while ((count = read(pipe_fd, buffer, 255)) > 0) {
      if (output.size() + count > kMaxOutputSize)
        return false;
      output += std::string(buffer, count);
    }
    return true;
  }

  bool spawnChildProcess(const std::string& command, const std::string& arguments,
                         std::string& output, int timeout_ms) {
    std::vector<std::string> arg_storage;
    std::vector<char*> args = parseArguments(command, arguments, arg_storage);

    int out_pipe[2];
    int err_pipe[2];
    posix_spawn_file_actions_t file_actions;

    if (!setupPipes(out_pipe, err_pipe, &file_actions))
      return false;

    pid_t pid;
    int status;
    int result = posix_spawnp(&pid, command.c_str(), &file_actions, nullptr, args.data(), environment());

    posix_spawn_file_actions_destroy(&file_actions);

    if (result != 0) {
      closePipes(out_pipe[0], out_pipe[1]);
      closePipes(err_pipe[0], err_pipe[1]);
      return false;
    }

    closePipes(out_pipe[1], err_pipe[1]);

    fcntl(out_pipe[0], F_SETFL, O_NONBLOCK);
    fcntl(err_pipe[0], F_SETFL, O_NONBLOCK);

    char buffer[256];
    output = "";

    auto start_time = std::chrono::steady_clock::now();
    bool process_finished = false;

    while (!process_finished) {
      int wait_result = waitpid(pid, &status, WNOHANG);
      if (wait_result == pid)
        process_finished = true;
      else if (wait_result == -1) {
        if (errno == ECHILD) {
          process_finished = true;
          status = 0;
        }
        else {
          closePipes(out_pipe[0], err_pipe[0]);
          return false;
        }
      }

      bool output_fits = readPipeWithSizeCheck(out_pipe[0], buffer, output) &&
                         readPipeWithSizeCheck(err_pipe[0], buffer, output);

      auto elapsed = std::chrono::steady_clock::now() - start_time;
      bool timed_out = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout_ms;
      if (!output_fits || (timed_out && !process_finished)) {
        if (process_finished)
          closePipes(out_pipe[0], err_pipe[0]);
        else
          terminateAndCleanup(pid, out_pipe[0], err_pipe[0], &status);
        return false;
      }

      if (!process_finished)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    closePipes(out_pipe[0], err_pipe[0]);

    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
  }
}
