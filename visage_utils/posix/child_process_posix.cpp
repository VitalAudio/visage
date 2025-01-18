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

#include <spawn.h>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace visage {

  bool spawnChildProcess(const std::string& command, const std::string& arguments,
                         std::string& output, int timeout_ms) {
    static constexpr char* kEnvironment[] = { nullptr };

    std::istringstream stream(arguments);
    std::vector<std::string> argument_list;
    std::string segment;
    while (std::getline(stream, segment, ' '))
      argument_list.push_back(segment);

    std::string arg0 = command;
    std::vector<char*> args;
    args.push_back(&arg0[0]);
    for (auto& arg : argument_list)
      args.push_back(&arg[0]);

    args.push_back(nullptr);

    int out_pipe[2];
    int err_pipe[2];

    if (pipe(out_pipe) == -1 || pipe(err_pipe) == -1)
      return false;

    posix_spawn_file_actions_t file_actions;
    posix_spawn_file_actions_init(&file_actions);

    posix_spawn_file_actions_adddup2(&file_actions, out_pipe[1], STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&file_actions, err_pipe[1], STDERR_FILENO);

    posix_spawn_file_actions_addclose(&file_actions, out_pipe[0]);
    posix_spawn_file_actions_addclose(&file_actions, err_pipe[0]);

    posix_spawn_file_actions_addclose(&file_actions, out_pipe[1]);
    posix_spawn_file_actions_addclose(&file_actions, err_pipe[1]);

    pid_t pid;
    int status;
    int result = posix_spawn(&pid, command.c_str(), &file_actions, nullptr, args.data(), kEnvironment);
    close(out_pipe[1]);
    close(err_pipe[1]);

    char buffer[256];
    ssize_t count;

    output = "";
    while ((count = read(out_pipe[0], buffer, sizeof(buffer) - 1)) > 0)
      output += std::string(buffer, count);

    while ((count = read(err_pipe[0], buffer, sizeof(buffer) - 1)) > 0)
      output += std::string(buffer, count);

    posix_spawn_file_actions_destroy(&file_actions);
    if (result != 0)
      return false;

    if (waitpid(pid, &status, 0) != -1) {
      if (!WIFEXITED(status))
        return false;
      if (WEXITSTATUS(status))
        return false;
      return true;
    }
    return false;
  }
}
