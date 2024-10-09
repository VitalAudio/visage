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
