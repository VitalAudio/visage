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

#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace visage {
  bool spawnChildProcess(const std::string& command, std::string& output, int timeout_ms) {
    std::istringstream stream(command);
    std::vector<std::string> arguments;
    std::string segment;
    while (std::getline(stream, segment, ' '))
      arguments.push_back(segment);

    std::vector<char*> args;
    for (auto& arg : arguments)
      args.push_back(&arg[0]);

    args.push_back(nullptr);

    pid_t pid = fork();

    if (pid == -1)
      return false;

    if (pid > 0) {
      int status = 0;
      waitpid(pid, &status, 0);

      if (!WIFEXITED(status))
        return false;
      if (WEXITSTATUS(status))
        return false;
    }
    else {
      execvp(args[0], args.data());
      exit(1);
    }

    return true;
  }
}
