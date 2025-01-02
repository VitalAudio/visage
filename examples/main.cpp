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

int runExample();

#if VISAGE_WINDOWS
#include <windows.h>
int WINAPI WinMain(_In_ HINSTANCE instance, _In_opt_ HINSTANCE prev_instance, _In_ LPSTR cmd_line,
                   _In_ int show_cmd) {
  return runExample();
}
#else
int main(int argc, char** argv) {
  return runExample();
}
#endif
