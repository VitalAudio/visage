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

#pragma once

#include <filesystem>
#include <vector>

namespace visage {
  typedef std::filesystem::path File;

  void replaceFileWithData(const File& file, const char* data, size_t size);
  void replaceFileWithText(const File& file, const std::string& text);
  bool hasWriteAccess(const File& file);
  bool fileExists(const File& file);
  void appendTextToFile(const File& file, const std::string& text);
  std::unique_ptr<char[]> loadFileData(const File& file, int& size);
  std::string loadFileAsString(const File& file);

  File hostExecutable();
  File appDataDirectory();
  File userDocumentsDirectory();
  File createTemporaryFile(const std::string& extension);
  std::string fileName(const File& file);
  std::string fileStem(const File& file);
  std::string hostName();
  std::vector<File> searchForFiles(const File& directory, const std::string& regex);
  std::vector<File> searchForDirectories(const File& directory, const std::string& regex);
}