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