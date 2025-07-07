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

#include "visage_utils/file_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <thread>

using namespace visage;

TEST_CASE("File existence check", "[utils]") {
  File temp_file = createTemporaryFile("txt");

  REQUIRE_FALSE(fileExists(temp_file));

  REQUIRE(replaceFileWithText(temp_file, "test content"));
  REQUIRE(fileExists(temp_file));

  std::filesystem::remove(temp_file);
  REQUIRE_FALSE(fileExists(temp_file));
}

TEST_CASE("Replace file with text", "[utils]") {
  File temp_file = createTemporaryFile("txt");
  std::string test_content = "Hello, World!\nLine 2\nUnicode: ñáéíóú";

  REQUIRE(replaceFileWithText(temp_file, test_content));
  REQUIRE(fileExists(temp_file));

  std::string loaded_content = loadFileAsString(temp_file);
  REQUIRE(loaded_content == test_content);

  std::filesystem::remove(temp_file);
}

TEST_CASE("Replace file with binary data", "[utils]") {
  File temp_file = createTemporaryFile("bin");

  unsigned char test_data[] = { 0x00, 0x01, 0x02, 0xFF, 0xFE, 0xFD, 0x80, 0x7F };
  size_t data_size = sizeof(test_data);

  REQUIRE(replaceFileWithData(temp_file, test_data, data_size));
  REQUIRE(fileExists(temp_file));

  int loaded_size = 0;
  auto loaded_data = loadFileData(temp_file, loaded_size);
  REQUIRE(loaded_size == static_cast<int>(data_size));

  bool data_matches = true;
  for (int i = 0; i < loaded_size; ++i) {
    if (loaded_data[i] != test_data[i]) {
      data_matches = false;
      break;
    }
  }
  REQUIRE(data_matches);

  std::filesystem::remove(temp_file);
}

TEST_CASE("Append text to file", "[utils]") {
  File temp_file = createTemporaryFile("txt");

  REQUIRE(replaceFileWithText(temp_file, "First line\n"));
  REQUIRE(appendTextToFile(temp_file, "Second line\n"));
  REQUIRE(appendTextToFile(temp_file, "Third line"));

  std::string content = loadFileAsString(temp_file);
  REQUIRE(content == "First line\nSecond line\nThird line");

  std::filesystem::remove(temp_file);
}

TEST_CASE("Load non-existent file", "[utils]") {
  File non_existent = "/this/path/should/not/exist/test.txt";

  REQUIRE_FALSE(fileExists(non_existent));

  std::string content = loadFileAsString(non_existent);
  REQUIRE(content.empty());

  int size = -1;
  auto data = loadFileData(non_existent, size);
  REQUIRE(data == nullptr);
  REQUIRE(size == -1);
}

TEST_CASE("Empty file operations", "[utils]") {
  File temp_file = createTemporaryFile("empty");

  REQUIRE(replaceFileWithText(temp_file, ""));
  REQUIRE(fileExists(temp_file));

  std::string content = loadFileAsString(temp_file);
  REQUIRE(content.empty());

  int size = -1;
  auto data = loadFileData(temp_file, size);
  REQUIRE(size == 0);
  REQUIRE(data != nullptr);

  std::filesystem::remove(temp_file);
}

TEST_CASE("File name utilities", "[utils]") {
  File test_path = "/path/to/file.txt";

  REQUIRE(fileName(test_path) == "file.txt");
  REQUIRE(fileStem(test_path) == "file");

  File no_extension = "/path/to/filename";
  REQUIRE(fileName(no_extension) == "filename");
  REQUIRE(fileStem(no_extension) == "filename");

  File multiple_dots = "/path/to/file.backup.txt";
  REQUIRE(fileName(multiple_dots) == "file.backup.txt");
  REQUIRE(fileStem(multiple_dots) == "file.backup");
}

TEST_CASE("Host executable path", "[utils]") {
  File executable = hostExecutable();
  REQUIRE_FALSE(executable.empty());
  REQUIRE(fileExists(executable));

  std::string host_name = hostName();
  REQUIRE_FALSE(host_name.empty());
}

TEST_CASE("System directories", "[utils]") {
  File app_data = appDataDirectory();
  File documents = userDocumentsDirectory();

  REQUIRE_FALSE(app_data.empty());
  REQUIRE_FALSE(documents.empty());
}

TEST_CASE("Temporary file creation", "[utils]") {
  File temp1 = createTemporaryFile("test");
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  File temp2 = createTemporaryFile("test");

  REQUIRE(temp1 != temp2);
  REQUIRE(temp1.extension() == ".test");
  REQUIRE(temp2.extension() == ".test");

  File temp_no_ext = createTemporaryFile("");
  REQUIRE(temp_no_ext.extension() == ".");
}

TEST_CASE("Search for files", "[utils]") {
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path() / "visage_test_search";
  std::filesystem::create_directories(temp_dir);

  (void)replaceFileWithText(temp_dir / "test1.txt", "content");
  (void)replaceFileWithText(temp_dir / "test2.cpp", "content");
  (void)replaceFileWithText(temp_dir / "other.log", "content");
  std::filesystem::create_directories(temp_dir / "subdir");
  (void)replaceFileWithText(temp_dir / "subdir" / "test3.txt", "content");

  std::vector<File> txt_files = searchForFiles(temp_dir, ".*\\.txt");
  REQUIRE(txt_files.size() == 2);

  std::vector<File> test_files = searchForFiles(temp_dir, "test.*");
  REQUIRE(test_files.size() == 3);

  std::vector<File> no_match = searchForFiles(temp_dir, "nonexistent");
  REQUIRE(no_match.empty());

  std::filesystem::remove_all(temp_dir);
}

TEST_CASE("Search for directories", "[utils]") {
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path() /
                                   "visage_test_dir_search";
  std::filesystem::create_directories(temp_dir);

  std::filesystem::create_directories(temp_dir / "testdir1");
  std::filesystem::create_directories(temp_dir / "testdir2");
  std::filesystem::create_directories(temp_dir / "otherdir");
  std::filesystem::create_directories(temp_dir / "subdir" / "testdir3");

  std::vector<File> test_dirs = searchForDirectories(temp_dir, "test.*");
  REQUIRE(test_dirs.size() == 3);

  std::vector<File> all_dirs = searchForDirectories(temp_dir, ".*dir.*");
  REQUIRE(test_dirs.size() <= all_dirs.size());

  std::vector<File> no_match = searchForDirectories(temp_dir, "nonexistent");
  REQUIRE(no_match.empty());

  std::filesystem::remove_all(temp_dir);
}

TEST_CASE("Search in non-existent directory", "[utils]") {
  File non_existent_dir = "/this/path/should/not/exist";

  std::vector<File> files = searchForFiles(non_existent_dir, ".*");
  REQUIRE(files.empty());

  std::vector<File> dirs = searchForDirectories(non_existent_dir, ".*");
  REQUIRE(dirs.empty());
}

TEST_CASE("Write access check", "[utils]") {
  File temp_file = createTemporaryFile("access_test");

  REQUIRE(replaceFileWithText(temp_file, "test"));
  REQUIRE(hasWriteAccess(temp_file));

  std::filesystem::remove(temp_file);
}