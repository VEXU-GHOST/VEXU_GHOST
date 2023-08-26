// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================
#include "util/serialization.h"

#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <string>

#include "glog/logging.h"

namespace util {
namespace serialization {

std::string RandomString(size_t length) {
  srand(time(NULL));
  auto randchar = []() -> char {
    const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    const size_t max_index = (sizeof(charset) - 1);
    return charset[rand() % max_index];
  };
  std::string str(length, 0);
  std::generate_n(str.begin(), length, randchar);
  return str;
}

std::string PrepareDirectory(size_t random_string_length) {
  const std::string path =
      "test_outputs/dir" + RandomString(random_string_length) + "/";
  for (int i = 0;
       i < 10 && mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0;
       ++i) {
  }
  return path;
}
constexpr int kNumRandomChars = 20;
constexpr int kNumTotalChars = kNumRandomChars + 18;
static char kRandomDirectory[kNumTotalChars] = {0};

int CreateOrEraseFileForWrite(const std::string& file_name) {
  // Initialize the static folder path buffer, if not already initialized.
  if (strlen(kRandomDirectory) == 0) {
    strncpy(kRandomDirectory, PrepareDirectory(kNumRandomChars).c_str(),
            kNumTotalChars - 1);
  }
  mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  int file_descriptor = open((kRandomDirectory + file_name).c_str(),
                             O_CREAT | O_WRONLY | O_TRUNC, mode);
  if (file_descriptor < 0) {
    LOG(FATAL) << "Error opening file " << (kRandomDirectory + file_name);
  }
  return file_descriptor;
}

int OpenFileForRead(const std::string& file_name) {
  // Initialize the static folder path buffer, if not already initialized.
  if (strlen(kRandomDirectory) == 0) {
    strncpy(kRandomDirectory, PrepareDirectory(kNumRandomChars).c_str(),
            kNumTotalChars - 1);
  }
  return OpenGeneralFileForRead(kRandomDirectory + file_name);
}

int OpenGeneralFileForRead(const std::string& file_name) {
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    LOG(FATAL) << "Error opening file " << file_name;
  }
  return file_descriptor;
}

std::string GetFolderName() {
  if (strlen(kRandomDirectory) == 0) {
    strncpy(kRandomDirectory, PrepareDirectory(kNumRandomChars).c_str(),
            kNumTotalChars - 1);
  }
  return std::string(kRandomDirectory);
}

std::string GetFullFolderPath() {
  if (strlen(kRandomDirectory) == 0) {
    strncpy(kRandomDirectory, PrepareDirectory(kNumRandomChars).c_str(),
            kNumTotalChars - 1);
  }

  char cwd[1024] = {0};
  const auto result = getcwd(cwd, sizeof(cwd));
  CHECK_NOTNULL(result);
  return std::string(cwd) + "/" + std::string(kRandomDirectory);
}

}  // namespace serialization
}  // namespace util
