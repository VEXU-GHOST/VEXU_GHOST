// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Serialization library.
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

#ifndef SRC_UTIL_SERIALIZATION_H_
#define SRC_UTIL_SERIALIZATION_H_

#include <string>

namespace util {
namespace serialization {

int CreateOrEraseFileForWrite(const std::string& file_name);
int OpenGeneralFileForRead(const std::string& file_name);
int OpenFileForRead(const std::string& file_name);
std::string GetFolderName();
std::string GetFullFolderPath();

}  // namespace serialization
}  // namespace util
#endif  // SRC_UTIL_SERIALIZATION_H_
