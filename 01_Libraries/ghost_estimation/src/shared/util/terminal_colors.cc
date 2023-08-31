//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    terminal_utils.cc
\brief   Subroutines to color stdout
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <stdio.h>
#include <vector>
#include "terminal_colors.h"

using std::vector;
using std::size_t;

namespace terminal_colors {

void ColorTerminal(unsigned char fg,
                   unsigned char bg,
                   std::vector<unsigned char> attr) {
  printf("\033[%d;%d;", bg + 40, fg + 30);
  for (std::size_t i = 0; i < attr.size(); ++i) {
    printf("%d", attr[i]);
    if (i + 1 < attr.size()) {
      printf(";");
    } else {
      printf("m");
    }
  }
  fflush(stdout);
}

void ColorTerminal(unsigned char fg,
                   unsigned char bg,
                   unsigned char attr) {
  printf("\033[%d;%d;%dm", attr, bg + 40, fg + 30);
  fflush(stdout);
}

void ColorTerminal(unsigned char fg,
                   unsigned char bg) {
  printf("\033[%d;%dm", bg + 40, fg + 30);
  fflush(stdout);
}

void ColorTerminal(unsigned char fg) {
  printf("\033[%dm", fg + 30);
  fflush(stdout);
}

void ResetTerminal() {
  printf("\033[0m");
  fflush(stdout);
}

}  // namespace terminal_colors
