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
\file    terminal_colors.h
\brief   Subroutines to color stdout
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <vector>

#ifndef TERMINAL_COLORS
#define TERMINAL_COLORS


namespace terminal_colors {
static const unsigned char TERMINAL_ATTR_RESET         = 0;
static const unsigned char TERMINAL_ATTR_BRIGHT        = 1;
static const unsigned char TERMINAL_ATTR_DIM           = 2;
static const unsigned char TERMINAL_ATTR_ITALIC        = 3;
static const unsigned char TERMINAL_ATTR_UNDERLINE     = 4;
static const unsigned char TERMINAL_ATTR_SLOWBLINK     = 5;
static const unsigned char TERMINAL_ATTR_FASTBLINK     = 6;
static const unsigned char TERMINAL_ATTR_INVERT        = 7;
static const unsigned char TERMINAL_ATTR_CONCEAL       = 8;
static const unsigned char TERMINAL_ATTR_CROSSEDOUT    = 9;

static const unsigned char TERMINAL_COL_BLACK          = 0;
static const unsigned char TERMINAL_COL_RED            = 1;
static const unsigned char TERMINAL_COL_GREEN          = 2;
static const unsigned char TERMINAL_COL_YELLOW         = 3;
static const unsigned char TERMINAL_COL_BLUE           = 4;
static const unsigned char TERMINAL_COL_MAGENTA        = 5;
static const unsigned char TERMINAL_COL_CYAN           = 6;
static const unsigned char TERMINAL_COL_WHITE          = 7;

void ColorTerminal(unsigned char fg,
                   unsigned char bg,
                   std::vector<unsigned char> attr);

void ColorTerminal(unsigned char fg,
                   unsigned char bg,
                   unsigned char attr);

void ColorTerminal(unsigned char fg,
                   unsigned char bg);

void ColorTerminal(unsigned char fg);

void ResetTerminal();

}  // namespace terminal_colors

#endif //TERMINAL_COLORS
