
/***************************************************************************
 *  pipeline.h - This header defines a image processing pipeline for the
 *               front camera
 *
 *  Generated: Wed Jun 15 16:30:22 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_UTILS_COLORS_H_
#define __FIREVISION_UTILS_COLORS_H_

#include <string>

namespace std {

  /** Print black on console */
  static string cblack("\033[0;30m");

  /** Print dark gray on console */
  static string cdarkgray("\033[1;30m");

  /** Print red on console */
  static string cred("\033[0;31m");

  /** Print light red on console */
  static string clightred("\033[1;31m");

  /** Print green on console */
  static string cgreen("\033[0;32m");

  /** Print light green on console */
  static string clightgreen("\033[1;32m");

  /** Print brown on console */
  static string cbrown("\033[0;33m");

  /** Print yellow on console */
  static string cyellow("\033[1;33m");

  /** Print blue on console */
  static string cblue("\033[0;34m");

  /** Print light blue on console */
  static string clightblue("\033[1;34m");

  /** Print purple on console */
  static string cpurple("\033[0;35m");

  /** Print light purple on console */
  static string clightpurple("\033[1;35m");

  /** Print cyan on console */
  static string ccyan("\033[0;36m");

  /** Print light cyan on console */
  static string clightcyan("\033[1;36m");

  /** Print light gray on console */
  static string clightgray("\033[0;37m");

  /** Print white on console */
  static string cwhite("\033[1;37m");

  /** Print normal on console, without colors, depends on console settings */
  static string cnormal("\033[0;39m");

}

#endif
