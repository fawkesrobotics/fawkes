
/***************************************************************************
 *  pipeline.h - This header defines a image processing pipeline for the
 *               front camera
 *
 *  Generated: Wed Jun 15 16:30:22 2005 (from FireVision)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __UTILS_CONSOLE_COLORS_H_
#define __UTILS_CONSOLE_COLORS_H_

#include <string>

namespace fawkes {

  /** Print black on console. */
  static const char *c_black = "\033[0;30m";
  /** Print black on console. */
  static std::string cblack = c_black;

  /** Print dark gray on console. */
  static const char *c_darkgray = "\033[1;30m";
  /** Print dark gray on console. */
  static std::string cdarkgray = c_darkgray;

  /** Print red on console. */
  static const char *c_red = "\033[0;31m";
  /** Print red on console. */
  static std::string cred = c_red;

  /** Print light red on console. */
  static const char *c_lightred = "\033[1;31m";
  /** Print light red on console. */
  static std::string clightred = c_lightred;

  /** Print green on console. */
  static const char *c_green = "\033[0;32m";
  /** Print green on console. */
  static std::string cgreen = c_green;

  /** Print light green on console. */
  static const char *c_lightgreen = "\033[1;32m";
  /** Print light green on console. */
  static std::string clightgreen = c_lightgreen;

  /** Print brown on console. */
  static const char *c_brown = "\033[0;33m";
  /** Print brown on console. */
  static std::string cbrown = c_brown;

  /** Print yellow on console. */
  static const char *c_yellow = "\033[1;33m";
  /** Print yellow on console. */
  static std::string cyellow = c_yellow;

  /** Print blue on console. */
  static const char *c_blue = "\033[0;34m";
  /** Print blue on console. */
  static std::string cblue = c_blue;

  /** Print light blue on console. */
  static const char *c_lightblue = "\033[1;34m";
  /** Print light blue on console. */
  static std::string clightblue = c_lightblue;

  /** Print purple on console. */
  static const char *c_purple = "\033[0;35m";
  /** Print purple on console. */
  static std::string cpurple = c_purple;

  /** Print light purple on console. */
  static const char *c_lightpurple = "\033[1;35m";
  /** Print light purple on console. */
  static std::string clightpurple = c_lightpurple;

  /** Print cyan on console. */
  static const char *c_cyan = "\033[0;36m";
  /** Print cyan on console. */
  static std::string ccyan = c_cyan;

  /** Print light cyan on console. */
  static const char *c_lightcyan = "\033[1;36m";
  /** Print light cyan on console. */
  static std::string clightcyan = c_lightcyan;

  /** Print light gray on console. */
  static const char *c_lightgray = "\033[2;37m";
  /** Print light gray on console. */
  static std::string clightgray = c_lightgray;

  /** Print white on console. */
  static const char *c_white = "\033[1;37m";
  /** Print white on console. */
  static std::string cwhite = c_white;

  /** Print normal on console, without colors, depends on console settings. */
  static const char *c_normal = "\033[0;39m";
  /** Print normal on console, without colors, depends on console settings. */
  static std::string cnormal = c_normal;

}

#endif
