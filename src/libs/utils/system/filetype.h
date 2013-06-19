
/***************************************************************************
 *  filetype.h - little utility to decide on filetype
 *
 *  Generated: Tue Feb 23 13:49:38 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_FILETYPE_H_
#define __UTILS_FILETYPE_H_

#include <string>

namespace fawkes {

std::string filetype_file(const char *filename);
std::string filetype_file(int fd);
std::string mimetype_file(const char *filename);
std::string mimetype_file(int fd);

}

#endif
