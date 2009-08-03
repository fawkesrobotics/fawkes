
/***************************************************************************
 *  pathparser.h - Header for path parser
 *
 *  Created: Mon Jul 07 13:23:19 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_SYSTEM_PATHPARSER_H_
#define __UTILS_SYSTEM_PATHPARSER_H_

#include <vector>
#include <string>

namespace fawkes {

class PathParser : public std::vector<std::string>
{
 public:
  PathParser(std::string &path);
  PathParser(const char *path);

  void        print_debug();
  std::string path_as_string();

  bool        is_absolute() const;

 private:
  void        ctor(const std::string &path);

 private:
  bool __abs_path;
};

} // end namespace fawkes

#endif
