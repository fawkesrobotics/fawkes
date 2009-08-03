
/***************************************************************************
 *  string_compare.h - Fawkes string compare utils
 *
 *  Created: Fri May 11 23:39:18 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_MISC_STRING_COMPARE_H_
#define __UTILS_MISC_STRING_COMPARE_H_

namespace fawkes {


class StringEquality
{
 public:
  bool operator()(const char *__s1, const char *__s2) const;
};

class StringLess
{
 public:
  bool operator()(const char *__s1, const char *__s2) const;
};


} // end namespace fawkes

#endif
