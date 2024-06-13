
/***************************************************************************
 *  string_conversions.h - string conversions
 *
 *  Created: Thu Oct 12 12:03:49 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef _UTILS_MISC_STRINGTOOLS_H_
#define _UTILS_MISC_STRINGTOOLS_H_

#include <string>
#include <vector>

namespace fawkes {

class StringConversions
{
public:
	static std::string to_upper(std::string str);
	static std::string to_lower(std::string str);

	static std::string to_string(unsigned int i);
	static std::string to_string(int i);
	static std::string to_string(long int i);
	static std::string to_string(float f);
	static std::string to_string(double d);
	static std::string to_string(bool b);

	/** No-op conversion of string.
   * @param s value to convert
   * @return string the very same string
   */
	static std::string
	to_string(const std::string &s)
	{
		return s;
	}

	static unsigned int to_uint(std::string s);
	static int          to_int(std::string s);
	static long         to_long(std::string s);
	static float        to_float(std::string s);
	static double       to_double(std::string s);
	static bool         to_bool(std::string s);

	static void        trim_inplace(std::string &s);
	static std::string trim(const std::string &s);

	static std::string              resolve_path(std::string s);
	static std::vector<std::string> resolve_paths(std::vector<std::string> s);

private:
	// may not be instantiated!
	StringConversions() {};
};

} // end namespace fawkes

#endif
