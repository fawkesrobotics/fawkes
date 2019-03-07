
/***************************************************************************
 *  string_compare.cpp - Fawkes string compare utils
 *
 *  Created: Fri May 11 23:40:28 2007
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

#include <utils/misc/string_compare.h>

#include <cstring>

namespace fawkes {

/** @class StringEquality <utils/misc/string_compare.h>
 * String equality checker.
 * This is a valid binary predicate that can be used for instance hash maps
 * as the equality predicate.
 *
 * The only method is used to check whether two supplied strings are equal.
 * Uses strcmp for char arrays.
 *
 * @author Tim Niemueller
 */

/** Check equality of two strings.
 * @param s1_ first string
 * @param s2_ second string
 * @return true, if the strings are equal, false otherwise
 */
bool
StringEquality::operator()(const char *s1_, const char *s2_) const
{
	return (strcmp(s1_, s2_) == 0);
}

/** @class StringLess <utils/misc/string_compare.h>
 * String less than test.
 * This is a valid binary predicate that can be used for instance for maps
 * as the less predicate.
 *
 * The only method is used to check whether one supplied strings is less
 * then the other. Uses strcmp for char arrays.
 *
 * @author Tim Niemueller
 */

/** Check equality of two strings.
 * @param s1_ first string
 * @param s2_ second string
 * @return true, if the s1_ < s2_
 */
bool
StringLess::operator()(const char *s1_, const char *s2_) const
{
	return (strcmp(s1_, s2_) < 0);
}

} // end namespace fawkes
