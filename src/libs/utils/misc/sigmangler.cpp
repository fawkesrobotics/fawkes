
/***************************************************************************
 *  sigmangler.cpp - C++ Signature Mangler
 *
 *  Created: Sun Apr 20 11:03:57 2008 (GO2008, day 2)
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <utils/misc/sigmangler.h>

#include <string>
#include <cstring>


namespace fawkes {


/** @class CppSignatureMangler <utils/misc/sigmangler.h>
 * C++ signature mangling.
 * Simple methods to process C++ signatures. It is not true signature handling but
 * rather hacking it to useful output for current GCC (4.x).
 * @author Tim Niemueller
 */

/** Strip numbers at the beginning of the class type.
 * This has been implemented by observations of C++ class names as returned by GCC's
 * typeid operator.
 * @param type type name to strip
 * @return stripped class type, use delete to free it after you are done
 */
char *
CppSignatureMangler::strip_class_type(const char *type)
{
  std::string t = type;
  t = t.substr( t.find_first_not_of("0123456789") );
  char *rv = new char[t.length() + 1];
  strcpy(rv, t.c_str());
  return rv;
}


} // end namespace fawkes
