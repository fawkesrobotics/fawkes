
/***************************************************************************
 *  software.cpp - Katana Controller exceptions
 *
 *  Created: Tue Jan 03 11:40:31 2012
 *  Copyright  2012  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#include "exception.h"

#include <cmath>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class KatanaNoSolutionException <plugins/katana/exception.h>
 * No joint configuration for desired target found.
 * @ingroup Exceptions
 */
/** Constructor
 * @param format message format, takes sprintf() parameters as variadic arguments
 */
KatanaNoSolutionException::KatanaNoSolutionException(const char *format, ...) throw()
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}

/** @class KatanaOutOfRangeException <plugins/katana/exception.h>
 * At least one motor is out of range.
 * @ingroup Exceptions
 */
/** Constructor
 * @param format message format, takes sprintf() parameters as variadic arguments
 */
KatanaOutOfRangeException::KatanaOutOfRangeException(const char *format, ...) throw()
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}

/** @class KatanaMotorCrashedException <plugins/katana/exception.h>
 * At least one motor crashed
 * @ingroup Exceptions
 */
/** Constructor
 * @param format message format, takes sprintf() parameters as variadic arguments
 */
KatanaMotorCrashedException::KatanaMotorCrashedException(const char *format, ...) throw()
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}

/** @class KatanaUnsupportedException <plugins/katana/exception.h>
 * Unsupported command.
 * @ingroup Exceptions
 */
/** Constructor
 * @param format message format, takes sprintf() parameters as variadic arguments
 */
KatanaUnsupportedException::KatanaUnsupportedException(const char *format, ...) throw()
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}


} // end namespace fawkes
