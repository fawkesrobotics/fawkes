/***************************************************************************
 *  exceptions.cpp - Fawkes tf exceptions
 *
 *  Created: Tue Oct 18 16:41:19 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include <tf/exceptions.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class TransformException
 * Base class for fawkes tf exceptions.
 */

/** Constructor. */
TransformException::TransformException()
  : Exception() {}

/** @class ConnectivityException
 * No connection between two frames in tree.
 * While looking for a connection between two frames in the transform
 * tree it was detected that there is no such connection.
 */

/** Constructor.
 * @param format format of explanatory message of the error, format
 * and parameters similar to sprintf.
 */
ConnectivityException::ConnectivityException(const char *format, ...)
  : TransformException()
{
  va_list args;
  va_start(args, format);
  append_nolock_va(format, args);
  va_end(args);
}

/** @class LookupException
 * A frame could not be looked up.
 * Thrown if you try to access a frame which is unknown to the
 * transforms system.
 */

/** Constructor.
 * @param format format of explanatory message of the error, format
 * and parameters similar to sprintf.
 */
LookupException::LookupException(const char *format, ...)
  : TransformException()
{
  va_list args;
  va_start(args, format);
  append_nolock_va(format, args);
  va_end(args);
}

/** @class ExtrapolationException
 * Request would have required extrapolation beyond current limits.
 */

/** Constructor.
 * @param format format of explanatory message of the error, format
 * and parameters similar to sprintf.
 */
ExtrapolationException::ExtrapolationException(const char *format, ...)
  : TransformException()
{
  va_list args;
  va_start(args, format);
  append_nolock_va(format, args);
  va_end(args);
}

/** @class InvalidArgumentException
 * Passed argument was invalid.
 * A typica problem is passing an uninitialized quaternion (0,0,0,0).
 */

/** Constructor.
 * @param format format of explanatory message of the error, format
 * and parameters similar to sprintf.
 */
InvalidArgumentException::InvalidArgumentException(const char *format, ...)
  : TransformException()
{
  va_list args;
  va_start(args, format);
  append_nolock_va(format, args);
  va_end(args);
}


/** @class DisabledException
 * The requested feature is disabled.
 */

/** Constructor.
 * @param format format of explanatory message of the error, format
 * and parameters similar to sprintf.
 */
DisabledException::DisabledException(const char *format, ...)
  : TransformException()
{
  va_list args;
  va_start(args, format);
  append_nolock_va(format, args);
  va_end(args);
}


} // end namespace tf
} // end namespace fawkes
