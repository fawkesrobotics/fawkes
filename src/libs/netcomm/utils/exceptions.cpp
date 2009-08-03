
/***************************************************************************
 *  exceptions.cpp - Generic network related exceptions
 *
 *  Created: Wed Nov 14 13:26:54 2007
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

#include <netcomm/utils/exceptions.h>

namespace fawkes {

/** @class ConnectionDiedException exceptions.h <netcomm/utils/exceptions.h>
 * Thrown if the connection died during an operation.
 * @ingroup NetComm
 */

/** Constructor.
 * @param format format of the message
 */
ConnectionDiedException::ConnectionDiedException(const char *format, ...)
  : Exception()
{
  va_list va;
  va_start(va, format);
  append(format, va);
  va_end(va);
}

} // end namespace fawkes
