
/***************************************************************************
 *  exception.h - Katana Controller exceptions
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

#ifndef _PLUGINS_KATANA_EXCEPTION_H_
#define _PLUGINS_KATANA_EXCEPTION_H_

#include <core/exception.h>

namespace fawkes {

class KatanaNoSolutionException : public Exception
{
public:
	KatanaNoSolutionException(const char *format, ...) noexcept;
};

class KatanaOutOfRangeException : public Exception
{
public:
	KatanaOutOfRangeException(const char *format, ...) noexcept;
};

class KatanaMotorCrashedException : public Exception
{
public:
	KatanaMotorCrashedException(const char *format, ...) noexcept;
};

class KatanaUnsupportedException : public Exception
{
public:
	KatanaUnsupportedException(const char *format, ...) noexcept;
};

} // end namespace fawkes

#endif
