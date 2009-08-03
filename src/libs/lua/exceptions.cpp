
/***************************************************************************
 *  exceptions.h - Lua related exceptions
 *
 *  Created: Mon Jun 23 10:28:58 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <lua/exceptions.h>

namespace fawkes {

/** @class LuaRuntimeException exceptions.h <lua/exceptions.h>
 * Lua runtime exception.
 * Thrown if a runtime error occurs while executing Lua code.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param what in what?
 * @param errmsg error message
 */
LuaRuntimeException::LuaRuntimeException(const char *what, const char *errmsg)
  : Exception("Lua runtime error (in '%s'): %s", what, errmsg)
{
}


/** @class LuaErrorException exceptions.h <lua/exceptions.h>
 * Lua error exception.
 * Thrown if a runtime error occurs while executing an error handler.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param what in what?
 * @param errmsg error message
 */
LuaErrorException::LuaErrorException(const char *what, const char *errmsg)
  : Exception("Lua error while running error handler (in '%s'): %s", what, errmsg)
{
}

} // end of namespace fawkes
