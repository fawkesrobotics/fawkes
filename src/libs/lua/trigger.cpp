
/***************************************************************************
 *  trigger.cpp - Fawkes Lua Trigger Support
 *
 *  Created: Mon Jun 23 10:28:05 2008
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

#include <lua/trigger.h>
#include <lua/context.h>

#include <core/exceptions/system.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>

namespace fawkes {

/** @class LuaTriggerManager <lua/trigger.h>
 * Lua Trigger Manager.
 * This class interfaces with a trigger sub-system running inside Lua (with
 * the trigger system provided by Fawkes' Lua packages).
 * @author Tim Niemueller
 */

/** Constructor.
 * @param lua Lua context to use that has a running trigger system
 * @param trigger_var the name of the (global) variable pointing to the
 * trigger system
 */
LuaTriggerManager::LuaTriggerManager(LuaContext *lua, const char *trigger_var)
{
  __lua         = lua;
  __trigger_var = strdup(trigger_var);
}


/** Destructor. */
LuaTriggerManager::~LuaTriggerManager()
{
  free(__trigger_var);
}


/** Cause a trigger event.
 * @param event name of the event to trigger
 * @param param_format a format string for a string passed plain as Lua code
 * in the trigger() function call as second argument. The code executed looks
 * like "lua_trigger_var:trigger(event, string)" with string being what you
 * pass, so it can be any number of arguments, for instance you could pass
 * @code
 * {x=%f, y=%f}
 * @endcode
 * which would result in a table set with the two floats you provide in the
 * ellipsis.
 */
void
LuaTriggerManager::trigger(const char *event, const char *param_format, ...)
{
  va_list args;
  char *params = NULL;
  if ( param_format ) {
    va_start(args, param_format);
    if (vasprintf(&params, param_format, args) == -1) {
      throw OutOfMemoryException("Lua trigger: Could not allocate param string");
    }
    va_end(args);

    __lua->do_string("%s:trigger(\"%s\", %s)", __trigger_var, event, params);
    free(params);
  } else {
    __lua->do_string("%s:trigger(\"%s\")", __trigger_var, event);
  }
}

} // end of namespace fawkes
