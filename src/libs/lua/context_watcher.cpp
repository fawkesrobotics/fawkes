
/***************************************************************************
 *  context_watcher.cpp - Fawkes Lua Context Watcher
 *
 *  Created: Thu Jan 01 15:18:14 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <lua/context_watcher.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LuaContextWatcher <lua/context_watcher.h>
 * Lua context watcher.
 * This interface allows for notification of LuaContext events.
 * @author Tim Niemueller
 *
 * @fn void LuaContextWatcher::lua_restarted(LuaContext *context) = 0
 * Lua restart event.
 * This is called when the LuaContext has been restarted, for example when file
 * watching is enabled and a file changed. It is executed after all packages have
 * been loaded and variables have been set, but before the start script is run.
 * The implementation may throw an exception if anything prevents it from using
 * the new context properly.
 * @param context This is a temporary LuaContext that is valid as long as the
 * method is executed. It is a wrapper context around the new Lua state, just before
 * the start script is run and it the calling context is switched to the new state
 * (if no error occurs).
 */

/** Virtual empty destructor. */
LuaContextWatcher::~LuaContextWatcher()
{
}


} // end of namespace fawkes
