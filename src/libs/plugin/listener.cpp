
/***************************************************************************
 *  listener.cpp - Fawkes plugin manager listener
 *
 *  Created: Thu Feb 12 13:16:28 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <plugin/listener.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PluginManagerListener <plugin/listener.h>
 * PluginManager listener.
 * The PluginManagerListener interface can be implemented to register to
 * the PluginManager to receive notifications if a plugin is loaded or unloaded.
 * @author Tim Niemueller
 *
 * @fn virtual void PluginManagerListener::plugin_loaded(const char *plugin_name)
 * Plugin loaded event.
 * @param plugin_name name of the plugin that has just been loaded
 *
 * @fn virtual void PluginManagerListener::plugin_unloaded(const char *plugin_name)
 * Plugin unloaded event.
 * @param plugin_name name of the plugin that has just been unloaded
 */

/** Virtual empty destructor. */
PluginManagerListener::~PluginManagerListener()
{
}

} // end namespace fawkes
