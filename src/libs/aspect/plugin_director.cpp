
/***************************************************************************
 *  plugin_director.cpp - Plugin director aspect for Fawkes
 *
 *  Created: Thu Feb 12 12:02:58 2009
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/plugin_director.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PluginDirectorAspect <aspect/plugin_director.h>
 * Thread aspect to access the PluginManager.
 * Give this aspect to your thread to gain access to the plugin manager.
 * The plugin manager can be used to load and unload plugins. Use this
 * carefully, as it can interfere with the internals of the runtime system.
 *
 * Note: This aspect can only be added for continuous threads, as plugins
 * cannot be loaded while the main loop is running.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** @var PluginManager *  PluginDirectorAspect::plugin_manager
 * This is the member used to access the PluginManager.
 * The plugin manager will remain valid for the whole lifetime of the
 * thread.
 */

/** Constructor. */
PluginDirectorAspect::PluginDirectorAspect()
{
  add_aspect("PluginDirectorAspect");
}

/** Virtual empty Destructor. */
PluginDirectorAspect::~PluginDirectorAspect()
{
}


/** Set the PluginManager.
 * It is guaranteed that this is called for a configurable thread before
 * Thread::start() is called (when running regularly inside Fawkes).
 * @param manager PluginManager instance to use
 */
void
PluginDirectorAspect::init_PluginDirectorAspect(PluginManager *manager)
{
  plugin_manager = manager;
}

} // end namespace fawkes
