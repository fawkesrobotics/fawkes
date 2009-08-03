
/***************************************************************************
 *  plugin.cpp - Interface for a Fawkes plugin, some method have a base
 *               implementation that can be overridden in special situations.
 *
 *  Created: Sat Sep 16 17:04:55 2006
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <core/plugin.h>
#include <core/threading/thread.h>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class Plugin <core/plugin.h>
 * Plugin interface class.
 * Derive this class to create a new Fawkes plugin. There is not much that
 * you have to do to get a basic plugin working. The base plugin will already
 * handle all the important details.
 *
 * To implement a plugin create a new class that inherits from Plugin. Call
 * the Plugin constructor with the proper parameters in your derivate's
 * constructor. Then in your constructor fill the thread_list member with
 * the threads that your plugin needs. Instantiate all threads that your
 * plugin may ever need during its lifetime, creating (blocked timing)
 * threads during the life time of a plugin is not allowed. After the
 * constructor the thread list has to be considered to be sealed.
 * At the end of the file add a line like
 * @code
 * EXPORT_PLUGIN(PluginClass)
 * @endcode
 * where PluginClass is the class name of your plugin. This will create the
 * proper glue code to make this class loadable as plugin by Fawkes.
 *
 * @see ThreadList
 *
 * @ingroup FCL
 * @author Tim Niemueller
 */

/* IMPLEMENTOR'S NOTE:
 * I'm aware that we do not link libfawkescore against libfawkesconfig, so why
 * do we put the reference to fawkes::Configuration here? Two things to consider:
 * 1. We only pass through the pointer, nothing more. We do not need to know about
 * the declaration or definition!
 * 2. We want to keep plugin.(h|cpp) in libfawkescore, rather than in
 * libfawkesplugin to keep the minimum requirements for plugins low.
 */

/** Constructor.
 * Pass the name of your plugin to this ctor.
 * @param config configuration
 */
Plugin::Plugin(Configuration *config)
{
  this->config = config;
  _name_alloc = NULL;
  _name = "PluginNameNotSet";
}

/** Virtual destructor */
Plugin::~Plugin()
{
  for (ThreadList::iterator i = thread_list.begin(); i != thread_list.end(); ++i) {
    delete *i;
  }
  if (_name_alloc) free(_name_alloc);
}


/** Determines if the plugin can be unloaded.
 * This method tells the plugin loader if this plugin can be unloaded. Use
 * with care. No plugins but core plugins should return true. Only override
 * this if needed. The default behaviour if not overridden is to return false.
 * @return true, if the plugin cannot be unloaded, false otherwise. The default
 * implementation returns false.
 */
bool
Plugin::persistent()
{
  return false;
}

/** Get a list of threads.
 * This function shall return a list of threads. See the FawkesThreadManager
 * for supported special types of threads. This method is called only once
 * right after the plugin has been initialised. You may not change the
 * list afterwards by adding or removing threads. Especially you may not delete
 * the threads!
 * @return list of threads.
 */
ThreadList &
Plugin::threads()
{
  return thread_list;
}


/** Set plugin name.
 * Set the name of this plugin. This method should never be called from user code,
 * but only from the plugin loding/initialization system.
 * @param name new name
 */
void
Plugin::set_name(const char *name)
{
  if ( _name_alloc )  free(_name_alloc);

  thread_list.set_name("%s", name);

  _name_alloc = strdup(name);
  if ( ! _name_alloc ) {
    // We do not want to throw an exception here
    _name = "OutOfMemoryForPluginName";
  } else {
    _name = _name_alloc;
  }
}


/** Get the name of the plugin.
 * @return name of the plugin
 */
const char *
Plugin::name() const
{
  return _name;
}


} // end namespace fawkes
