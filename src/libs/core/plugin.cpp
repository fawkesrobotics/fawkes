
/***************************************************************************
 *  plugin.cpp - Interface for a Fawkes plugin, some method have a base
 *               implementation that can be overridden in special situations.
 *
 *  Generated: Sat Sep 16 17:04:55 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <core/plugin.h>
#include <core/threading/thread.h>
#include <cstring>

/** @class Plugin core/plugin.h
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


/** @typedef void      (* PluginDestroyFunc)  (Plugin *)
 * Plugin destructor function for the shared library.
 * Declare and define this function exactly like this:
 *
 * @code
 * extern "C"
 * void
 * plugin_destroy(Plugin *plugin)
 * {
 *   delete plugin;
 * }
 * @endcode
 * Do not change the type or name of this function or type of arguments
 * of this function!
 *
 * There is a convenience macro that you can use to create a function like
 * the one above called PLUGIN_FACTORY(class_name).
 *
 * It is recommended to use the EXPORT_PLUGIN(class_name) macro which will
 * create proper plugin factory and destroy functions.
 *
 * @relates Plugin
 */

/** @typedef Plugin *  (* PluginFactoryFunc)  (void);
 * Plugin loader function for the shared library
 * Declare and define this function exactly like this:
 *
 * @code
 * extern "C"
 * Plugin *
 * plugin_factory()
 * {
 *  return new MightyPlugin();
 * }
 * @endcode
 * Do not change the type or name of this function, replace MightyPlugin
 * with the name of your plugin derivative.
 *
 * There is a convenience macro that you can use to create a function like
 * the one above called PLUGIN_DESTROY(class_name).
 *
 * It is recommended to use the EXPORT_PLUGIN(class_name) macro which will
 * create proper plugin factory and destroy functions.
 *
 * @relates Plugin
 */


/** Constructor.
 * Pass the appropriate plugin type and the name of your plugin to this ctor.
 * @param plugin_type type of the plugin
 * @param plugin_name name of the plugin
 */
Plugin::Plugin(PluginType plugin_type, const char *plugin_name)
  : thread_list(plugin_name)
{
  _type = plugin_type;
  _name_alloc = strdup(plugin_name);
  if ( ! _name_alloc ) {
    // We do not want to throw an exception here
    _name = "OutOfMemoryForPluginName";
  } else {
    _name = _name_alloc;
  }
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


/** Get the type of the plugin.
 * @return type of the plugin
 */
Plugin::PluginType
Plugin::type() const
{
  return _type;
}

/** Get the name of the plugin.
 * @return name of the plugin
 */
const char *
Plugin::name() const
{
  return _name;
}
