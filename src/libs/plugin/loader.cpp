
/***************************************************************************
 *  loader.cpp - Loads plugins from .so shared objects
 *
 *  Created: Wed Aug 23 15:23:36 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <plugin/loader.h>

#include <utils/system/dynamic_module/module_manager_factory.h>
#include <utils/system/dynamic_module/module_manager.h>
#include <utils/system/dynamic_module/module.h>

#include <map>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond QA
class PluginLoaderData
{
 public:
  ModuleManager  *mm;
  std::map< Plugin *, Module * >    plugin_module_map;
  std::map< std::string, Plugin * > name_plugin_map;
  std::map< Plugin *, std::string > plugin_name_map;
};
/// @endcond

/** @class PluginLoadException <plugin/loader.h>
 * This exception is thrown if the requested plugin could not be loaded.
 */

/** Constructor.
 * @param plugin name of the plugin that caused the exception
 * @param message message of exception
 */
PluginLoadException::PluginLoadException(const char *plugin, const char *message)
  : Exception(), __plugin_name(plugin)
{
  append("Plugin '%s' could not be loaded: %s", plugin, message);
}


/** Destructor. */
PluginLoadException::~PluginLoadException() throw()
{
}

/** Constructor.
 * @param plugin name of the plugin that caused the exception
 * @param message message of exception
 * @param e exception to copy further messages from
 */
PluginLoadException::PluginLoadException(const char *plugin, const char *message,
					 Exception &e)
  : Exception(), __plugin_name(plugin)
{
  append("Plugin '%s' could not be loaded: %s", plugin, message);
  copy_messages(e);
}

/** Get name of plugin which failed to load.
 * @return plugin name
 */
std::string
PluginLoadException::plugin_name() const
{
  return __plugin_name;
}


/** @class PluginUnloadException <plugin/loader.h>
 * This exception is thrown if the requested plugin could not be unloaded.
 */

/** Constructor.
 * @param plugin_name name of the plugin
 * @param add_msg additional message, reason for problem
 */
PluginUnloadException::PluginUnloadException(const char *plugin_name,
					     const char *add_msg)
  : Exception()
{
  append("Plugin '%s' could not be unloaded", plugin_name);
  append(add_msg);
}


/** @class PluginLoader <plugin/loader.h>
 * This class manages plugins.
 * With this class plugins can be loaded and unloaded. Information is
 * kept about active plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor
 * @param plugin_base_dir The base directory where to search for the shared
 * libraries which contain the plugins
 * @param config Fawkes configuration
 */
PluginLoader::PluginLoader(const char *plugin_base_dir, Configuration *config)
{
  d = new PluginLoaderData();
  __config = config;
  d->mm = ModuleManagerFactory::getInstance(ModuleManagerFactory::MMT_DL, plugin_base_dir);
}

/** Destructor */
PluginLoader::~PluginLoader()
{
  delete d->mm;
  delete d;
}


Module *
PluginLoader::open_module(const char *plugin_name)
{
  std::string module_name = std::string(plugin_name) + "." + d->mm->get_module_file_extension();

  try {
    return d->mm->open_module(module_name.c_str());
  } catch (ModuleOpenException &e) {
    throw PluginLoadException(plugin_name, "failed to open module", e);
  }
}


Plugin *
PluginLoader::create_instance(const char *plugin_name, Module *module)
{
  if ( ! module->has_symbol("plugin_factory") ) {
    throw PluginLoadException(plugin_name, "Symbol 'plugin_factory' not found. Forgot EXPORT_PLUGIN?");
  }
  if ( ! module->has_symbol("plugin_description") ) {
    throw PluginLoadException(plugin_name, "Symbol 'plugin_description' not found. Forgot PLUGIN_DESCRIPTION?");
  }

  PluginFactoryFunc pff = (PluginFactoryFunc)module->get_symbol("plugin_factory");
  Plugin *p = NULL;

  p = pff(__config);
  if ( p == NULL ) {
    throw PluginLoadException(plugin_name, "Plugin could not be instantiated");
  } else {
    p->set_name(plugin_name);
  }

  return p;
}


/** Load a specific plugin
 * The plugin loader is clever and guarantees that every plugin is only
 * loaded once (as long as you use only one instance of the PluginLoader,
 * using multiple instances is discouraged. If you try to open a plugin
 * a second time it will return the
 * very same instance that it returned on previous load()s.
 * @param plugin_name The name of the plugin to be loaded, the plugin name has to
 * correspond to a plugin name and the name of the shared object that will
 * be opened for this plugin (for instance on Linux systems opening the
 * plugin test_plugin will look for plugin_base_dir/test_plugin.so)
 * @return Returns a pointer to the opened plugin.  Do not under any
 * circumstances delete this object, use unload() instead! Since the delete
 * operator could be overloaded this would result in memory chaos.
 * @exception PluginLoadException thrown if plugin could not be loaded
 * @exception ModuleOpenException passed along from module manager
 */
Plugin *
PluginLoader::load(const char *plugin_name)
{
  std::string pn = plugin_name;

  if ( d->name_plugin_map.find(pn) != d->name_plugin_map.end() ) {
    return d->name_plugin_map[pn];
  }

  try {
    Module *module = open_module(plugin_name);
    Plugin *p = create_instance(plugin_name, module);

    d->plugin_module_map[p] = module;
    d->name_plugin_map[pn]  = p;
    d->plugin_name_map[p]   = pn;

    return p;
  } catch (PluginLoadException &e) {
    throw;
  }
}


/** Get plugin description.
 * @param plugin_name name of the plugin
 * @return plugin description tring
 * @throw PluginLoadException thrown if opening the plugin fails
 */
std::string
PluginLoader::get_description(const char *plugin_name)
{
  Module *module = open_module(plugin_name);

  if ( ! module->has_symbol("plugin_description") ) {
    throw PluginLoadException(plugin_name, "Symbol 'plugin_description' not found. Forgot PLUGIN_DESCRIPTION?");
  }

  PluginDescriptionFunc pdf = (PluginDescriptionFunc)module->get_symbol("plugin_description");
  std::string rv = pdf();
  d->mm->close_module(module);

  return rv;
}


/** Check if a plugin is loaded.
 * @param plugin_name name of the plugin to chekc
 * @return true if the plugin is loaded, false otherwise
 */
bool
PluginLoader::is_loaded(const char *plugin_name)
{
  return ( d->name_plugin_map.find(plugin_name) != d->name_plugin_map.end() );
}


/** Unload the given plugin
 * This will unload the given plugin. The plugin is destroyed with the
 * proper destroy method from the shared object. The shared object is unloaded
 * after the destruction of the plugin.
 * Note that even though you may call load() multiple times per plugin you may
 * only unload() it once! Every further access will lead to a segmentation
 * fault.
 * Make sure that you have closed any resources claimed by the plugin like
 * threads, memory access etc.
 * @param plugin The plugin that has to be unloaded
 */
void
PluginLoader::unload(Plugin *plugin)
{
  if ( d->plugin_module_map.find(plugin) != d->plugin_module_map.end() ) {
    
    PluginDestroyFunc pdf = (PluginDestroyFunc)d->plugin_module_map[plugin]->get_symbol("plugin_destroy");
    if ( pdf != NULL ) {
      pdf(plugin);
    }
    d->mm->close_module(d->plugin_module_map[plugin]);
    d->plugin_module_map.erase(plugin);

    d->name_plugin_map.erase(d->plugin_name_map[plugin]);
    d->plugin_name_map.erase(plugin);
  }
}

} // end namespace fawkes
