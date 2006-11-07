
/***************************************************************************
 *  plugin_loader.cpp - Loads plugins from .so shared objects
 *
 *  Generated: Wed Aug 23 15:23:36 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <utils/plugin/plugin_loader.h>
#include <utils/system/console_colors.h>
#include <utils/system/dynamic_module/module_manager_factory.h>
#include <utils/system/dynamic_module/module_manager.h>
#include <utils/system/dynamic_module/module.h>

#include <iostream>

#include <map>
#include <string>

/// @cond QA
class PluginLoaderData
{
 public:
  ModuleManager  *mm;
  std::map< Plugin *, Module * >    plugin_module_map;
  std::map< std::string, Plugin * > name_plugin_map;
  std::map< Plugin *, std::string > plugin_name_map;

  std::string msg_prefix;
};
/// @endcond

/** @class PluginNotFoundException utils/plugin/plugin_loader.h
 * This exception is thrown if the requested plugin could not be loaded.
 * @ingroup Exceptions
 */

/** Constructor.
 * @param plugin_type type of the plugin
 * @param add_msg additional message, reason for problem
 */
PluginNotFoundException::PluginNotFoundException(const char *plugin_type,
						 const char *add_msg)
  : Exception()
{
  append("Plugin of type '%s' could not be found", plugin_type);
  append(add_msg);
}


/** @class PluginLoader utils/plugin/plugin_loader.h
 * This class manages plugins.
 * With this class plugins can be loaded and unloaded. Information is
 * kept about active plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor
 * @param plugin_base_dir The base directory where to search for the shared
 * libraries which contain the plugins
 */
PluginLoader::PluginLoader(const char *plugin_base_dir)
{
  d = new PluginLoaderData();
  d->mm = ModuleManagerFactory::getInstance(ModuleManagerFactory::MMT_DL, plugin_base_dir);
  d->msg_prefix = std::cblue + "PluginLoader: " + std::cnormal;
}

/** Destructor */
PluginLoader::~PluginLoader()
{
  delete d->mm;
  delete d;
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
 * @exception PluginNotFoundException thrown if plugin could not be found
 * (loading failed)
 */
Plugin *
PluginLoader::load(const char *plugin_name)
{

  std::string pn = plugin_name;

  if ( d->name_plugin_map.find(pn) != d->name_plugin_map.end() ) {
    return d->name_plugin_map[pn];
  }

  // This is dependent on the system architecture!
  std::string module_name = pn + "." + d->mm->getModuleFileExtension();

  Module *pm = d->mm->openModule(module_name);

  if ( pm == NULL ) {
    // we could NOT open the plugin module
    // std::cout << d->msg_prefix << "Could not open the plugin module" << std::endl;
    throw PluginNotFoundException(plugin_name,
				  "Could not open plugin module");
  }

  if ( ! pm->hasSymbol("plugin_factory") ) {
    throw PluginNotFoundException(plugin_name, "Symbol 'plugin_factory' not found");
  }

  PluginFactoryFunc pff = (PluginFactoryFunc)pm->getSymbol("plugin_factory");

  Plugin *p = pff();
  if ( p == NULL ) {
    throw PluginNotFoundException(plugin_name, "Plugin could not be instantiated");
  }

  d->plugin_module_map[p] = pm;
  d->name_plugin_map[pn] = p;
  d->plugin_name_map[p] = pn;

  return p;
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
    
    PluginDestroyFunc pdf = (PluginDestroyFunc)d->plugin_module_map[plugin]->getSymbol("plugin_destroy");
    if ( pdf != NULL ) {
      pdf(plugin);
    }
    d->mm->closeModule(d->plugin_module_map[plugin]);
    d->plugin_module_map.erase(plugin);

    d->name_plugin_map.erase(d->plugin_name_map[plugin]);
    d->plugin_name_map.erase(plugin);
  }
}
