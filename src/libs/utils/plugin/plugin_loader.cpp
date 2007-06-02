
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
#include <utils/system/dynamic_module/module_manager_factory.h>
#include <utils/system/dynamic_module/module_manager.h>
#include <utils/system/dynamic_module/module.h>
#include <utils/plugin/load_thread.h>

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
  std::map<std::string, PluginLoadThread *> load_threads;
};
/// @endcond

/** @class PluginLoadException utils/plugin/plugin_loader.h
 * This exception is thrown if the requested plugin could not be loaded.
 */

/** Constructor.
 * @param plugin_name name of the plugin
 * @param add_msg additional message, reason for problem
 */
PluginLoadException::PluginLoadException(const char *plugin_name,
					 const char *add_msg)
  : Exception()
{
  append("Plugin '%s' could not be loaded", plugin_name);
  append(add_msg);
}


/** @class PluginUnloadException utils/plugin/plugin_loader.h
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
 * @exception ModuleOpenException passed along from module manager
 */
Plugin *
PluginLoader::load(const char *plugin_name)
{

  std::string pn = plugin_name;

  if ( d->name_plugin_map.find(pn) != d->name_plugin_map.end() ) {
    return d->name_plugin_map[pn];
  }

  PluginLoadThread plt(d->mm, plugin_name);
  plt.load_blocking();
  try {
    Plugin *p = plt.plugin();

    d->plugin_module_map[p] = plt.module();
    d->name_plugin_map[pn] = p;
    d->plugin_name_map[p] = pn;

    return p;
  } catch ( PluginLoadException &e) {
    e.append("PluginLoader failed to load plugin '%s' blocking", plugin_name);
    throw;
  }
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


/** Request deferred loading of plugin.
 * @param plugin_name name of the plugin to load deferred
 * @exception PluginLoadException thrown, if the plugin is already loaded or
 * a load request is already running
 */
void
PluginLoader::request_load(const char *plugin_name)
{
  std::string pn = plugin_name;

  if ( d->name_plugin_map.find(pn) != d->name_plugin_map.end() ) {
    throw PluginLoadException("Cannot request load for already loaded plugin");
  }

  if ( d->load_threads.find(plugin_name) != d->load_threads.end() ) {
    throw PluginLoadException("Load already requested");    
  }

  PluginLoadThread *plt = new PluginLoadThread(d->mm, plugin_name);
  plt->start();

  d->load_threads[plugin_name] = plt;
}


/** Check if load operation succeeded.
 * Checks if the load operation for the given plugin has been finished.
 * @param plugin_name name of the plugin to check
 * @return true, if the plugin has finished loading, false otherwise.
 * @exception PluginLoadException thrown if loading was not requested or
 * if the load failed.
 */
bool
PluginLoader::finished_load(const char *plugin_name)
{
  std::map<std::string, PluginLoadThread *>::iterator i;
  if ( (i = d->load_threads.find(plugin_name)) == d->load_threads.end() ) {
    PluginLoadException e("Plugin loading not requested");
    e.append("Loading of plugin '%s' was not requested");
    throw e;
  } else {
    return (*i).second->finished();
  }
}


/** Finish deferred loading.
 * Call this to finish a load operation. Check that the load operation
 * has finished with finished_load() before.
 * @param plugin_name name of the plugin to finish the load operation of
 * @return loaded plugin
 * @exception PluginLoadException thrown, if the loading was not requested,
 * or the loading has not finished or if the loading failed.
 */
Plugin *
PluginLoader::finish_deferred_load(const char *plugin_name)
{
  std::map<std::string, PluginLoadThread *>::iterator i;
  if ( (i = d->load_threads.find(plugin_name)) == d->load_threads.end() ) {
    PluginLoadException e("Plugin loading not requested");
    e.append("Loading of plugin '%s' was not requested");
    throw e;
  } else if ( ! (*i).second->finished()) {
    throw PluginLoadException("Plugin loading not finished");
  } else {
    try {
      Plugin *p = (*i).second->plugin();

      d->plugin_module_map[p]         = (*i).second->module();
      d->name_plugin_map[plugin_name] = p;
      d->plugin_name_map[p]           = plugin_name;

      (*i).second->join();
      delete (*i).second;
      d->load_threads.erase(i);

      return p;      
    } catch (Exception &e) {
      throw;
    }
  }  
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
