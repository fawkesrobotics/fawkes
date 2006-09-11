
/***************************************************************************
 *  plugin_loader.h - Loads plugins from .so shared objects
 *
 *  Generated: Wed Aug 23 15:18:13 2006
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

#ifndef __UTILS_PLUGIN_PLUGIN_LOADER_H_
#define __UTILS_PLUGIN_PLUGIN_LOADER_H_

#include <utils/plugin/plugin.h>
#include <utils/system/dynamic_module/module_manager.h>
#include <utils/system/dynamic_module/module.h>

#include <map>
#include <string>

/** This class manages loaded plugins
 */
class PluginLoader {
 public:

  /* Constructor
   * @param plugin_base_dir The base directory where to search for the shared
   * libraries which contain the plugins
   */
  PluginLoader(const char *plugin_base_dir);

  /** Destructor */
  ~PluginLoader();

  /** Load a specific plugin
   * The plugin loader is clever and guarantees that every plugin is only
   * loaded once (as long as you use only one instance of the PluginLoader,
   * using multiple instances is discouraged. If you try to open a plugin
   * a second time it will return the
   * very same instance that it returned on previous load()s.
   * @param plugin The name of the plugin to be loaded, the plugin name has to
   * correspond to a plugin name and the name of the shared object that will
   * be opened for this plugin (for instance on Linux systems opening the
   * plugin test_plugin will look for plugin_base_dir/test_plugin.so)
   * @param plugin This is a reference to a pointer to the plugin. If the
   * plugin has been loaded successfully (check the return value) plugin will
   * point to an instance of the Plugin sub-class. Do not under any
   * circumstances delete this object, use unload() instead! Since the delete
   * operator could be overloaded this would result in memory chaos.
   * @return Returns true on successful loading of the plugin, false otherwise
   */
  bool load(std::string plugin, Plugin *& plugin);

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
  void unload(Plugin *plugin);

 private:
  ModuleManager  *mm;
  std::map< Plugin *, Module * >  plugin_module_map;
  std::map< std::string, Plugin * > name_plugin_map;
  std::map< Plugin *, std::string > plugin_name_map;

  std::string msg_prefix;
};


#endif
