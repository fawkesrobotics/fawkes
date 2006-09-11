
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

#include <iostream>

PluginLoader::PluginLoader(const char *plugin_base_dir)
{
  mm = ModuleManagerFactory::getInstance(ModuleManagerFactory::MMT_DL, plugin_base_dir);
  msg_prefix = std::cblue + "PluginLoader: " + std::cnormal;
}

PluginLoader::~PluginLoader()
{
  delete mm;
}


bool
PluginLoader::load(std::string plugin_name, Plugin *& plugin)
{

  if ( name_plugin_map.find(plugin_name) != name_plugin_map.end() ) {
    plugin = name_plugin_map[plugin_name];
    return true;
  }

  // This is dependent on the system architecture!
  std::string module_name = plugin_name + "." + mm->getModuleFileExtension();

  Module *pm = mm->openModule(module_name);

  if ( pm == NULL ) {
    // we could NOT open the plugin module
    std::cout << msg_prefix << "Could not open the plugin module" << std::endl;
    return false;
  }

  if ( ! pm->hasSymbol("plugin_factory") ) {
    return false;
  }

  PluginFactoryFunc pff = (PluginFactoryFunc)pm->getSymbol("plugin_factory");

  Plugin *p = pff();
  if ( p == NULL ) {
    return false;
  }

  plugin = p;

  plugin_module_map[p] = pm;
  name_plugin_map[plugin_name] = p;
  plugin_name_map[p] = plugin_name;

  return true;
}


void
PluginLoader::unload(Plugin *plugin)
{
  if ( plugin_module_map.find(plugin) != plugin_module_map.end() ) {
    
    PluginDestroyFunc pdf = (PluginDestroyFunc)plugin_module_map[plugin]->getSymbol("plugin_destroy");
    if ( pdf != NULL ) {
      pdf(plugin);
    }
    mm->closeModule(plugin_module_map[plugin]);
    plugin_module_map.erase(plugin);

    name_plugin_map.erase(plugin_name_map[plugin]);
    plugin_name_map.erase(plugin);
  }
}
