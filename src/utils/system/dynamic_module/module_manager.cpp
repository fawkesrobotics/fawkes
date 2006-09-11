
/***************************************************************************
 *  module_manager.cpp - manager for modules (i.e. shared objects)
 *
 *  Generated: Wed Aug 23 16:19:41 2006
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

#include <utils/system/dynamic_module/module_manager.h>


ModuleManager::ModuleManager(std::string module_base_dir)
{
  modules.clear();
  this->module_base_dir = module_base_dir;
}


Module *
ModuleManager::openModule(std::string filename)
{
  if ( modules.find(filename) != modules.end() ) {
    modules[filename]->ref();
    return modules[filename];
  } else {
    Module *module = new Module(module_base_dir + filename);
    if ( module->open() ) {
      modules[filename] = module;
      return module;
    } else {
      delete module;
      return NULL;
    }
  }
}


void
ModuleManager::closeModule(Module *module)
{
  for (mit = modules.begin(); mit != modules.end(); ++mit) {
    if ( (*mit).second == module ) {
      (*mit).second->unref();
      if ((*mit).second->notref()) {
	delete (*mit).second;
	modules.erase( (*mit).first );
      }
    }
  }
}


void
ModuleManager::closeModule(std::string filename)
{
  if ( modules.find(filename) != modules.end() ) {
    modules[filename]->unref();
    if (modules[filename]->notref()) {
      delete modules[filename];
      modules.erase( filename );
    }
  }
}


std::string
ModuleManager::getModuleFileExtension()
{
  return Module::getFileExtension();
}
