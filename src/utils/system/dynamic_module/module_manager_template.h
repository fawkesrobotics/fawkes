
/***************************************************************************
 *  module_manager.h - manager for modules (i.e. shared objects)
 *
 *  Generated: Wed Aug 23 16:15:02 2006
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

#ifndef __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_MANAGER_TEMPLATE_H_
#define __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_MANAGER_TEMPLATE_H_

#include <utils/system/dynamic_module/module.h>
#include <utils/system/dynamic_module/module_manager.h>
#include <map>
#include <string>

/** Manager to load and unload modules, keeps track of loaded modules and
 * does not reload modules if they are already loaded
 * This implementation is a simple template that takes the Module
 * implementation class name as its template parameter
 */
template <class MODULE_CLASS>
class ModuleManagerTemplate : public ModuleManager {

 public:

  /** Constructor of NetworkManagerTemplate
   * @param module_base_dir The module basedir where to look for plugins
   */
  ModuleManagerTemplate(std::string module_base_dir = "")
    {
      modules.clear();
      this->module_base_dir = module_base_dir;
    }

  /** Open a module
   * @param filename The file name of the module that should be
   * opened. This filename is relative to the base dir given to the
   * constructor
   * @return Returns the module if the file was opened successfully
   * or NULL otherwise. Do NOT delete the module after usage but use
   * closeModule to close it.
   */
  MODULE_CLASS *  openModule(std::string filename)
  {
    if ( modules.find(filename) != modules.end() ) {
      modules[filename]->ref();
      return modules[filename];
    } else {
      MODULE_CLASS *module = new MODULE_CLASS(module_base_dir + "/" + filename);
      if ( module->open() ) {
	module->ref();
	modules[module->getBaseFilename()] = module;
	return module;
      } else {
	delete module;
	return NULL;
      }
    }
  }

  /** Close a module by Module instance
   * @param module The module that is to be closed
   */
  void closeModule(Module *module)
  {
    closeModule(module->getBaseFilename());
  }

  /** Close a module by filename
   * @param filename the name of the module file that should be closed, this
   * is compared to loaded modules and must match what
   * Module::GetBaseFilename() returns
   */
  void closeModule(std::string filename)
  {
    if ( modules.find(filename) != modules.end() ) {
      modules[filename]->unref();
      if (modules[filename]->notref()) {
	delete modules[filename];
	modules.erase( filename );
      }
    }
  }


  /** Check if the module for the given filename is already
   * opened
   * @param filename the name of the module file to check if it is opened.
   * It is compared to loaded modules and must match what
   * MODULE_CLASS::GetBaseFilename() returns
   */
  bool moduleOpened(std::string filename)
  {
    return ( modules.find(filename) != modules.end() );
  }

  
  /** Get the file extension for the current module type
   * @return Returns a string with the file extension that has to
   * be used for modules on the current system (for example "so")
   */
  std::string getModuleFileExtension()
  {
    return MODULE_CLASS::getFileExtension();
  }

 private:
  std::map<std::string, MODULE_CLASS * >            modules;

  std::string module_base_dir;

};

#endif
