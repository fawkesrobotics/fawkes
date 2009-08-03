
/***************************************************************************
 *  module_manager.h - manager for modules (i.e. shared objects)
 *
 *  Generated: Wed Aug 23 16:15:02 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_MANAGER_TEMPLATE_H_
#define __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_MANAGER_TEMPLATE_H_

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/system/dynamic_module/module.h>
#include <utils/system/dynamic_module/module_manager.h>
#include <map>
#include <string>

namespace fawkes {

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
  ModuleManagerTemplate(const char *module_base_dir = "")
  {
    modules.clear();
    this->module_base_dir = module_base_dir;
    mutex = new Mutex();
  }

  /** Destructor. */
  ~ModuleManagerTemplate()
  {
    for (typename std::map<std::string, MODULE_CLASS * >::iterator i = modules.begin(); i != modules.end(); ++i) {
      delete (*i).second;
    }
    modules.clear();
    delete mutex;
  }

  MODULE_CLASS *  open_module(const char *filename)
  {
    mutex->lock();
    if ( modules.find(filename) != modules.end() ) {
      modules[filename]->ref();
      mutex->unlock();
      return modules[filename];
    } else {
      MODULE_CLASS *module = new MODULE_CLASS(std::string(module_base_dir) + "/" + filename);
      try {
	module->open();
	// ref count of module is now 1
	modules[module->get_base_filename()] = module;
	mutex->unlock();
	return module;
      } catch (ModuleOpenException &e) {
	delete module;
	mutex->unlock();
	throw;
      }
    }
    mutex->unlock();
  }

  void close_module(Module *module)
  {
    close_module(module->get_base_filename().c_str());
  }

  void close_module(const char *filename)
  {
    mutex->lock();
    if ( modules.find(filename) != modules.end() ) {
      modules[filename]->unref();
      if (modules[filename]->notref()) {
	delete modules[filename];
	modules.erase( filename );
      }
    }
    mutex->unlock();
  }


  Module *  get_module(const char *filename)
  {
    MutexLocker lock(mutex);
    if ( modules.find(filename) != modules.end() ) {
      modules[filename]->ref();
      return modules[filename];
    } else {
      return NULL;
    }
  }


  bool module_opened(const char *filename)
  {
    return ( modules.find(filename) != modules.end() );
  }

  
  const char *get_module_file_extension()
  {
    return MODULE_CLASS::get_file_extension();
  }

 private:
  std::map<std::string, MODULE_CLASS * >            modules;

  const char *module_base_dir;
  Mutex *mutex;

};

} // end namespace fawkes

#endif
