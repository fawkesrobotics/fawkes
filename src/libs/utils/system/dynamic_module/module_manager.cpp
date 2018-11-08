
/***************************************************************************
 *  module_manager.cpp - manager for modules (i.e. shared objects)
 *
 *  Generated: Thu Jun 02 11:45:03 2011 (based on module_manager_template.h)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/dynamic_module/module_manager.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

namespace fawkes {


/** @class ModuleManager <utils/system/dynamic_module/module_manager.h>
 * Dynamic module manager.
 * Manager to load and unload modules, keeps track of loaded modules
 * and does not reload modules if they are already loaded.
 * @author Tim Niemueller
 */

/** Constructor of NetworkManagerTemplate
 * @param module_base_dir The module basedir where to look for plugins
 * @param open_flags flags to pass to modules when opening them
 */
ModuleManager::ModuleManager(const char *module_base_dir,
			     Module::ModuleFlags open_flags)
{
  modules_.clear();
  module_base_dir_ = module_base_dir;
  mutex_ = new Mutex();
  open_flags_ = open_flags;
}

/** Destructor. */
ModuleManager::~ModuleManager()
{
  std::map<std::string, Module * >::iterator i;
  for (i = modules_.begin(); i != modules_.end(); ++i) {
    delete (*i).second;
  }
  modules_.clear();
  delete mutex_;
}


/** Set flags to open modules with.
 * @param open_flags flags to pass to modules when opening them
 */
void
ModuleManager::set_open_flags(Module::ModuleFlags open_flags)
{
  open_flags_ = open_flags;
}


/** Open a module
 * @param filename The file name of the module that should be
 * opened. If the ModuleManager implementation takes a base dir
 * argument (recommended) this filename is relative to that
 * base dir
 * @return Returns the module if the file was opened successfully
 * or NULL otherwise. Do NOT delete the module after usage but use
 * closeModule to close it.
 * @exception ModuleOpenException thrown if the module could not be opened
 */
Module *
ModuleManager::open_module(const char *filename)
{
  mutex_->lock();
  if ( modules_.find(filename) != modules_.end() ) {
    modules_[filename]->ref();
    mutex_->unlock();
    return modules_[filename];
  } else {
    Module *module = new Module(std::string(module_base_dir_) + "/" + filename,
				open_flags_);
    try {
      module->open();
      // ref count of module is now 1
      modules_[module->get_base_filename()] = module;
      mutex_->unlock();
      return module;
    } catch (ModuleOpenException &e) {
      delete module;
      mutex_->unlock();
      throw;
    }
  }
  mutex_->unlock();
}


/** Close a module by Module instance
 * @param module The module that is to be closed
 */
void
ModuleManager::close_module(Module *module)
{
  close_module(module->get_base_filename().c_str());
}


/** Close a module by filename
 * @param filename the name of the module file that should be closed, this
 * is compared to loaded modules and must match what
 * Module::GetBaseFilename() returns
 */
void
ModuleManager::close_module(const char *filename)
{
  mutex_->lock();
  if ( modules_.find(filename) != modules_.end() ) {
    modules_[filename]->unref();
    if (modules_[filename]->notref()) {
      delete modules_[filename];
      modules_.erase( filename );
    }
  }
  mutex_->unlock();
}


/** Get a module if opened.
 * This will return a pointer to a module if it had already been opened! The
 * reference count is increased and you have to manually unref the module once
 * you are done with it! This method works similar to open_module() with the
 * difference that it is not tried to load the module if it is not open.
 * @param filename file name of the module
 * @return a pointer to the module with the reference cound incremented by one
 * if the module had been opened already or NULL if it was not opened.
 */
Module *
ModuleManager::get_module(const char *filename)
{
  MutexLocker lock(mutex_);
  if ( modules_.find(filename) != modules_.end() ) {
    modules_[filename]->ref();
    return modules_[filename];
  } else {
    return NULL;
  }
}


/** Check if the module is already opened.
 * @param filename the name of the module file to check if it is opened.
 * It is compared to loaded modules and must match what
 * Module::get_base_filename() returns
 * @return true if module has been opened, false otherwise
 */
bool
ModuleManager::module_opened(const char *filename)
{
  return ( modules_.find(filename) != modules_.end() );
}

  
/** Get the file extension for the current module type.
 * @return Returns a string with the file extension that has to
 * be used for modules on the current system (for example "so")
 */
const char *
ModuleManager::get_module_file_extension()
{
  return Module::get_file_extension();
}

} // end of namespace fawkes
