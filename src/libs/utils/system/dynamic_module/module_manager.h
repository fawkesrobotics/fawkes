
/***************************************************************************
 *  module_manager.h - manager for modules (i.e. shared objects)
 *
 *  Created: Wed Aug 23 16:15:02 2006
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

#ifndef __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_MANAGER_H_
#define __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_MANAGER_H_

#include <map>
#include <string>

#include <utils/system/dynamic_module/module.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Module;
class Mutex;

class ModuleManager {
 public:

  ModuleManager(const char *module_base_dir,
		Module::ModuleFlags open_flags = Module::MODULE_FLAGS_DEFAULT);
  virtual ~ModuleManager();

  virtual Module *  open_module(const char *filename);
  virtual void      close_module(Module *module);
  virtual void      close_module(const char *filename);
  virtual bool      module_opened(const char *filename);
  virtual Module *  get_module(const char *filename);

  virtual const char * get_module_file_extension();

  void set_open_flags(Module::ModuleFlags open_flags);

 private:
  std::map<std::string, Module * > __modules;

  const char *__module_base_dir;
  Mutex *__mutex;
  Module::ModuleFlags __open_flags;

};

} // end namespace fawkes

#endif
