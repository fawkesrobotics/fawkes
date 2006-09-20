
/***************************************************************************
 *  module_dl.h - representation of a module (i.e. shared object) using
 *                dl of glibc, applicable for Linux systems
 *
 *  Generated: Wed Aug 23 15:48:23 2006
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

#ifndef __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_DL_H_
#define __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_DL_H_

#include <utils/system/dynamic_module/module.h>
#include <string>

class ModuleDL : public Module {
 public:

  ModuleDL(std::string filename, Module::ModuleFlags flags = MODULE_FLAGS_NONE);

  virtual ~ModuleDL();

  virtual bool    open();
  virtual bool    close();

  virtual void    ref();
  virtual void    unref();
  virtual bool    notref();
  virtual unsigned int getRefCount();


  virtual bool    hasSymbol(const char *symbol_name);
  virtual void *  getSymbol(const char *symbol_name);

  virtual bool    operator==(ModuleDL &cmod);

  virtual std::string getFilename();
  virtual std::string getBaseFilename();

  static const char * getFileExtension();

 private:
  static const char *FILE_EXTENSION;

  void *       handle;
  std::string  filename;
  ModuleFlags  flags;
  bool         file_found;
  bool         is_resident;
  unsigned int ref_count;
};


#endif
