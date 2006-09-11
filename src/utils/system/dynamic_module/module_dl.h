
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

/** A Module implementation for the dl dynamic loader library that comes
 * with glibc, applicable for Linux Systems
 */
class ModuleDL : public Module {
 public:

  /** Constructor for ModuleDL
   * @param filename Full filename of the module
   * @param flags Module flags, @see Module
   */
  ModuleDL(std::string filename, Module::ModuleFlags flags = MODULE_FLAGS_NONE);

  /** Destructor of ModuleDL
   */
  virtual ~ModuleDL();

  /** Open the module
   * @return Returns true if the module could be opened, false otherwise
   */
  virtual bool    open();

  /** Close the module
   * @return Returns true if the module could be closed, false otherwise
   */
  virtual bool    close();


  /** Increment the reference count of this module
   */
  virtual void    ref();

  /** Decrease the reference count of this module
   */
  virtual void    unref();

  /** Check if there are no reference to this module
   * @return Returns true if there are no references to this module,
   * false if there is at least one reference
   */
  virtual bool    notref();

  /** Get the reference count of this module
   * @return Returns the number of references to this module
   */
  virtual unsigned int getRefCount();


  /** Check if the module has the given symbol
   * @param symbol_name The name of the symbol.
   * NOTE: C++ symbols are mangled with type info and thus are not plainly
   * available as symbol name. Use extern "C" to avoid this.
   * Read
   * http://www.isotton.com/howtos/C++-dlopen-mini-HOWTO/C++-dlopen-mini-HOWTO.html
   * for more information on this topic.
   * @return Returns true if the symbol was found, false otherwise
   */
  virtual bool    hasSymbol(const char *symbol_name);

  /** Get a symbol from the module
   * @param symbol_name The name of the symbol.
   * NOTE: C++ symbols are mangled with type info and thus are not plainly
   * available as symbol name. Use extern "C" to avoid this.
   * Read
   * http://www.isotton.com/howtos/C++-dlopen-mini-HOWTO/C++-dlopen-mini-HOWTO.html
   * for more information on this topic.
   * @return Returns a pointer to the symbol or NULL if symbol was not found
   */
  virtual void *  getSymbol(const char *symbol_name);

  /** Compare to another ModuleDL instance
   * @param cmod a reference to the other comparison instance
   * @return Returns true, if the full file names of both modules are the
   * same, false otherwise
   */
  virtual bool    operator==(ModuleDL &cmod);

  /** Get the full file name of the module
   * @return Returns a string with the full file name of the module
   */
  virtual std::string getFilename();

  /** Get the base file name of the module
   * @return Returns the base file name of the module. On Unix systems this is
   * everything after the last slash
   */
  virtual std::string getBaseFilename();

  /** Get file extension for dl modules
   * @return Returns the file extension for dl modules, this is "so"
   */
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
