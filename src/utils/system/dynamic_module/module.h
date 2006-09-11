
/***************************************************************************
 *  module.h - interface for modules (i.e. shared object, dynamic library)
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

#ifndef __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_H_
#define __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_H_

#include <string>

/** Interface representing a dynamically loaded software module
 */
class Module {
 public:

  /** Flags for the loading process
   * MODULE_FLAGS_NONE - no flags
   * MODULE_BIND_LAZY - Perform lazy binding. Only resolve symbols as the 
   *                    code that references them is executed. If the symbol
   *                    is never referenced,then it  is never resolved.
   *                    (Lazy  binding  is only performed for function
   *                     references; references to variables are always
   *                     immediately bound when the library is loaded.)
   * MODULE_BIND_LOCAL - Symbols defined in this library are not made
   *                     available to resolve references in subsequently
   *                     loaded libraries.
   * MODULE_BIND_MASK - Can be used to encode flags in a longer data field
   */
  typedef enum {
    MODULE_FLAGS_NONE   = 0,
    MODULE_BIND_LAZY	= 1 << 0,
    MODULE_BIND_LOCAL	= 1 << 1,
    MODULE_BIND_MASK	= 0x03
  } ModuleFlags;

  /** virtual destructor for pure virtual class
   */
  virtual ~Module() {}

  /** Open the module
   * @return Returns true if the module could be opened, false otherwise
   */
  virtual bool    open() = 0;

  /** Close the module
   * @return Returns true if the module could be closed, false otherwise
   */
  virtual bool    close() = 0;


  /** Increment the reference count of this module
   */
  virtual void    ref() = 0;

  /** Decrease the reference count of this module
   */
  virtual void    unref() = 0;

  /** Check if there are no reference to this module
   * @return Returns true if there are no references to this module,
   * false if there is at least one reference
   */
  virtual bool    notref() = 0;

  /** Get the reference count of this module
   * @return Returns the number of references to this module
   */
  virtual unsigned int getRefCount() = 0;

  /** Check if the module has the given symbol
   * @param symbol_name The name of the symbol.
   * @return Returns true if the symbol was found, false otherwise
   */
  virtual bool    hasSymbol(const char *symbol_name) = 0;

  /** Get a symbol from the module
   * @param symbol_name The name of the symbol.
   * @return Returns a pointer to the symbol or NULL if symbol was not found
   */
  virtual void *  getSymbol(const char *symbol_name) = 0;

  /** Get the full file name of the module
   * @return Returns a string with the full file name of the module
   */
  virtual std::string getFilename() = 0;

  /** Get the base file name of the module
   * @return Returns the base file name of the module. On Unix systems this is
   * everything after the last slash
   */
  virtual std::string getBaseFilename() = 0;

};


#endif
