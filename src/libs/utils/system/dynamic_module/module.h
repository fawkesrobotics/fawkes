
/***************************************************************************
 *  module.h - representation of a module (i.e. shared object) using
 *             dl of glibc, applicable for Linux/FreeBSD/MacOS X systems
 *
 *  Created: Wed Aug 23 15:48:23 2006
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

#ifndef __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_H_
#define __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_H_

#include <core/exception.h>
#include <string>

namespace fawkes {


class ModuleOpenException : public Exception
{
 public:
  ModuleOpenException(const char *msg);
};

class Module {
 public:

  /** Flags for the loading process */
  typedef enum {
    MODULE_FLAGS_NONE   = 0,		/**< No flags */
    MODULE_FLAGS_DEFAULT= 0x000E,	/**< Default flags, these are
					 *   MODULE_BIND_GLOBAL, MODULE_BIND_NOW and
					 *   MODULE_BIND_DEEP. */
    MODULE_BIND_LAZY	= 0x0001,	/**< Perform lazy binding. Only resolve
					 *   symbols as thecode that references
					 *   them is executed. If the symbol
					 *   is never referenced,then it is
					 *   never resolved. (Lazy  binding is
					 *   only performed for function
					 *   references; references to variables
					 *   are always immediately bound when
					 *   the library is loaded.)
					 */
    MODULE_BIND_NOW     = 0x0002,	/**< Resolve all symbols immediately when
					 *   loading the library. It's the opposite
					 *   of MODULE_BIND_LAZY. It shall be the
					 *   the default (makes sense for the
					 *   framework robotics).
					 */
    MODULE_BIND_LOCAL	= 0x0000,	/**< Symbols defined in this library are
					 *   not made available to resolve
					 *   references in subsequently
					 *   loaded libraries. It's the opposite
					 *   of MODULE_BIND_GLOBAL. It shall be the
					 *   default and MODULE_BIND_GLOBAL shall
					 *   automatically override it.
					 */
    MODULE_BIND_GLOBAL	= 0x0004,	/**< Symbols defined in this library are
					 *   not made available to resolve
					 *   references in subsequently
					 *   loaded libraries.
					 */
    MODULE_BIND_MASK	= 0x0003,	/**< Can be used to encode flags in a
					 *   longer data field
					 */
    MODULE_BIND_DEEP    = 0x0008,	/**< Place the lookup scope of the symbols
					 *   in this library ahead of the global
					 *   scope. This means that a self-contained
					 *   library will use its own symbols in
					 *   preference to global symbols with the
					 *   same name contained in libraries that
					 *   have already been loaded.
					 */
    MODULE_NODELETE    = 0x1000		/**< Do not unload the library during
					 * dlclose().  Consequently, the
					 * library's static variables are not
					 * reinitialized if the library is
					 * reloaded with dlopen() at a later time.
					 */
  } ModuleFlags;

  Module(std::string filename, ModuleFlags flags = MODULE_FLAGS_DEFAULT);
  virtual ~Module();

  virtual void          open();
  virtual bool          close();
  virtual void          ref();
  virtual void          unref();
  virtual bool          notref();
  virtual unsigned int  get_ref_count();
  virtual bool          has_symbol(const char *symbol_name);
  virtual void *        get_symbol(const char *symbol_name);
  virtual std::string   get_filename();
  virtual std::string   get_base_filename();
  virtual bool          operator==(const Module &cmod);

  static const char * get_file_extension();

 private:
  static const char *FILE_EXTENSION;

  void *       __handle;
  std::string  __filename;
  ModuleFlags  __flags;
  bool         __is_resident;
  unsigned int __ref_count;
};


/** Concatenation of flags.
 * @param flags_a flags to concatenate
 * @param flags_b other flags to concatenate
 * @return concatenated flags
 */
inline Module::ModuleFlags operator|(const Module::ModuleFlags &flags_a,
				     const Module::ModuleFlags &flags_b)
{
  return (Module::ModuleFlags)((int)flags_a | (int)flags_b);
}

} // end namespace fawkes

#endif
