
/***************************************************************************
 *  module.h - interface for modules (i.e. shared object, dynamic library)
 *
 *  Generated: Wed Aug 23 15:48:23 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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
    MODULE_BIND_LOCAL	= 0x0002,	/**< Symbols defined in this library are
					 *   not made available to resolve
					 *   references in subsequently
					 *   loaded libraries.
					 */
    MODULE_BIND_GLOBAL	= 0x0004,	/**< Symbols defined in this library are
					 *   not made available to resolve
					 *   references in subsequently
					 *   loaded libraries.
					 */
    MODULE_BIND_MASK	= 0x0003,	/**< Can be used to encode flags in a
					 *   longer data field
					 */
  } ModuleFlags;

  virtual ~Module();

  virtual void          open()                                             = 0;
  virtual bool          close()                                            = 0;
  virtual void          ref()                                              = 0;
  virtual void          unref()                                            = 0;
  virtual bool          notref()                                           = 0;
  virtual unsigned int  getRefCount()                                      = 0;
  virtual bool          hasSymbol(const char *symbol_name)                 = 0;
  virtual void *        getSymbol(const char *symbol_name)                 = 0;
  virtual std::string   getFilename()                                      = 0;
  virtual std::string   getBaseFilename()                                  = 0;

};

} // end namespace fawkes

#endif
