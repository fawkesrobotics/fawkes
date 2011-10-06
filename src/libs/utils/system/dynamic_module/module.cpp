
/***************************************************************************
 *  module.cpp - interface for modules (i.e. shared object, dynamic library)
 *
 *  Created: Wed May 09 11:03:40 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/dynamic_module/module.h>
#include <utils/system/file.h>

#include <cstring>
#include <dlfcn.h>

namespace fawkes {

/** @class ModuleOpenException <utils/system/dynamic_module/module.h>
 * Opening a module failed.
 * Thrown if a call to Module::open() failed.
 */

/** Constructor.
 * @param msg message
 */
ModuleOpenException::ModuleOpenException(const char *msg)
  : Exception(msg)
{
}

/** @class Module <utils/system/dynamic_module/module.h>
 * Dynamic module loader for Linux, FreeBSD, and MacOS X.
 * A Module implementation for the dl dynamic loader library that comes
 * with glibc, applicable for Linux, FreeBSD, and MacOS X Systems.
 *
 * For nice reading and hints about using dynamic module loading with C++ you
 * should have a look at
 * http://www.isotton.com/howtos/C++-dlopen-mini-HOWTO/C++-dlopen-mini-HOWTO.html
 * @author Tim Niemueller
 */

// SOEXT is a macro passed in from the build system and set in config.mk or
// a build type specific config file.
const char * Module::FILE_EXTENSION = SOEXT;

/** Constructor.
 * @param filename full filename of the module
 * @param flags module flags
 */
Module::Module(std::string filename, Module::ModuleFlags flags)
{
  __filename = filename;
  __flags    = flags;

  __handle         = NULL;

  __is_resident    = false;
  __ref_count      = 0;
}


/** Destructor.
 * Closes the module. */
Module::~Module()
{
  close();
}

/** Open the module
 * @return true if the module could be opened, false otherwise
 * @exception ModuleOpenException thrown if there was any problem
 * while loading the module
 */
void
Module::open()
{
  if ( __handle != NULL )  return;

  // Note: We assume Linux-style shared objects
  std::string full_filename = "";
  full_filename = __filename;
  //                                                                .   SOEXT
  if ( full_filename.find("."SOEXT, 0) != (full_filename.length() - 1 - strlen(FILE_EXTENSION)) ) {
    // filename has no proper ending
    full_filename += "."SOEXT;
  }

  int tflags = 0;
  tflags |= ((__flags & MODULE_BIND_LAZY)   != 0) ? RTLD_LAZY : RTLD_NOW;
  tflags |= ((__flags & MODULE_BIND_NOW)    != 0) ? RTLD_NOW : 0;
  tflags |= ((__flags & MODULE_BIND_LOCAL)  != 0) ? RTLD_LOCAL : 0;
  tflags |= ((__flags & MODULE_BIND_GLOBAL) != 0) ? RTLD_GLOBAL : 0;
  tflags |= ((__flags & MODULE_NODELETE)    != 0) ? RTLD_NODELETE : 0;
#ifdef linux
  tflags |= ((__flags & MODULE_BIND_DEEP)   != 0) ? RTLD_DEEPBIND : 0;
#endif

  if ( full_filename == "") {
    __handle = dlopen (NULL, tflags);

    __filename    = "main";
    __is_resident = true;
    __ref_count   = 1;
  } else {

    // check whether we have a readable file right away
    if (File::is_regular(full_filename.c_str())) {
      // ok, try loading the module
      __handle = dlopen(full_filename.c_str(), tflags);

      if ( NULL == __handle) {
	const char *err = dlerror();
	if ( NULL == err ) {
	  throw ModuleOpenException("dlopen failed with an unknown error");
	} else {
	  ModuleOpenException e("dlopen failed");
	  e.append("dlerror: %s", err);
	  throw e;
	}
      } else {
	__is_resident = false;
	__ref_count   = 1;
      }
    } else {
      ModuleOpenException e("Cannot open module");
      e.append("File '%s' does not exist", full_filename.c_str());
      throw e;
    }
  }
}


/** Close the module
 * @return Returns true if the module could be closed, false otherwise
 */
bool
Module::close()
{
  if ( __handle == NULL )  return true;

  if ( __ref_count > 0 )  --__ref_count;

  if ( (__ref_count == 0) && ! __is_resident ) {
    if ( dlclose(__handle) != 0 ) {
      __handle = NULL;
      return false;
    }
    __handle = NULL;
  }

  return true;
}


/** Increment the reference count of this module */
void
Module::ref()
{
  ++__ref_count;
}


/** Decrease the reference count of this module */
void
Module::unref()
{
  if ( __ref_count > 0 ) {
    --__ref_count;
  }
}


/** Check if there are no reference to this module
 * @return Returns true if there are no references to this module,
 * false if there is at least one reference
 */
bool
Module::notref()
{
  return (__ref_count == 0);
}


/** Get the reference count of this module
 * @return Returns the number of references to this module
 */
unsigned int
Module::get_ref_count()
{
  return __ref_count;
}


/** Compare to another Module instance
 * @param cmod a reference to the other comparison instance
 * @return Returns true, if the full file names of both modules are the
 * same, false otherwise
 */
bool
Module::operator==(const Module &cmod)
{
  return (__filename == cmod.__filename);
}


/** Check if the module has the given symbol
 * @param symbol_name The name of the symbol.
 * NOTE: C++ symbols are mangled with type info and thus are not plainly
 * available as symbol name. Use extern "C" to avoid this.
 * Read
 * http://www.isotton.com/howtos/C++-dlopen-mini-HOWTO/C++-dlopen-mini-HOWTO.html
 * for more information on this topic.
 * @return Returns true if the symbol was found, false otherwise
 */
bool
Module::has_symbol(const char *symbol_name)
{
  if( symbol_name == NULL ) {
    return false;
  }
  if ( __handle == NULL ) {
    return false;
  }

  return ( dlsym( __handle, symbol_name ) != NULL );
}


/** Get a symbol from the module
 * @param symbol_name The name of the symbol.
 * NOTE: C++ symbols are mangled with type info and thus are not plainly
 * available as symbol name. Use extern "C" to avoid this.
 * Read
 * http://www.isotton.com/howtos/C++-dlopen-mini-HOWTO/C++-dlopen-mini-HOWTO.html
 * for more information on this topic.
 * @return Returns a pointer to the symbol or NULL if symbol was not found
 */
void *
Module::get_symbol(const char *symbol_name)
{
  if( symbol_name == NULL ) return NULL;
  if ( __handle == NULL ) return NULL;

  return dlsym( __handle, symbol_name );
}


/** Get file extension for dl modules
 * @return Returns the file extension for dl modules, this is "so" on Linux
 * and FreeBSD systems, and dylib on MacOS X. It is defined at compile time
 * in config.mk.
 */
const char *
Module::get_file_extension()
{
  return FILE_EXTENSION;
}


/** Get the full file name of the module
 * @return Returns a string with the full file name of the module
 */
std::string
Module::get_filename()
{
  return __filename;
}


/** Get the base file name of the module
 * @return Returns the base file name of the module. On Unix systems this is
 * everything after the last slash
 */
std::string
Module::get_base_filename()
{
  if ( __filename.find("/", 0) != std::string::npos ) {
    std::string rv = __filename.substr(__filename.rfind("/", __filename.length()) + 1, __filename.length());
    return rv;
  } else {
    return __filename.c_str();
  }
}

} // end namespace fawkes
