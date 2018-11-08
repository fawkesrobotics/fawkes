
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
  filename_ = filename;
  flags_    = flags;

  handle_         = NULL;

  is_resident_    = false;
  ref_count_      = 0;
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
  if ( handle_ != NULL )  return;

  // Note: We assume Linux-style shared objects
  std::string full_filename = "";
  full_filename = filename_;
  //                                                                .   SOEXT
  if ( full_filename.find("." SOEXT, 0) != (full_filename.length() - 1 - strlen(FILE_EXTENSION)) ) {
    // filename has no proper ending
    full_filename += "." SOEXT;
  }

  int tflags = 0;
  tflags |= ((flags_ & MODULE_BIND_LAZY)   != 0) ? RTLD_LAZY : RTLD_NOW;
  tflags |= ((flags_ & MODULE_BIND_NOW)    != 0) ? RTLD_NOW : 0;
  tflags |= ((flags_ & MODULE_BIND_LOCAL)  != 0) ? RTLD_LOCAL : 0;
  tflags |= ((flags_ & MODULE_BIND_GLOBAL) != 0) ? RTLD_GLOBAL : 0;
  tflags |= ((flags_ & MODULE_NODELETE)    != 0) ? RTLD_NODELETE : 0;
#ifdef linux
  tflags |= ((flags_ & MODULE_BIND_DEEP)   != 0) ? RTLD_DEEPBIND : 0;
#endif

  if ( full_filename == "") {
    handle_ = dlopen (NULL, tflags);

    filename_    = "main";
    is_resident_ = true;
    ref_count_   = 1;
  } else {

    // check whether we have a readable file right away
    if (File::is_regular(full_filename.c_str())) {
      // ok, try loading the module
      handle_ = dlopen(full_filename.c_str(), tflags);

      if ( NULL == handle_) {
	const char *err = dlerror();
	if ( NULL == err ) {
	  throw ModuleOpenException("dlopen failed with an unknown error");
	} else {
	  ModuleOpenException e("dlopen failed");
	  e.append("dlerror: %s", err);
	  throw e;
	}
      } else {
	is_resident_ = false;
	ref_count_   = 1;
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
  if ( handle_ == NULL )  return true;

  if ( ref_count_ > 0 )  --ref_count_;

  if ( (ref_count_ == 0) && ! is_resident_ ) {
    if ( dlclose(handle_) != 0 ) {
      handle_ = NULL;
      return false;
    }
    handle_ = NULL;
  }

  return true;
}


/** Increment the reference count of this module */
void
Module::ref()
{
  ++ref_count_;
}


/** Decrease the reference count of this module */
void
Module::unref()
{
  if ( ref_count_ > 0 ) {
    --ref_count_;
  }
}


/** Check if there are no reference to this module
 * @return Returns true if there are no references to this module,
 * false if there is at least one reference
 */
bool
Module::notref()
{
  return (ref_count_ == 0);
}


/** Get the reference count of this module
 * @return Returns the number of references to this module
 */
unsigned int
Module::get_ref_count()
{
  return ref_count_;
}


/** Compare to another Module instance
 * @param cmod a reference to the other comparison instance
 * @return Returns true, if the full file names of both modules are the
 * same, false otherwise
 */
bool
Module::operator==(const Module &cmod)
{
  return (filename_ == cmod.filename_);
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
  if ( handle_ == NULL ) {
    return false;
  }

  return ( dlsym( handle_, symbol_name ) != NULL );
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
  if ( handle_ == NULL ) return NULL;

  return dlsym( handle_, symbol_name );
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
  return filename_;
}


/** Get the base file name of the module
 * @return Returns the base file name of the module. On Unix systems this is
 * everything after the last slash
 */
std::string
Module::get_base_filename()
{
  if ( filename_.find("/", 0) != std::string::npos ) {
    std::string rv = filename_.substr(filename_.rfind("/", filename_.length()) + 1, filename_.length());
    return rv;
  } else {
    return filename_.c_str();
  }
}

} // end namespace fawkes
