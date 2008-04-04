
/***************************************************************************
 *  module-dl.cpp - representation of a module (i.e. shared object)
 *                  This code is based on gmodule from glib
 *
 *  Generated: Wed Aug 23 15:58:27 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

/*
 * For nice reading and hints about using dynamic module loading with C++ you
 * should have a look at
 * http://www.isotton.com/howtos/C++-dlopen-mini-HOWTO/C++-dlopen-mini-HOWTO.html
 */

#include <utils/system/dynamic_module/module_dl.h>
#include <utils/system/file.h>

#include <cstring>
#include <dlfcn.h>

/** @class ModuleDL utils/system/dynamic_module/module_dl.h
 * A Module implementation for the dl dynamic loader library that comes
 * with glibc, applicable for Linux Systems
 */


const char * ModuleDL::FILE_EXTENSION = "so";


/** Constructor for ModuleDL
 * @param filename Full filename of the module
 * @param flags Module flags, @see Module
 */
ModuleDL::ModuleDL(std::string filename, Module::ModuleFlags flags)
{
  this->filename = filename;
  this->flags    = flags;

  handle         = NULL;

  file_found     = false;
  is_resident    = false;
  ref_count      = 0;
}


/** Destructor of ModuleDL */
ModuleDL::~ModuleDL()
{
  close();
}


/** Open the module
 * @return Returns true if the module could be opened, false otherwise
 * @exception ModuleOpenException Thrown if there was any problem while loading the
 * module
 */
void
ModuleDL::open()
{
  if ( handle != NULL )  return;

  // Note: We assume Linux-style shared objects
  std::string full_filename = "";
  full_filename = filename;
  if ( full_filename.find(".so", 0) != (full_filename.length() - 3)) {
    // filename has no proper ending
    full_filename += ".so";
  }

  if ( full_filename == "") {      
    handle = dlopen (NULL, RTLD_GLOBAL | RTLD_LAZY);

    filename    = "main";
    is_resident = true;
    ref_count   = 1;
  } else {

    // check whether we have a readable file right away */
    if (File::is_regular(full_filename.c_str())) {

      // ok, try loading the module
      int tflags = ((flags & MODULE_BIND_LAZY) != 0) ? RTLD_LAZY : RTLD_NOW;
      if ( (flags & MODULE_BIND_LOCAL) != 0 ) {
	tflags |= RTLD_LOCAL;
      } else if ( (flags & MODULE_BIND_GLOBAL) != 0 ) {
	tflags |= RTLD_GLOBAL;
      }
      //tflags = RTLD_LAZY;
      //printf("Loading module %s, flags: %i\n", full_filename.c_str(), tflags);
      handle = dlopen(full_filename.c_str(), tflags);

      if ( NULL == handle) {
	const char *err = dlerror();
	if ( NULL == err ) {
	  throw ModuleOpenException("dlopen failed with an unknown error");
	} else {
	  ModuleOpenException e("dlopen failed");
	  e.append("dlerror: %s", err);
	  throw e;
	}
      } else {
	is_resident = false;
	ref_count   = 1;
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
ModuleDL::close()
{
  if ( handle == NULL )  return true;

  if ( ref_count > 0 )  --ref_count;

  if ( (ref_count == 0) && ! is_resident ) {
    if ( dlclose(handle) != 0 ) {
      handle = NULL;
      return false;
    }
    handle = NULL;
  }

  return true;
}


/** Increment the reference count of this module */
void
ModuleDL::ref()
{
  ++ref_count;
}


/** Decrease the reference count of this module */
void
ModuleDL::unref()
{
  if ( ref_count > 0 ) {
    --ref_count;
  }
}


/** Check if there are no reference to this module
 * @return Returns true if there are no references to this module,
 * false if there is at least one reference
 */
bool
ModuleDL::notref()
{
  return (ref_count == 0);
}


/** Get the reference count of this module
 * @return Returns the number of references to this module
 */
unsigned int
ModuleDL::getRefCount()
{
  return ref_count;
}


/** Compare to another ModuleDL instance
 * @param cmod a reference to the other comparison instance
 * @return Returns true, if the full file names of both modules are the
 * same, false otherwise
 */
bool
ModuleDL::operator==(ModuleDL &cmod)
{
  return ( filename == cmod.getFilename() );
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
ModuleDL::hasSymbol(const char *symbol_name)
{
  if( symbol_name == NULL ) {
    return false;
  }
  if ( handle == NULL ) {
    return false;
  }

  return ( dlsym( handle, symbol_name ) != NULL );
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
ModuleDL::getSymbol(const char *symbol_name)
{
  if( symbol_name == NULL ) return NULL;
  if ( handle == NULL ) return NULL;

  return dlsym( handle, symbol_name );
}


/** Get file extension for dl modules
 * @return Returns the file extension for dl modules, this is "so"
 */
const char *
ModuleDL::getFileExtension()
{
  return FILE_EXTENSION;
}


/** Get the full file name of the module
 * @return Returns a string with the full file name of the module
 */
std::string
ModuleDL::getFilename()
{
  return filename;
}


/** Get the base file name of the module
 * @return Returns the base file name of the module. On Unix systems this is
 * everything after the last slash
 */
std::string
ModuleDL::getBaseFilename()
{
  if ( filename.find("/", 0) != std::string::npos ) {
    std::string rv = filename.substr(filename.rfind("/", filename.length()) + 1, filename.length());
    return rv;
  } else {
    return filename.c_str();
  }
}
