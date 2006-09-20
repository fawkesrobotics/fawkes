
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

/*
 * For nice reading and hints about using dynamic module loading with C++ you
 * should have a look at
 * http://www.isotton.com/howtos/C++-dlopen-mini-HOWTO/C++-dlopen-mini-HOWTO.html
 */

#include <utils/system/dynamic_module/module_dl.h>
#include <utils/system/file.h>

#include <string>
#include <dlfcn.h>

const char * ModuleDL::FILE_EXTENSION = "so";

ModuleDL::ModuleDL(std::string filename, Module::ModuleFlags flags)
{
  this->filename = filename;
  this->flags    = flags;

  handle         = NULL;

  file_found     = false;
  is_resident    = false;
  ref_count      = 0;
}


ModuleDL::~ModuleDL()
{
  close();
}


bool
ModuleDL::open()
{
  if ( handle != NULL )  return true;

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
    if (File::isRegular(full_filename.c_str())) {
      // We cannot read the file
      file_found = true;

      // ok, try loading the module
      handle = dlopen(full_filename.c_str(),
		      (((flags & MODULE_BIND_LAZY) != 0) ? RTLD_LAZY : RTLD_NOW) |
		      (((flags & MODULE_BIND_LOCAL) != 0) ? 0 : RTLD_GLOBAL) );

      if ( ! handle) {
	file_found = false;
      } else {
	is_resident = false;
	ref_count   = 1;
      }
    }
  }

  return file_found;
}


bool
ModuleDL::close()
{
  if ( handle == NULL )  return true;

  --ref_count;

  if ( (ref_count == 0) && ! is_resident ) {
    if ( dlclose(handle) != 0 ) {
      handle = NULL;
      return false;
    }
    handle = NULL;
  }

  return true;
}


void
ModuleDL::ref()
{
  ++ref_count;
}


void
ModuleDL::unref()
{
  if ( ref_count > 0 ) {
    --ref_count;
  }
}


bool
ModuleDL::notref()
{
  return (ref_count == 0);
}


unsigned int
ModuleDL::getRefCount()
{
  return ref_count;
}


bool
ModuleDL::operator==(ModuleDL &cmod)
{
  return ( filename == cmod.getFilename() );
}


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


void *
ModuleDL::getSymbol(const char *symbol_name)
{
  if( symbol_name == NULL ) return NULL;
  if ( handle == NULL ) return NULL;

  return dlsym( handle, symbol_name );
}


const char *
ModuleDL::getFileExtension()
{
  return FILE_EXTENSION;
}


std::string
ModuleDL::getFilename()
{
  return filename;
}


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
