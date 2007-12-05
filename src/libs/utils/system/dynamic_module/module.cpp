
/***************************************************************************
 *  module.cpp - interface for modules (i.e. shared object, dynamic library)
 *
 *  Created: Wed May 09 11:03:40 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/dynamic_module/module.h>

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
 * Interface representing a dynamically loaded software module
 * @author Tim Niemueller
 *
 * @fn void Module::open() = 0
 * Open the module
 * @exception ModuleOpenException thrown if there was any problem loading the module
 *
 * @fn bool Module::close() = 0
 * Close the module
 * @return Returns true if the module could be closed, false otherwise
 *
 *
 * @fn void Module::ref() = 0
 * Increment the reference count of this module
 *
 * @fn void Module::unref() = 0
 * Decrease the reference count of this module
 *
 * @fn bool Module::notref() = 0
 * Check if there are no reference to this module
 * @return Returns true if there are no references to this module,
 * false if there is at least one reference
 *
 * @fn unsigned int Module::getRefCount() = 0
 * Get the reference count of this module
 * @return Returns the number of references to this module
 *
 * @fn bool Module::hasSymbol(const char *symbol_name) = 0
 * Check if the module has the given symbol
 * @param symbol_name The name of the symbol.
 * @return Returns true if the symbol was found, false otherwise
 *
 * @fn void * Module::getSymbol(const char *symbol_name) = 0
 * Get a symbol from the module
 * @param symbol_name The name of the symbol.
 * @return Returns a pointer to the symbol or NULL if symbol was not found
 *
 * @fn std::string Module::getFilename() = 0
 * Get the full file name of the module
 * @return Returns a string with the full file name of the module
 *
 * @fn std::string Module::getBaseFilename() = 0
 * Get the base file name of the module
 * @return Returns the base file name of the module. On Unix systems this is
 * everything after the last slash
 */

/** Virtual empty destructor */
Module::~Module()
{
}
