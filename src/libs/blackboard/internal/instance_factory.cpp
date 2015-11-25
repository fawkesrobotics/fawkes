 
/***************************************************************************
 *  instance_factory.cpp - BlackBoard interface instance factory
 *
 *  Created: Mon Mar 03 18:01:53 2008
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

#include <blackboard/internal/instance_factory.h>
#include <blackboard/exceptions.h>

#include <interface/interface.h>

#include <utils/system/dynamic_module/module_manager.h>
#include <utils/system/dynamic_module/module.h>

#include <cstdlib>
#include <cstring>

namespace fawkes {

/** @class BlackBoardInstanceFactory <blackboard/internal/instance_factory.h>
 * BlackBoard instance factory.
 * This class is used to interact with the interface shared object to create
 * and delete interface instances.
 *
 * @author Tim Niemueller
 */


/** Constructor.*/
BlackBoardInstanceFactory::BlackBoardInstanceFactory()
{
  __mm = new ModuleManager(IFACEDIR);
}


/** Destructor */
BlackBoardInstanceFactory::~BlackBoardInstanceFactory()
{
  delete __mm;
}


/** Creates a new interface instance.
 * This method will look in the for the appropriate library in LIBDIR/interfaces
 * and then use the factory function for the interface of the given type. If
 * this was found a new instance of the interface is returned.
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @return a new instance of the requested interface type
 * @exception BlackBoardInterfaceNotFoundException thrown if the factory function
 * for the given interface type could not be found
 */
Interface *
BlackBoardInstanceFactory::new_interface_instance(const char *type, const char *identifier)
{
	if (strlen(identifier) == 0) {
		throw Exception("Interface ID may not be empty");	
	}
	if (strlen(type) == 0) {
		throw Exception("Interface type may not be empty");	
	}
	if (strlen(type) > __INTERFACE_TYPE_SIZE) {
		throw Exception("Interface type '%s' too long, maximum length is %zu",
		                type, __INTERFACE_TYPE_SIZE);
	}
	if (strlen(identifier) > __INTERFACE_ID_SIZE) {
		throw Exception("Interface ID '%s' too long, maximum length is %zu",
		                type, __INTERFACE_ID_SIZE);
	}

  Module *mod = NULL;
  std::string filename = std::string("lib") + type + "." + __mm->get_module_file_extension();
  try {
      mod = __mm->open_module(filename.c_str());
  } catch (Exception &e) {
    throw BlackBoardInterfaceNotFoundException(type, " Module file not found.");
  }

  if ( ! mod->has_symbol("interface_factory") ) {
    throw BlackBoardInterfaceNotFoundException(type, " Generator function not found.");
  }

  InterfaceFactoryFunc iff = (InterfaceFactoryFunc)mod->get_symbol("interface_factory");

  Interface *iface = iff();
  iface->set_type_id(type, identifier);

  return iface;
}


/** Destroy an interface instance.
 * The destroyer function for the given interface is called to destroy the given
 * interface instance.
 * @param interface to destroy
 * @exception BlackBoardInterfaceNotFoundException thrown if the destroyer function
 * for the given interface could not be found. The interface will not be freed.
 */
void
BlackBoardInstanceFactory::delete_interface_instance(Interface *interface)
{
  std::string filename = std::string("lib") + interface->__type + "." + __mm->get_module_file_extension();
  Module *mod = __mm->get_module(filename.c_str());

  if ( ! mod) {
    throw BlackBoardInterfaceNotFoundException(interface->__type, " Interface module not opened.");
  }

  if ( ! mod->has_symbol("interface_destroy") ) {
    throw BlackBoardInterfaceNotFoundException(interface->__type, " Destroyer function not found.");
  }

  InterfaceDestroyFunc idf = (InterfaceDestroyFunc)mod->get_symbol("interface_destroy");
  idf(interface);

  mod->unref();
  __mm->close_module(mod);
}

} // end namespace fawkes
