 
/***************************************************************************
 *  instance_factory.cpp - BlackBoard interface instance factory
 *
 *  Created: Mon Mar 03 18:01:53 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/instance_factory.h>
#include <blackboard/exceptions.h>

#include <interface/interface.h>

#include <utils/system/dynamic_module/module_dl.h>
#include <utils/logging/liblogger.h>

#include <cstdlib>
#include <cstring>

namespace fawkes {

/** @class BlackBoardInstanceFactory <blackboard/instance_factory.h>
 * BlackBoard instance factory.
 * This class is used to interact with the interface shared object to create
 * and delete interface instances.
 *
 * @author Tim Niemueller
 */


/** Constructor.*/
BlackBoardInstanceFactory::BlackBoardInstanceFactory()
{
  try {
    iface_module = new ModuleDL( LIBDIR"/libinterfaces.so" );
    iface_module->open();
  } catch (Exception &e) {
    e.append("BlackBoardInstanceFactory cannot open interface module");
    delete iface_module;
    throw;
  }
}


/** Destructor */
BlackBoardInstanceFactory::~BlackBoardInstanceFactory()
{
  delete iface_module;
}


/** Creates a new interface instance.
 * This method will look in the libinterfaces shared object for a factory function
 * for the interface of the given type. If this was found a new instance of the
 * interface is returned.
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @return a new instance of the requested interface type
 * @exception BlackBoardInterfaceNotFoundException thrown if the factory function
 * for the given interface type could not be found
 */
Interface *
BlackBoardInstanceFactory::new_interface_instance(const char *type, const char *identifier)
{
  char *generator_name;
  asprintf(&generator_name, "new%s", type);
  if ( ! iface_module->hasSymbol(generator_name) ) {
    free(generator_name);
    throw BlackBoardInterfaceNotFoundException(type);
  }

  InterfaceFactoryFunc iff = (InterfaceFactoryFunc)iface_module->getSymbol(generator_name);

  Interface *iface = iff();

  iface->set_type_id(type, identifier);

  free(generator_name);
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
  char *destroyer_name;
  asprintf(&destroyer_name, "delete%s", interface->__type);
  if ( ! iface_module->hasSymbol(destroyer_name) ) {
    free(destroyer_name);
    throw BlackBoardInterfaceNotFoundException(interface->__type);
  }

  InterfaceDestroyFunc idf = (InterfaceDestroyFunc)iface_module->getSymbol(destroyer_name);
  idf(interface);
  free(destroyer_name);
}

} // end namespace fawkes
