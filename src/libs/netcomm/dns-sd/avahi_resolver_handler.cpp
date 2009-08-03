
/***************************************************************************
 *  avahi_resolver_handler.cpp - Avahi name resolver
 *
 *  Created: Tue Feb 19 09:40:51 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/dns-sd/avahi_resolver_handler.h>

namespace fawkes {

/** @class AvahiResolverHandler <netcomm/dns-sd/avahi_resolver_handler.h>
 * Avahi resolver handler interface.
 * This interface has to be implemented to make use of the threaded
 * Avahi lookup of names and addresses. After you have ordered a
 * lookup this handler is called with the result.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 *
 * @fn void AvahiResolverHandler::resolved_name(char *name, struct sockaddr *addr, socklen_t addrlen) = 0
 * Name has been successfully resolved.
 * The ordered name lookup was successful for the given name resulting in
 * the given addr of addrlen bytes length.
 * Note that all of the parameters are given to the handler's ownership, that means
 * especially that the handler is responsible for freeing the associated memory
 * after it is done with the result using free() on name and addr.
 * @param name name that was resolved
 * @param addr resulting addr record, currently always of type struct sockaddr_in (only IPv4)
 * @param addrlen length of addr in bytes
 *
 * @fn void AvahiResolverHandler::resolved_address(struct sockaddr_in *addr, socklen_t addrlen, char *name) = 0
 * Address has been successfully resolved.
 * The ordered name lookup was successful for the given address resulting in
 * the given name.
 * Note that all of the parameters are given to the handler's ownership, that means
 * especially that the handler is responsible for freeing the associated memory
 * after it is done with the result using free() on name and addr.
 * @param name the resulting hostname
 * @param addr addr record, currently always of type struct sockaddr_in (only IPv4)
 * @param addrlen length of addr in bytes
 *
 * @fn void AvahiResolverHandler::name_resolution_failed(char *name) = 0
 * Name resolution failed.
 * The given hostname could not be resolved.
 * Note that the parameter name is given to the handler's ownership. This means
 * especially that the handler is responsible for freeing the memory with free()
 * after it is done with the variable.
 * @param name name whose lookup failed
 *
 * @fn void AvahiResolverHandler::address_resolution_failed(struct sockaddr_in *addr, socklen_t addrlen) = 0
 * Address resolution failed.
 * The given address could not be resolved.
 * Note that the parameter addr is given to the handler's ownership. This means
 * especially that the handler is responsible for freeing the memory with free()
 * after it is done with the variable.
 * @param addr address whose lookup failed
 * @param addrlen length of address
 */

/** Virtual empty destructor. */
AvahiResolverHandler::~AvahiResolverHandler()
{
}

} // end namespace fawkes
