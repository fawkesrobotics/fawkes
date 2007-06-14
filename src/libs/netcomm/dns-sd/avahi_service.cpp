
/***************************************************************************
 *  avahi_service.cpp - Avahi service representation
 *
 *  Generated: Tue Nov 07 18:02:23 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <netcomm/dns-sd/avahi_service.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdarg.h>
#include <avahi-common/cdecl.h>
#include <avahi-common/gccmacro.h>
#include <avahi-common/strlst.h>
#include <avahi-common/malloc.h>
#include <cstddef>

#include <string.h>

/** @class AvahiService netcomm/dns-sd/avahi_service.h
 * Representation of a service announced or found via Avahi.
 * This class is used in the C++ wrapper to talk about services.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * This constructor sets all parameters.
 * @param name name of service
 * @param type type of service
 * @param domain domain of service
 * @param host host of service
 * @param port port of service
 */
AvahiService::AvahiService(const char         *name,
			   const char         *type,
			   const char         *domain,
			   const char         *host,
			   unsigned short int  port)
{
  _name   = avahi_strdup(name);
  _type   = avahi_strdup(type);
  _domain = avahi_strdup(domain);
  _host   = avahi_strdup(host);
  _port   = port;
  list    = NULL;
}


/** Constructor.
 * This constructor sets all parameters. Host and domain are the
 * default values, which means local host name in domain .local
 * (if not set otherwise in Avahi system configuration).
 * @param name name of service
 * @param type type of service
 * @param port port of service
 */
AvahiService::AvahiService(const char         *name,
			   const char         *type,
			   unsigned short int  port)
{
  _name   = avahi_strdup(name);
  _type   = avahi_strdup(type);
  _domain = NULL;
  _host   = NULL;
  _port   = port;
  list    = NULL;
}


/** Destructor. */
AvahiService::~AvahiService()
{
  avahi_string_list_free( list );
  list = NULL;
  avahi_free( _name );
  avahi_free( _type );
  avahi_free( _domain );
  avahi_free( _host );
}


/** Add a TXT record.
 * @param txt TXT record to add, must be a "key=value" string.
 */
void
AvahiService::add_txt(const char *txt)
{
  list = avahi_string_list_add( list, txt );
}


/** Set name of service.
 * @param new_name new name
 */
void
AvahiService::set_name(const char *new_name)
{
  avahi_free( _name );
  _name = avahi_strdup(new_name);
}


/** Get name of service.
 * @return name of service
 */
const char *
AvahiService::name()
{
  return _name;
}


/** Get type of service.
 * @return type of service
 */
const char *
AvahiService::type()
{
  return _type;
}


/** Get domain of service.
 * @return domain of service
 */
const char *
AvahiService::domain()
{
  return _domain;
}


/** Get host of service.
 * @return host of service
 */
const char *
AvahiService::host()
{
  return _host;
}


/** Get port of service.
 * @return port of service
 */
unsigned short int
AvahiService::port()
{
  return _port;
}


/** Get TXT record list of service.
 * @return TXT record list of service
 */
AvahiStringList *
AvahiService::txt()
{
  return list;
}


/** Equal operator for AvahiService reference.
 * @param s reference of service to compare to.
 * @return true, if the services are the same (same name and type), false otherwise
 */
bool
AvahiService::operator==(const AvahiService &s) const
{
  return ( (strcmp(_name, s._name) == 0) &&
	   (strcmp(_type, s._type) == 0) );
}


/** Equal operator for AvahiService pointer.
 * @param s pointer to service to compare to.
 * @return true, if the services are the same (same name and type), false otherwise
 */
bool
AvahiService::operator==(const AvahiService *s) const
{
  return ( (strcmp(_name, s->_name) == 0) &&
	   (strcmp(_type, s->_type) == 0) );
}
