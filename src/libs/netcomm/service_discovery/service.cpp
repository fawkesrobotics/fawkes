
/***************************************************************************
 *  service.cpp - Network service representation
 *
 *  Generated: Tue Nov 07 18:02:23 2006
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

#include <netcomm/service_discovery/service.h>
#include <netcomm/utils/resolver.h>
#include <core/exceptions/system.h>

#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <inttypes.h>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NetworkService <netcomm/service_discovery/service.h>
 * Representation of a service announced or found via service
 * discovery (i.e. mDNS/DNS-SD via Avahi).
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
NetworkService::NetworkService(const char         *name,
			       const char         *type,
			       const char         *domain,
			       const char         *host,
			       unsigned short int  port)
{
  _name   = strdup(name);
  _type   = strdup(type);
  _domain = strdup(domain);
  _host   = strdup(host);
  _port   = port;

  _modified_name = NULL;

  _addr = NULL;
  _addr_size = 0;
}


/** Constructor.
 * This constructor sets all parameters.
 * @param name name of service
 * @param type type of service
 * @param domain domain of service
 * @param host host of service
 * @param port port of service
 * @param addr address of the service
 * @param addr_size size in bytes of addr parameter
 * @param txt list of TXT records
 */
NetworkService::NetworkService(const char         *name,
			       const char         *type,
			       const char         *domain,
			       const char         *host,
			       unsigned short int  port,
			       const struct sockaddr *addr,
			       const socklen_t     addr_size,
			       std::list<std::string> &txt)

{
  _name   = strdup(name);
  _type   = strdup(type);
  _domain = strdup(domain);
  _host   = strdup(host);
  _port   = port;

  _modified_name = NULL;

  _addr = (struct sockaddr *)malloc(addr_size);
  memcpy(_addr, addr, addr_size);
  _addr_size = addr_size;
  list = txt;
}


/** Constructor.
 * This constructor sets all parameters. Host and domain are the
 * default values, which means local host name in domain .local
 * (if not set otherwise in Avahi system configuration).
 * @param name name of service
 * @param type type of service
 * @param port port of service
 */
NetworkService::NetworkService(const char         *name,
			       const char         *type,
			       unsigned short int  port)
{
  _name   = strdup(name);
  _type   = strdup(type);
  _domain = NULL;
  _host   = NULL;
  _port   = port;

  _modified_name = NULL;

  _addr = NULL;
  _addr_size = 0;
}


/** Constructor.
 * This constructor sets all parameters. Host and domain are the
 * default values, which means local host name in domain .local
 * (if not set otherwise in Avahi system configuration).
 * This specific constructor allows the usage of a "%h" token in
 * the name, which is replaced with the short hostname.
 * @param nnresolver network name resolver to get the host from for
 * the replacement of a %h token.
 * @param name name of service
 * @param type type of service
 * @param port port of service
 */
NetworkService::NetworkService(NetworkNameResolver *nnresolver,
			       const char         *name,
			       const char         *type,
			       unsigned short int  port)
{
  std::string s = name;
  std::string::size_type hpos = s.find("%h");
  if (nnresolver && (hpos != std::string::npos)) {
    s.replace(hpos, 2, nnresolver->short_hostname());
  }
  _name   = strdup(s.c_str());
  _type   = strdup(type);
  _domain = NULL;
  _host   = NULL;
  _port   = port;

  _modified_name = NULL;

  _addr = NULL;
  _addr_size = 0;
}

/** Constructor.
 * This constructor sets all parameters.
 * @param name name of service
 * @param type type of service
 * @param domain domain of service
 */
NetworkService::NetworkService(const char         *name,
			       const char         *type,
			       const char         *domain)
{
  _name   = strdup(name);
  _type   = strdup(type);
  _domain = strdup(domain);

  _modified_name = NULL;

  _host   = NULL;
  _port   = 0;
  _addr = NULL;
  _addr_size = 0;
}


/** Destructor. */
NetworkService::~NetworkService()
{
  if ( _name   != NULL)  free( _name );
  if ( _type   != NULL)  free( _type );
  if ( _domain != NULL)  free( _domain );
  if ( _host   != NULL)  free( _host );
  if ( _addr   != NULL)  free( _addr );
  if ( _modified_name   != NULL)  free( _modified_name );
}


/** Copy constructor (pointer).
 * Create a copy of given NetworkService.
 * @param s network service to copy from
 */
NetworkService::NetworkService(const NetworkService *s)
{
  _name = strdup(s->_name);
  _type = strdup(s->_type);
  _port = s->_port;
  if ( s->_domain != NULL ) {
    _domain = strdup(s->_domain);
  } else {
    _domain = NULL;
  }
  if ( s->_host != NULL ) {
    _host = strdup(s->_host);
  } else {
    _host = NULL;
  }

  _modified_name = NULL;
  if (s->_modified_name != NULL) {
    _modified_name = strdup(s->_modified_name);
  }

  _addr = NULL;
  _addr_size = 0;

  list = s->list;
}


/** Copy constructor (reference).
 * Create a copy of given NetworkService.
 * @param s network service to copy from
 */
NetworkService::NetworkService(const NetworkService &s)
{
  _name = strdup(s._name);
  _type = strdup(s._type);
  _port = s._port;
  if ( s._domain != NULL ) {
    _domain = strdup(s._domain);
  } else {
    _domain = NULL;
  }
  if ( s._host != NULL ) {
    _host = strdup(s._host);
  } else {
    _host = NULL;
  }

  _modified_name = NULL;
  if (s._modified_name != NULL) {
    _modified_name = strdup(s._modified_name);
  }

  _addr = NULL;
  _addr_size = 0;

  list = s.list;
}


/** Add a TXT record.
 * @param format format for TXT record to add, must be a "key=value" string,
 * takes the same arguments as sprintf.
 */
void
NetworkService::add_txt(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  char *tmp;
  if (vasprintf(&tmp, format, arg) == -1) {
    throw OutOfMemoryException("Cannot add txt record, no memory");
  }
  list.push_back(tmp);
  free(tmp);
  va_end(arg);
}


/** Set TXT records all at once.
 * @param txtlist list of TXT records
 */
void
NetworkService::set_txt(std::list<std::string> &txtlist)
{
  list = txtlist;
}


/** Set name of service.
 * @param new_name new name
 */
void
NetworkService::set_name(const char *new_name)
{
  free( _name );
  _name = strdup(new_name);
}


/** Get name of service.
 * @return name of service
 */
const char *
NetworkService::name() const
{
  return _name;
}


/** Set modified name of service.
 * The modified name is the original name with a suffix that has been added
 * to resolve a name collision.
 * @param new_name new name
 */
void
NetworkService::set_modified_name(const char *new_name) const
{
  if (_modified_name)  free(_modified_name);
  _modified_name = strdup(new_name);
}


/** Get modified name of service.
 * The modified name is the original name with a suffix that has been added
 * to resolve a name collision.
 * @return modified name of service, this may be NULL if the name has not
 * been modified
 */
const char *
NetworkService::modified_name() const
{
  return _modified_name;
}


/** Get type of service.
 * @return type of service
 */
const char *
NetworkService::type() const
{
  return _type;
}


/** Get domain of service.
 * @return domain of service
 */
const char *
NetworkService::domain() const
{
  return _domain;
}


/** Get host of service.
 * @return host of service
 */
const char *
NetworkService::host() const
{
  return _host;
}


/** Get port of service.
 * @return port of service
 */
unsigned short int
NetworkService::port() const
{
  return _port;
}


/** Get IP address of entry as string.
 * @return IP address as string
 * @exception NullPointerException thrown if the address has not been set
 */
std::string
NetworkService::addr_string() const
{
  char ipaddr[INET_ADDRSTRLEN];
  struct sockaddr_in *saddr = (struct sockaddr_in *)_addr;
  return std::string(inet_ntop(AF_INET, &(saddr->sin_addr), ipaddr, sizeof(ipaddr)));
}


/** Get TXT record list of service.
 * @return TXT record list of service
 */
const std::list<std::string> &
NetworkService::txt() const
{
  return list;
}


/** Equal operator for NetworkService reference.
 * @param s reference of service to compare to.
 * @return true, if the services are the same (same name and type), false otherwise
 */
bool
NetworkService::operator==(const NetworkService &s) const
{
  return ( (strcmp(_name, s._name) == 0) &&
	   (strcmp(_type, s._type) == 0) );
}


/** Equal operator for NetworkService pointer.
 * @param s pointer to service to compare to.
 * @return true, if the services are the same (same name and type), false otherwise
 */
bool
NetworkService::operator==(const NetworkService *s) const
{
  return ( (strcmp(_name, s->_name) == 0) &&
	   (strcmp(_type, s->_type) == 0) );
}


/** Less than operator.
 * @param s reference of service to compare to
 * @return true, if either the type is less than (according to strcmp) or if types
 * are equal if the service name is less than the given service's name.
 */
bool
NetworkService::operator<(const NetworkService &s) const
{
  int typediff = strcmp(_type, s._type);
  if ( typediff == 0 ) {
    return (strcmp(_name, s._name) < 0);
  } else {
    return (typediff < 0);
  }
}

} // end namespace fawkes
