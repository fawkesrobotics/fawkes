
/***************************************************************************
 *  service.h - Network service representation
 *
 *  Generated: Tue Nov 07 17:58:10 2006
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

#ifndef __NETCOMM_SERVICE_DISCOVERY_SERVICE_H_
#define __NETCOMM_SERVICE_DISCOVERY_SERVICE_H_

#include <string>
#include <list>

#include <sys/types.h>
#include <sys/socket.h>

namespace fawkes {

class NetworkNameResolver;

class NetworkService
{
 public:
  NetworkService(const char         *name,
		 const char         *type,
		 const char         *domain,
		 const char         *host,
		 unsigned short int  port);

  NetworkService(const char         *name,
		 const char         *type,
		 const char         *domain,
		 const char         *host,
		 unsigned short int  port,
		 const struct sockaddr *addr,
		 const socklen_t     addr_size,
		 std::list<std::string> &txt);

  NetworkService(const char         *name,
		 const char         *type,
		 unsigned short int  port);

  NetworkService(const char         *name,
		 const char         *type,
		 const char         *domain);

  NetworkService(NetworkNameResolver *nnresolver,
		 const char *name, const char *type,
		 unsigned short int port);

  NetworkService(const NetworkService *s);
  NetworkService(const NetworkService &s);
  ~NetworkService();

  void                add_txt(const char *format, ...);
  void                set_txt(std::list<std::string> &txtlist);

  void                set_name(const char *new_name);
  void                set_modified_name(const char *new_name) const;

  const char *        name() const;
  const char *        modified_name() const;
  const char *        type() const;
  const char *        domain() const;
  const char *        host() const;
  std::string         addr_string() const;
  unsigned short int  port() const;
  const std::list<std::string> & txt() const;

  bool                operator==(const NetworkService &s) const;
  bool                operator==(const NetworkService *s) const;
  bool                operator<(const NetworkService &s) const;

 private:
  std::list<std::string> list;
  char *              _name;
  char *              _type;
  char *              _domain;
  char *              _host;
  unsigned short int  _port;
  struct sockaddr    *_addr;
  socklen_t           _addr_size;

  mutable char *      _modified_name;
};

} // end namespace fawkes

#endif
