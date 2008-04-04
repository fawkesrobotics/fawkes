
/***************************************************************************
 *  service.h - Network service representation
 *
 *  Generated: Tue Nov 07 17:58:10 2006
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

#ifndef __NETCOMM_SERVICE_DISCOVERY_SERVICE_H_
#define __NETCOMM_SERVICE_DISCOVERY_SERVICE_H_

#include <string>
#include <list>

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
		 unsigned short int  port);

  NetworkService(const NetworkService *s);
  NetworkService(const NetworkService &s);
  ~NetworkService();

  void add_txt(const char *txt);

  void                set_name(const char *new_name);

  const char *        name() const;
  const char *        type() const;
  const char *        domain() const;
  const char *        host() const;
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

};

#endif
