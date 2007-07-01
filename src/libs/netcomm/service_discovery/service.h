
/***************************************************************************
 *  avahi_service.h - Avahi service representation
 *
 *  Generated: Tue Nov 07 17:58:10 2006
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

  ~NetworkService();

  void add_txt(const char *txt);

  void                set_name(const char *new_name);

  const char *        name();
  const char *        type();
  const char *        domain();
  const char *        host();
  unsigned short int  port();
  std::list<std::string>  txt();

  bool                operator==(const NetworkService &s) const;
  bool                operator==(const NetworkService *s) const;

 private:
  std::list<std::string> list;
  char *              _name;
  char *              _type;
  char *              _domain;
  char *              _host;
  unsigned short int  _port;

};

#endif
