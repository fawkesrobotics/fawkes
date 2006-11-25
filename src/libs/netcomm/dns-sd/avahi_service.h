
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

#ifndef __NETCOMM_DNSSD_AVAHI_SERVICE_H_
#define __NETCOMM_DNSSD_AVAHI_SERVICE_H_

typedef struct AvahiStringList AvahiStringList;

class AvahiService
{
 public:
  AvahiService(const char         *name,
	       const char         *type,
	       const char         *domain,
	       const char         *host,
	       unsigned short int  port);

  AvahiService(const char         *name,
	       const char         *type,
	       unsigned short int  port);

  ~AvahiService();

  void add_txt(const char *txt);

  void                set_name(const char *new_name);

  const char *        name();
  const char *        type();
  const char *        domain();
  const char *        host();
  unsigned short int  port();
  AvahiStringList *   txt();

  bool                operator==(const AvahiService &s) const;
  bool                operator==(const AvahiService *s) const;

 private:
  AvahiStringList *    list;
  char *              _name;
  char *              _type;
  char *              _domain;
  char *              _host;
  unsigned short int  _port;

};

#endif
