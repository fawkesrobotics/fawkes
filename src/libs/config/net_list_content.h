
/***************************************************************************
 *  net_list_content.h - Fawkes Network Config List Message
 *
 *  Created: Dec Fri 07 14:52:59 2007
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

#ifndef __CONFIG_NET_LIST_CONTENT_H_
#define __CONFIG_NET_LIST_CONTENT_H_

#include <config/net_messages.h>
#include <config/config.h>
#include <netcomm/fawkes/message_content.h>

class DynamicBuffer;

class ConfigListContent : public FawkesNetworkMessageContent
{
 public:
  ConfigListContent();
  ConfigListContent(unsigned int component_id, unsigned int msg_id,
		    void *payload, size_t payload_size);
  virtual ~ConfigListContent();

  void append_float(const char *path, float f, bool def_val = false);
  void append_int(const char *path, int i, bool def_val = false);
  void append_uint(const char *path, unsigned int u, bool def_val = false);
  void append_bool(const char *path, bool b, bool def_val = false);
  void append_string(const char *path, const char *s, bool def_val = false);
  void append(Configuration::ValueIterator *i);

  virtual void serialize();

  void                    reset_iterator();
  bool                    has_next();
  config_list_entity_t *  next();

 private:
  DynamicBuffer     *config_list;
  config_list_msg_t  msg;
};

#endif
