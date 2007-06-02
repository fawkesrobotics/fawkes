
/***************************************************************************
 *  plugin_list_messages.h - Fawkes Plugin Messages
 *
 *  Created: Sat Jun 02 01:21:03 2007
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

#ifndef __FAWKES_PLUGIN_LIST_MESSAGE_H_
#define __FAWKES_PLUGIN_LIST_MESSAGE_H_

#include "plugin_messages.h"
#include <netcomm/fawkes/message_content.h>
#include <netcomm/utils/dynamic_buffer.h>

class DynamicBuffer;

class PluginListMessage : public FawkesNetworkMessageContent
{
 public:
  PluginListMessage();
  PluginListMessage(unsigned int component_id, unsigned int msg_id,
		    void *payload, size_t payload_size);
  virtual ~PluginListMessage();

  void append(const char *plugin_name, size_t len);
  virtual void serialize();

  void   reset_iterator();
  bool   has_next();
  char * next();
  

 private:
  DynamicBuffer     *plugin_list;
  plugin_list_msg_t  msg;
};

#endif
