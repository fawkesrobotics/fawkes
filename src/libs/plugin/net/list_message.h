
/***************************************************************************
 *  plugin_list_messages.h - Fawkes Plugin Messages
 *
 *  Created: Sat Jun 02 01:21:03 2007
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

#ifndef _FAWKES_PLUGIN_LIST_MESSAGE_H_
#define _FAWKES_PLUGIN_LIST_MESSAGE_H_

#include <netcomm/fawkes/message_content.h>
#include <plugin/net/messages.h>

namespace fawkes {

class DynamicBuffer;

class PluginListMessage : public FawkesNetworkMessageContent
{
public:
	PluginListMessage();
	PluginListMessage(unsigned int component_id,
	                  unsigned int msg_id,
	                  void        *payload,
	                  size_t       payload_size);
	virtual ~PluginListMessage();

	void         append(const char *plugin_name, size_t len);
	virtual void serialize();

	void  reset_iterator();
	bool  has_next();
	char *next();

private:
	DynamicBuffer    *plugin_list;
	plugin_list_msg_t msg;
};

} // namespace fawkes

#endif
