
/***************************************************************************
 *  plugin_list_messages.cpp - Fawkes Plugin List Message
 *
 *  Created: Sat Jun 02 01:25:48 2007
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

#include <plugin/net/list_message.h>

#include <netcomm/utils/dynamic_buffer.h>
#include <netcomm/fawkes/component_ids.h>
#include <core/exceptions/software.h>
#include <utils/misc/strndup.h>
#include <cstdlib>
#include <cstring>

namespace fawkes {

/** @class PluginListMessage <plugin/net/list_message.h>
 * Plugin list message.
 * A complex dynamic message with an arbitrary number of plugins. Uses
 * DynamicBuffer for the internal list of plugins and thus the buffer is
 * limited to 64 KB.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
PluginListMessage::PluginListMessage()
{
  plugin_list = new DynamicBuffer(&(msg.plugin_list));
}


/** Message content constructor.
 * This constructor is meant to be used with FawkesNetworkMessage::msgc().
 * @param component_id component ID
 * @param msg_id message ID
 * @param payload message payload
 * @param payload_size total payload size
 */
PluginListMessage::PluginListMessage(unsigned int component_id,
				     unsigned int msg_id,
				     void *payload, size_t payload_size)
{
  if ( component_id != FAWKES_CID_PLUGINMANAGER ) {
    throw TypeMismatchException("PluginListMessage: invalid component ID");
  }
  plugin_list_msg_t *tmsg = (plugin_list_msg_t *)payload;
  void *plugin_list_payload = (void *)((size_t)payload + sizeof(msg));
  plugin_list = new DynamicBuffer(&(tmsg->plugin_list), plugin_list_payload,
				  payload_size - sizeof(msg));
}


/** Destructor. */
PluginListMessage::~PluginListMessage()
{
  delete plugin_list;
  if (_payload != NULL) {
    free(_payload);
    _payload = NULL;
    _payload_size = 0;
  }
}


/** Append plugin name.
 * @param plugin_name plugin name
 * @param len length in bytes to append (can be used for example to avoid
 * adding a file extension.
 */
void
PluginListMessage::append(const char *plugin_name, size_t len)
{
  plugin_list->append(plugin_name, len);
}


void
PluginListMessage::serialize()
{
  _payload_size = sizeof(msg) + plugin_list->buffer_size();
  _payload = malloc(_payload_size);
  copy_payload(0, &msg, sizeof(msg));
  copy_payload(sizeof(msg), plugin_list->buffer(), plugin_list->buffer_size());
}


/** Reset iterator.
 * For incoming messages only.
 */
void
PluginListMessage::reset_iterator()
{
  plugin_list->reset_iterator();
}


/** Check if more list elements are available.
 * For incoming messages only.
 * @return true if there are more elements available, false otherwise.
 */
bool
PluginListMessage::has_next()
{
  return plugin_list->has_next();
}


/** Get next plugin from list.
 * @return next plugin from list. This string has been allocated via strndup, so
 * you have to free it yourself!
 */
char *
PluginListMessage::next()
{
  size_t size;
  void *tmp = plugin_list->next(&size);
  return strndup((const char *)tmp, size);
}

} // end namespace fawkes
