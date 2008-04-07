
/***************************************************************************
 *  config_list_content.cpp - Fawkes Config List Message Content
 *
 *  Created: Sat Dec 08 23:38:10 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <config/net_list_content.h>

#include <netcomm/utils/dynamic_buffer.h>
#include <netcomm/fawkes/component_ids.h>
#include <core/exceptions/software.h>
#include <cstdlib>
#include <cstring>

/** @class ConfigListContent <config/net_list_content.h>
 * Config list content.
 * A complex dynamic message with an arbitrary number of config entities. Uses
 * DynamicBuffer for the internal list of plugins and thus the buffer is
 * limited to 4 GB in total.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ConfigListContent::ConfigListContent()
{
  config_list = new DynamicBuffer(&(msg.config_list));
}


/** Message content constructor.
 * This constructor is meant to be used with FawkesNetworkMessage::msgc().
 * @param component_id component ID
 * @param msg_id message ID
 * @param payload message payload
 * @param payload_size total payload size
 */
ConfigListContent::ConfigListContent(unsigned int component_id,
				     unsigned int msg_id,
				     void *payload, size_t payload_size)
{
  if ( component_id != FAWKES_CID_CONFIGMANAGER ) {
    throw TypeMismatchException("ConfigListContent: invalid component ID");
  }
  config_list_msg_t *tmsg = (config_list_msg_t *)payload;
  void *config_list_payload = (void *)((size_t)payload + sizeof(msg));
  config_list = new DynamicBuffer(&(tmsg->config_list), config_list_payload,
				  payload_size - sizeof(msg));
}


/** Destructor. */
ConfigListContent::~ConfigListContent()
{
  delete config_list;
  if (_payload != NULL) {
    free(_payload);
    _payload = NULL;
    _payload_size = 0;
  }
}


/** Append from iterator.
 * Appends the value the iterator points to.
 * @param i iterator
 */
void
ConfigListContent::append(Configuration::ValueIterator *i)
{
  if ( i->is_float() ) {
    append_float(i->path(), i->get_float(), i->is_default());
  } else if ( i->is_int() ) {
    append_int(i->path(), i->get_int(), i->is_default());
  } else if ( i->is_uint() ) {
    append_int(i->path(), i->get_uint(), i->is_default());
  } else if ( i->is_bool() ) {
    append_bool(i->path(), i->get_bool(), i->is_default());
  } else if ( i->is_string() ) {
    append_string(i->path(), i->get_string().c_str(), i->is_default());
  }
}

/** Append float value.
 * @param path of value
 * @param f float value
 * @param def_val true if this is a default value, false otherwise
 */
void
ConfigListContent::append_float(const char *path, float f, bool def_val)
{
  config_list_float_entity_t cle;
  memset(&cle, 0, sizeof(cle));
  strncpy(cle.header.cp.path, path, CONFIG_MSG_PATH_LENGTH);
  cle.header.type = MSG_CONFIG_FLOAT_VALUE;
  cle.header.cp.is_default = (def_val ? 1 : 0);
  cle.f = f;
  config_list->append(&cle, sizeof(cle));
}


/** Append integer value.
 * @param path of value
 * @param i integer value
 * @param def_val true if this is a default value, false otherwise
 */
void
ConfigListContent::append_int(const char *path, int i, bool def_val)
{
  config_list_int_entity_t cle;
  memset(&cle, 0, sizeof(cle));
  strncpy(cle.header.cp.path, path, CONFIG_MSG_PATH_LENGTH);
  cle.header.type = MSG_CONFIG_INT_VALUE;
  cle.header.cp.is_default = (def_val ? 1 : 0);
  cle.i = i;
  config_list->append(&cle, sizeof(cle));
}


/** Append unsigned integer value.
 * @param path of value
 * @param u unsigned integer value
 * @param def_val true if this is a default value, false otherwise
 */
void
ConfigListContent::append_uint(const char *path, unsigned int u, bool def_val)
{
  config_list_uint_entity_t cle;
  memset(&cle, 0, sizeof(cle));
  strncpy(cle.header.cp.path, path, CONFIG_MSG_PATH_LENGTH);
  cle.header.type = MSG_CONFIG_UINT_VALUE;
  cle.header.cp.is_default = (def_val ? 1 : 0);
  cle.u = u;
  config_list->append(&cle, sizeof(cle));
}


/** Append boolean value.
 * @param path of value
 * @param b boolean value
 * @param def_val true if this is a default value, false otherwise
 */
void
ConfigListContent::append_bool(const char *path, bool b, bool def_val)
{
  config_list_bool_entity_t cle;
  memset(&cle, 0, sizeof(cle));
  strncpy(cle.header.cp.path, path, CONFIG_MSG_PATH_LENGTH);
  cle.header.type = MSG_CONFIG_BOOL_VALUE;
  cle.header.cp.is_default = (def_val ? 1 : 0);
  cle.b = b;
  config_list->append(&cle, sizeof(cle));
}


/** Append string value.
 * @param path of value
 * @param s string value
 * @param def_val true if this is a default value, false otherwise
 */
void
ConfigListContent::append_string(const char *path, const char *s, bool def_val)
{
  config_list_string_entity_t cle;
  memset(&cle, 0, sizeof(cle));
  strncpy(cle.header.cp.path, path, CONFIG_MSG_PATH_LENGTH);
  cle.header.type = MSG_CONFIG_STRING_VALUE;
  cle.header.cp.is_default = (def_val ? 1 : 0);
  strncpy(cle.s, s, CONFIG_MSG_MAX_STRING_LENGTH);
  config_list->append(&cle, sizeof(cle));
}


void
ConfigListContent::serialize()
{
  _payload_size = sizeof(msg) + config_list->buffer_size();
  _payload = malloc(_payload_size);
  copy_payload(0, &msg, sizeof(msg));
  copy_payload(sizeof(msg), config_list->buffer(), config_list->buffer_size());
}


/** Reset iterator.
 * For incoming messages only.
 */
void
ConfigListContent::reset_iterator()
{
  config_list->reset_iterator();
}


/** Check if more list elements are available.
 * For incoming messages only.
 * @return true if there are more elements available, false otherwise.
 */
bool
ConfigListContent::has_next()
{
  return config_list->has_next();
}


/** Get next plugin from list.
 * @param size upon return contains the size of the returned data element.
 * @return next config entitiy from the list. The value is only of the type of
 * the header. Check the message type and the size and cast the message to the correct
 * entity.
 */
config_list_entity_header_t *
ConfigListContent::next(size_t *size)
{
  void *tmp = config_list->next(size);
  return (config_list_entity_header_t *)tmp;
}
