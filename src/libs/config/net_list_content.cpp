
/***************************************************************************
 *  config_list_content.cpp - Fawkes Config List Message Content
 *
 *  Created: Sat Dec 08 23:38:10 2007
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

#include <config/net_list_content.h>

#include <netcomm/utils/dynamic_buffer.h>
#include <netcomm/fawkes/component_ids.h>
#include <core/exceptions/software.h>
#include <cstdlib>
#include <cstring>

namespace fawkes {

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

/// @cond INTERNAL
template <typename T>
static inline void
copy_data_vector(T *in, T *out, const size_t num_values)
{
  for (unsigned int j = 0; j < num_values; ++j) {
    out[j] = in[j];
  }
}
/// @endcond

/** Append from iterator.
 * Appends the value the iterator points to.
 * @param i iterator
 */
void
ConfigListContent::append(Configuration::ValueIterator *i)
{
  unsigned int num_values = (i->is_list() ? i->get_list_size() : 1);
  size_t data_size = 0;
  char *data;

  config_list_entity_header_t cle;
  memset(&cle, 0, sizeof(cle));
  strncpy(cle.cp.path, i->path(), CONFIG_MSG_PATH_LENGTH);
  cle.type = MSG_CONFIG_FLOAT_VALUE;
  cle.cp.is_default = (i->is_default() ? 1 : 0);
  cle.cp.num_values = (i->is_list() ? i->get_list_size() : 0);

  if ( i->is_uint() ) {
    cle.type = MSG_CONFIG_UINT_VALUE;
    data_size = num_values * sizeof(uint32_t);
  } else if ( i->is_int() ) {
    cle.type = MSG_CONFIG_INT_VALUE;
    data_size = num_values * sizeof(int32_t);
  } else if ( i->is_bool() ) {
    cle.type = MSG_CONFIG_BOOL_VALUE;
    data_size = num_values * sizeof(int32_t);
  } else if ( i->is_float() ) {
    cle.type = MSG_CONFIG_FLOAT_VALUE;
    data_size = num_values * sizeof(float);
  } else if ( i->is_string() ) {
    cle.type = MSG_CONFIG_STRING_VALUE;
    if (num_values > 1) {
      std::vector<std::string> values = i->get_strings();
      for (unsigned int j = 0; j < values.size(); ++j) {
	data_size += sizeof(config_string_value_t) + values[j].length() + 1;
      }
    } else {
      data_size = sizeof(config_string_value_t) + i->get_string().length() + 1;
    }
  } else {
    throw TypeMismatchException("Invalid type of config iterator value");
  }

  data = (char *)malloc(sizeof(config_list_entity_header_t) + data_size);
  memcpy(data, &cle, sizeof(config_list_entity_header_t));

  if ( i->is_uint() ) {
    if (num_values > 1) {
      copy_data_vector(&i->get_uints()[0], (uint32_t *)(data + sizeof(cle)), num_values);
    } else {
      *((uint32_t *)(data + sizeof(cle))) = i->get_uint();
    }
  } else if ( i->is_int() ) {
    if (num_values > 1) {
      copy_data_vector(&i->get_ints()[0], (int32_t *)(data + sizeof(cle)), num_values);
    } else {
      *((int32_t *)(data + sizeof(cle))) = i->get_int();
    }
  } else if ( i->is_bool() ) {
    if (num_values > 1) {
      std::vector<bool> values = i->get_bools();
      int32_t *msg_values = (int32_t *)(data + sizeof(cle));
      for (unsigned int j = 0; j < values.size(); ++j) {
	msg_values[j] = values[j] ? 1 : 0;
      }

    } else {
      *((int32_t *)(data + sizeof(cle))) = i->get_bool() ? 1 : 0;
    }
  } else if ( i->is_float() ) {
    if (num_values > 1) {
      copy_data_vector(&i->get_floats()[0], (float *)(data + sizeof(cle)), num_values);
    } else {
      *((float *)(data + sizeof(cle))) = i->get_float();
    }
  } else if ( i->is_string() ) {
    if (num_values > 1) {
      std::vector<std::string> values = i->get_strings();
      char *tmpdata = (char *)data + sizeof(cle);
      for (unsigned int j = 0; j < values.size(); ++j) {
	config_string_value_t *csv = (config_string_value_t *)tmpdata;
	csv->s_length = values[j].length();
	char *msg_string = tmpdata + sizeof(config_string_value_t);
	strcpy(msg_string, values[j].c_str());
	tmpdata += sizeof(config_string_value_t) + values[j].length() + 1;
      }
    } else {
      config_string_value_t *csv = (config_string_value_t *)((char *)data + sizeof(cle));
      csv->s_length = i->get_string().length();
      char *msg_string = data + sizeof(cle) + sizeof(config_string_value_t);
      strcpy(msg_string, i->get_string().c_str());
    }
  }

  config_list->append(data, sizeof(cle) + data_size);
}


void
ConfigListContent::serialize()
{
  _payload_size = sizeof(msg) + config_list->buffer_size();
  _payload = calloc(1, _payload_size);
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

} // end namespace fawkes
