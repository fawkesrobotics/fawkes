
/***************************************************************************
 *  net_list_content.h - Fawkes Network Config List Message
 *
 *  Created: Fri Dec 07 14:52:59 2007
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

#ifndef __CONFIG_NET_LIST_CONTENT_H_
#define __CONFIG_NET_LIST_CONTENT_H_

#include <config/net_messages.h>
#include <config/config.h>
#include <netcomm/fawkes/message_content.h>

namespace fawkes {

class DynamicBuffer;

class ConfigListContent : public FawkesNetworkMessageContent
{
 public:
  ConfigListContent();
  ConfigListContent(unsigned int component_id, unsigned int msg_id,
		    void *payload, size_t payload_size);
  virtual ~ConfigListContent();

  void append(Configuration::ValueIterator *i);

  virtual void serialize();

  void                           reset_iterator();
  bool                           has_next();
  config_list_entity_header_t *  next(size_t *size);

 private:
  DynamicBuffer     *config_list;
  config_list_msg_t  msg;
};

} // end namespace fawkes

#endif
