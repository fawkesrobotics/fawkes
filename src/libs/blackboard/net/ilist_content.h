
/***************************************************************************
 *  net_ilist_content.h - BlackBoard network: interface list content
 *
 *  Created: Mon Mar 03 12:00:39 2008 (before thesis topic switch)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_NET_ILIST_CONTENT_H_
#define __BLACKBOARD_NET_ILIST_CONTENT_H_

#include <blackboard/net/messages.h>
#include <netcomm/fawkes/message_content.h>
#include <interface/interface_info.h>

namespace fawkes {

class DynamicBuffer;

class BlackBoardInterfaceListContent : public FawkesNetworkMessageContent
{
 public:
  BlackBoardInterfaceListContent();
  BlackBoardInterfaceListContent(unsigned int component_id, unsigned int msg_id,
				 void *payload, size_t payload_size);
  virtual ~BlackBoardInterfaceListContent();

  void append_interface(const char *type, const char *id, const unsigned char *hash,
			unsigned int serial, bool has_writer, unsigned int num_readers,
			const fawkes::Time &time);
  void append_interface(InterfaceInfo &info);

  virtual void serialize();

  void               reset_iterator();
  bool               has_next();
  bb_iinfo_msg_t *   next(size_t *size);

 private:
  DynamicBuffer     *interface_list;
  bb_ilist_msg_t     msg;
};

} // end namespace fawkes

#endif
