
/***************************************************************************
 *  net_ilist_content.cpp - BlackBoard network: interface list content
 *
 *  Created: Mon Mar 03 12:04:51 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/net_ilist_content.h>

#include <netcomm/utils/dynamic_buffer.h>
#include <netcomm/fawkes/component_ids.h>
#include <core/exceptions/software.h>
#include <cstdlib>
#include <cstring>

/** @class BlackBoardInterfaceListContent <blackboard/net_ilist_content.h>
 * BlackBoard interface list content.
 * A complex dynamic message with an arbitrary number of interfaces. Uses
 * DynamicBuffer for the internal list of plugins and thus the buffer is
 * limited to 4 GB in total.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
BlackBoardInterfaceListContent::BlackBoardInterfaceListContent()
{
  interface_list = new DynamicBuffer(&(msg.interface_list));
}


/** Message content constructor.
 * This constructor is meant to be used with FawkesNetworkMessage::msgc().
 * @param component_id component ID
 * @param msg_id message ID
 * @param payload message payload
 * @param payload_size total payload size
 */
BlackBoardInterfaceListContent::BlackBoardInterfaceListContent(unsigned int component_id,
							       unsigned int msg_id,
							       void *payload,
							       size_t payload_size)
{
  if ( component_id != FAWKES_CID_BLACKBOARD ) {
    throw TypeMismatchException("BlackBoardInterfaceListContent: invalid component ID");
  }
  bb_ilist_msg_t *tmsg = (bb_ilist_msg_t *)payload;
  void *ilist_payload = (void *)((size_t)payload + sizeof(msg));
  interface_list = new DynamicBuffer(&(tmsg->interface_list), ilist_payload,
				     payload_size - sizeof(msg));
}


/** Destructor. */
BlackBoardInterfaceListContent::~BlackBoardInterfaceListContent()
{
  delete interface_list;
  if (_payload != NULL) {
    free(_payload);
    _payload = NULL;
    _payload_size = 0;
  }
}



/** Append interface info.
 * @param type type of interface
 * @param id ID of interface instance
 * @param hash version hash of interface instance/type
 * @param serial instance serial
 * @param has_writer true if a writer exists, false otherwise
 * @param num_readers number of readers
 */
void
BlackBoardInterfaceListContent::append_interface(const char *type, const char *id,
						 const unsigned char *hash,
						 unsigned int serial,
						 bool has_writer, unsigned int num_readers)
{
  bb_iinfo_msg_t info;
  memset(&info, 0, sizeof(info));
  strncpy(info.type, type, __INTERFACE_TYPE_SIZE);
  strncpy(info.id, id, __INTERFACE_ID_SIZE);
  memcpy(info.hash, hash, __INTERFACE_HASH_SIZE);
  interface_list->append(&info, sizeof(info));
  info.serial      = serial;
  info.has_writer  = has_writer ? 1 : 0;
  info.num_readers = num_readers;
}


/** Append interface info.
 * @param iinfo interface info
 */
void
BlackBoardInterfaceListContent::append_interface(InterfaceInfo &iinfo)
{
  bb_iinfo_msg_t info;
  memset(&info, 0, sizeof(info));
  strncpy(info.type, iinfo.type(), __INTERFACE_TYPE_SIZE);
  strncpy(info.id, iinfo.id(), __INTERFACE_ID_SIZE);
  memcpy(info.hash, iinfo.hash(), __INTERFACE_HASH_SIZE);
  info.serial      = iinfo.serial();
  info.has_writer  = iinfo.has_writer() ? 1 : 0;
  info.num_readers = iinfo.num_readers();
  interface_list->append(&info, sizeof(info));
}


void
BlackBoardInterfaceListContent::serialize()
{
  _payload_size = sizeof(msg) + interface_list->buffer_size();
  _payload = malloc(_payload_size);
  copy_payload(0, &msg, sizeof(msg));
  copy_payload(sizeof(msg), interface_list->buffer(), interface_list->buffer_size());
}


/** Reset iterator.
 * For incoming messages only.
 */
void
BlackBoardInterfaceListContent::reset_iterator()
{
  interface_list->reset_iterator();
}


/** Check if more list elements are available.
 * For incoming messages only.
 * @return true if there are more elements available, false otherwise.
 */
bool
BlackBoardInterfaceListContent::has_next()
{
  return interface_list->has_next();
}


/** Get next plugin from list.
 * @param size upon return contains the size of the returned data element.
 * @return next config entitiy from the list. The value is only of the type of
 * the header. Check the message type and the size and cast the message to the correct
 * entity.
 */
bb_iinfo_msg_t *
BlackBoardInterfaceListContent::next(size_t *size)
{
  void *tmp = interface_list->next(size);
  return (bb_iinfo_msg_t *)tmp;
}
