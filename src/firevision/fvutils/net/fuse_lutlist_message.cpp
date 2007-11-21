
/***************************************************************************
 *  fuse_lutlist_message.cpp - FUSE LUT list message encapsulation
 *
 *  Created: Wed Nov 21 16:33:56 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/net/fuse_lutlist_message.h>
#include <netcomm/utils/dynamic_buffer.h>

#include <cstdlib>
#include <cstring>
#include <netinet/in.h>

/** @class FuseLutListMessage <fvutils/net/fuse_lutlist_message.h>
 * FUSE lookup table list message.
 * This message provides means to send an arbitrary length list of LUT
 * information chunks.
 * @author Tim Niemueller
 * @ingroup FUSE
 * @ingroup FireVision
 */

/** Constructor.
 * Creates an empty list.
 */
FuseLutListMessage::FuseLutListMessage()
{
  __list = new DynamicBuffer(&(__lutlist_msg.lut_list));
  
  _msg.header.message_type = htonl(FUSE_MT_LUT_LIST);
  _msg.header.payload_size = htonl(0);
  _msg.payload = NULL;
}


/** Parsing constructor.
 * Can be used with the FuseMessage::fmsg() method to get correctly parsed output.
 * @param type message type, must be FUSE_MT_LUT_LIST
 * @param payload payload
 * @param payload_size size of payload
 * @exception TypeMismatchException thrown if the type is not FUSE_MT_LUT_LIST
 */
FuseLutListMessage::FuseLutListMessage(uint32_t type, void *payload, size_t payload_size)
{
  FUSE_lutlist_message_t *tmsg = (FUSE_lutlist_message_t *)payload;
  void *list_payload = (void *)((size_t)payload + sizeof(FUSE_lutlist_message_t));
  __list = new DynamicBuffer(&(tmsg->lut_list), list_payload,
			     payload_size - sizeof(FUSE_lutlist_message_t));
}


/** Destructor. */
FuseLutListMessage::~FuseLutListMessage()
{
  delete __list;
}


/** Add LUT info.
 * @param lut_id LUT ID
 * @param width width of LUT
 * @param height height of LUT
 * @param bytes_per_cell bytes per cell
 */
void
FuseLutListMessage::add_lutinfo(const char *lut_id,
				unsigned int width, unsigned int height,
				unsigned int bytes_per_cell)
{
  FUSE_lutinfo_t lutinfo;
  memset(&lutinfo, 0, sizeof(lutinfo));

  strncpy(lutinfo.lut_id, lut_id, LUT_ID_MAX_LENGTH);
  lutinfo.width = width;
  lutinfo.height = height;
  lutinfo.bytes_per_cell = bytes_per_cell;

  __list->append(&lutinfo, sizeof(lutinfo));
}


/** Reset iterator. */
void
FuseLutListMessage::reset_iterator()
{
  __list->reset_iterator();
}


/** Check if another LUT info is available.
 * @return true if another LUT info is available, false otherwise
 */
bool
FuseLutListMessage::has_next()
{
  return __list->has_next();
}


/** Get next LUT info.
 * @return next LUT info
 * @exception TypeMismatchException thrown if the message contained invalid data
 * @exception OutOfBoundsException thrown if no more data is available
 */
FUSE_lutinfo_t *
FuseLutListMessage::next()
{
  size_t size;
  void *tmp = __list->next(&size);
  if ( size != sizeof(FUSE_lutinfo_t) ) {
    throw TypeMismatchException("Lut list message contains element that is of an "
				"unexpected size");
  }

  return (FUSE_lutinfo_t *)tmp;
}


void
FuseLutListMessage::pack()
{
  unsigned int payload_size = sizeof(FUSE_lutlist_message_t) + __list->buffer_size();
  _msg.header.payload_size = htonl(payload_size);
  _msg.payload = malloc(payload_size);

  copy_payload(0, &__lutlist_msg, sizeof(FUSE_lutlist_message_t));
  copy_payload(sizeof(FUSE_lutlist_message_t), __list->buffer(), __list->buffer_size());
}
