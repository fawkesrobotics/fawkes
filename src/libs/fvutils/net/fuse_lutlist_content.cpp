
/***************************************************************************
 *  fuse_lutlist_content.cpp - FUSE LUT list content encapsulation
 *
 *  Created: Wed Nov 21 16:33:56 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/net/fuse_lutlist_content.h>
#include <netcomm/utils/dynamic_buffer.h>

#include <core/exceptions/software.h>

#include <cstdlib>
#include <cstring>
#include <netinet/in.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FuseLutListContent <fvutils/net/fuse_lutlist_content.h>
 * FUSE lookup table list content.
 * This content provides means to send an arbitrary length list of LUT
 * information chunks.
 * @author Tim Niemueller
 * @ingroup FUSE
 * @ingroup FireVision
 */

/** Constructor.
 * Creates an empty list.
 */
FuseLutListContent::FuseLutListContent()
{
  __list = new DynamicBuffer(&(__lutlist_msg.lut_list));
  
  _payload_size = 0;
  _payload = NULL;
}


/** Parsing constructor.
 * Can be used with the FuseContent::fmsg() method to get correctly parsed output.
 * @param type message type, must be FUSE_MT_LUT_LIST
 * @param payload payload
 * @param payload_size size of payload
 * @exception TypeMismatchException thrown if the type is not FUSE_MT_LUT_LIST
 */
FuseLutListContent::FuseLutListContent(uint32_t type, void *payload, size_t payload_size)
{
  FUSE_lutlist_message_t *tmsg = (FUSE_lutlist_message_t *)payload;
  void *list_payload = (void *)((size_t)payload + sizeof(FUSE_lutlist_message_t));
  __list = new DynamicBuffer(&(tmsg->lut_list), list_payload,
			     payload_size - sizeof(FUSE_lutlist_message_t));
}


/** Destructor. */
FuseLutListContent::~FuseLutListContent()
{
  delete __list;
}


/** Add LUT info.
 * @param lut_id LUT ID
 * @param width width of LUT
 * @param height height of LUT
 * @param depth depth of LUT
 * @param bytes_per_cell bytes per cell
 */
void
FuseLutListContent::add_lutinfo(const char *lut_id,
				unsigned int width, unsigned int height,
				unsigned int depth, unsigned int bytes_per_cell)
{
  FUSE_lutinfo_t lutinfo;
  memset(&lutinfo, 0, sizeof(lutinfo));

  strncpy(lutinfo.lut_id, lut_id, LUT_ID_MAX_LENGTH-1);
  lutinfo.width = ntohl(width);
  lutinfo.height = ntohl(height);
  lutinfo.depth  = ntohl(depth);  
  lutinfo.bytes_per_cell = ntohl(bytes_per_cell);

  __list->append(&lutinfo, sizeof(lutinfo));
}


/** Reset iterator. */
void
FuseLutListContent::reset_iterator()
{
  __list->reset_iterator();
}


/** Check if another LUT info is available.
 * @return true if another LUT info is available, false otherwise
 */
bool
FuseLutListContent::has_next()
{
  return __list->has_next();
}


/** Get next LUT info.
 * @return next LUT info
 * @exception TypeMismatchException thrown if the content contained invalid data
 * @exception OutOfBoundsException thrown if no more data is available
 */
FUSE_lutinfo_t *
FuseLutListContent::next()
{
  size_t size;
  void *tmp = __list->next(&size);
  if ( size != sizeof(FUSE_lutinfo_t) ) {
    throw TypeMismatchException("Lut list content contains element that is of an "
				"unexpected size");
  }

  return (FUSE_lutinfo_t *)tmp;
}


void
FuseLutListContent::serialize()
{
  _payload_size = sizeof(FUSE_lutlist_message_t) + __list->buffer_size();
  _payload = malloc(_payload_size);

  copy_payload(0, &__lutlist_msg, sizeof(FUSE_lutlist_message_t));
  copy_payload(sizeof(FUSE_lutlist_message_t), __list->buffer(), __list->buffer_size());
}

} // end namespace firevision
