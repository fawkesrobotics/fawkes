
/***************************************************************************
 *  fuse_imagelist_content.cpp - FUSE image list content encapsulation
 *
 *  Created: Tue Nov 20 15:00:50 2007
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

#include <fvutils/net/fuse_imagelist_content.h>
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

/** @class FuseImageListContent <fvutils/net/fuse_imagelist_content.h>
 * FUSE image list content.
 * This content provides means to send an arbitrary length list of image
 * information chunks.
 * @author Tim Niemueller
 * @ingroup FUSE
 * @ingroup FireVision
 */

/** Constructor.
 * Creates an empty list.
 */
FuseImageListContent::FuseImageListContent()
{
  __list = new DynamicBuffer(&(__imagelist_msg.image_list));
  
  _payload_size = 0;
  _payload      = NULL;
}


/** Parsing constructor.
 * Can be used with the FuseMessage::msgc() method to get correctly parsed output.
 * @param type message type, must be FUSE_MT_IMAGE_LIST
 * @param payload payload
 * @param payload_size size of payload
 * @exception TypeMismatchException thrown if the type is not FUSE_MT_IMAGE_LIST
 */
FuseImageListContent::FuseImageListContent(uint32_t type, void *payload, size_t payload_size)
{
  if ( type != FUSE_MT_IMAGE_LIST ) {
    throw TypeMismatchException("Type %u not equal to expected type FUSE_MT_IMAGE_LIST (%u)",
				type, FUSE_MT_IMAGE_LIST);
  }
  FUSE_imagelist_message_t *tmsg = (FUSE_imagelist_message_t *)payload;
  void *list_payload = (void *)((size_t)payload + sizeof(FUSE_imagelist_message_t));
  __list = new DynamicBuffer(&(tmsg->image_list), list_payload,
			     payload_size - sizeof(FUSE_imagelist_message_t));
}


/** Destructor. */
FuseImageListContent::~FuseImageListContent()
{
  delete __list;
}


/** Add image info.
 * This can only be called on contents that have been newly created, it is
 * a bug to call this method on contents read from the network.
 * @param image_id image ID
 * @param colorspace colorspace
 * @param pixel_width width of image in pixels
 * @param pixel_height height of image in pixels
 */
void
FuseImageListContent::add_imageinfo(const char *image_id, colorspace_t colorspace,
				    unsigned int pixel_width, unsigned int pixel_height)
{
  FUSE_imageinfo_t imageinfo;
  memset(&imageinfo, 0, sizeof(imageinfo));

  strncpy(imageinfo.image_id, image_id, IMAGE_ID_MAX_LENGTH-1);
  imageinfo.colorspace = htons(colorspace);
  imageinfo.width = htonl(pixel_width);
  imageinfo.height = htonl(pixel_height);
  imageinfo.buffer_size = htonl(colorspace_buffer_size(colorspace, pixel_width, pixel_height));

  __list->append(&imageinfo, sizeof(imageinfo));
}


/** Reset iterator. */
void
FuseImageListContent::reset_iterator()
{
  __list->reset_iterator();
}


/** Check if another image info is available.
 * @return true if another image info is available, false otherwise
 */
bool
FuseImageListContent::has_next()
{
  return __list->has_next();
}


/** Get next image info.
 * @return next image info
 * @exception TypeMismatchException thrown if the content contained invalid data
 * @exception OutOfBoundsException thrown if no more data is available
 */
FUSE_imageinfo_t *
FuseImageListContent::next()
{
  size_t size;
  void *tmp = __list->next(&size);
  if ( size != sizeof(FUSE_imageinfo_t) ) {
    throw TypeMismatchException("Image list content contains element that is of an "
				"unexpected size");
  }

  return (FUSE_imageinfo_t *)tmp;
}


void
FuseImageListContent::serialize()
{
  _payload_size = sizeof(FUSE_imagelist_message_t) + __list->buffer_size();
  _payload = malloc(_payload_size);

  copy_payload(0, &__imagelist_msg, sizeof(FUSE_imagelist_message_t));
  copy_payload(sizeof(FUSE_imagelist_message_t), __list->buffer(), __list->buffer_size());
}

} // end namespace firevision
