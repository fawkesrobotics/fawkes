
/***************************************************************************
 *  fuse_lut_content.cpp - FUSE LUT content encapsulation
 *
 *  Created: Wed Nov 21 16:49:18 2007
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

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <fvutils/ipc/shm_lut.h>
#include <fvutils/net/fuse_lut_content.h>
#include <netinet/in.h>

#include <cstdlib>
#include <cstring>

namespace firevision {

/** @class FuseLutContent <fvutils/net/fuse_lut_content.h>
 * FUSE lookup table content.
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor.
 * @param type content type, must be FUSE_MT_LUT
 * @param payload payload
 * @param payload_size size of payload
 * @exception TypeMismatchException thrown if type does not equal FUSE_MT_LUT
 */
FuseLutContent::FuseLutContent(uint32_t type, void *payload, size_t payload_size)
{
	if ((type != FUSE_MT_LUT) && (type != FUSE_MT_SET_LUT)) {
		throw fawkes::TypeMismatchException("Type %u != FUSE_MT_LUT/FUSE_MT_SET_LUT (%u/%u)",
		                                    type,
		                                    FUSE_MT_LUT,
		                                    FUSE_MT_SET_LUT);
	}

	_payload_size = payload_size;
	_payload      = payload;

	header_ = (FUSE_lut_message_header_t *)_payload;
	buffer_ = (unsigned char *)_payload + sizeof(FUSE_lut_message_header_t);

	lut_id_                    = (char *)malloc(LUT_ID_MAX_LENGTH + 1);
	lut_id_[LUT_ID_MAX_LENGTH] = 0;
	strncpy(lut_id_, header_->lut_id, LUT_ID_MAX_LENGTH);

	buffer_size_ = (size_t)ntohl(header_->width) * ntohl(header_->height)
	               * (size_t)ntohl(header_->depth) * ntohl(header_->bytes_per_cell);
}

/** Constructor.
 * @param b lookup table to copy data from
 */
FuseLutContent::FuseLutContent(SharedMemoryLookupTable *b)
{
	buffer_size_  = (size_t)b->width() * b->height() * b->depth() * b->bytes_per_cell();
	_payload_size = buffer_size_ + sizeof(FUSE_lut_message_header_t);

	_payload = malloc(_payload_size);
	if (_payload == NULL) {
		throw fawkes::OutOfMemoryException("Cannot allocate FuseLutContent buffer");
	}

	header_ = (FUSE_lut_message_header_t *)_payload;
	buffer_ = (unsigned char *)_payload + sizeof(FUSE_lut_message_header_t);

	strncpy(header_->lut_id, b->lut_id(), LUT_ID_MAX_LENGTH - 1);
	header_->width          = htonl(b->width());
	header_->height         = htonl(b->height());
	header_->depth          = htonl(b->depth());
	header_->bytes_per_cell = htonl(b->bytes_per_cell());
	lut_id_                 = strdup(b->lut_id());

	// b->lock_for_read();
	memcpy(buffer_, b->buffer(), buffer_size_);
	// b->unlock();
}

/** Constructor.
 * Create a brand new FuseLutContent from a raw buffer.
 * @param lut_id LUT ID
 * @param buffer buffer that holds the LUT data
 * @param width LUT width
 * @param height LUT height
 * @param depth LUT depth
 * @param bpc LUT bytes per cell
 */
FuseLutContent::FuseLutContent(const char * lut_id,
                               void *       buffer,
                               unsigned int width,
                               unsigned int height,
                               unsigned int depth,
                               unsigned int bpc)
{
	buffer_size_  = (size_t)width * height * depth * bpc;
	_payload_size = buffer_size_ + sizeof(FUSE_lut_message_header_t);

	_payload = malloc(_payload_size);
	if (_payload == NULL) {
		throw fawkes::OutOfMemoryException("Cannot allocate FuseLutContent buffer");
	}

	header_ = (FUSE_lut_message_header_t *)_payload;
	buffer_ = (unsigned char *)_payload + sizeof(FUSE_lut_message_header_t);

	strncpy(header_->lut_id, lut_id, LUT_ID_MAX_LENGTH - 1);
	header_->width          = htonl(width);
	header_->height         = htonl(height);
	header_->depth          = htonl(depth);
	header_->bytes_per_cell = htonl(bpc);
	lut_id_                 = strdup(lut_id);

	memcpy(buffer_, buffer, buffer_size_);
}

FuseLutContent::~FuseLutContent()
{
	free(lut_id_);
}

/** Get LUT ID.
 * @return LUT ID
 */
const char *
FuseLutContent::lut_id() const
{
	return lut_id_;
}

/** Get buffer.
 * @return buffer
 */
unsigned char *
FuseLutContent::buffer() const
{
	return buffer_;
}

/** Get buffer size.
 * @return size of buffer returned by buffer()
 */
size_t
FuseLutContent::buffer_size() const
{
	return buffer_size_;
}

/** Width of LUT.
 * @return width of LUT
 */
unsigned int
FuseLutContent::width() const
{
	return ntohl(header_->width);
}

/** Height of LUT.
 * @return height of LUT
 */
unsigned int
FuseLutContent::height() const
{
	return ntohl(header_->height);
}

/** Depth of LUT.
 * @return depth of LUT
 */
unsigned int
FuseLutContent::depth() const
{
	return ntohl(header_->depth);
}

/** Bytes per cell in LUT.
 * @return Bytes per cell in LUT
 */
unsigned int
FuseLutContent::bytes_per_cell() const
{
	return ntohl(header_->bytes_per_cell);
}

void
FuseLutContent::serialize()
{
	// Nothing to do here
}

} // end namespace firevision
