
/***************************************************************************
 *  rectinfo_block.cpp - Rectification info block encapsulation
 *
 *  Created: Wed Oct 31 14:35:36 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/rectification/rectinfo_block.h>
#include <core/exceptions/system.h>
#include <core/exceptions/software.h>

#include <cstdlib>
#include <cstring>

/** @class RectificationInfoBlock <fvutils/rectification/rectinfo_block.h>
 * Rectification info block.
 * This base class defines the basic interface to interact with rectification
 * info blocks. It manages a small memory chunk that may later be used via
 * other recitification information classes in an easy manner. Concrete
 * implementations of a specific block type shall be derived from this
 * class.
 * @author Tim Niemueller
 */

/** @var void * RectificationInfoBlock::_block_chunk
 * Pointer to the internal memory. This points to the raw memory chunk where
 * all data is stored (type agnostic header, type specific header, data).
 */
/** @var void * RectificationInfoBlock::_block_data
 * Pointer to the data part of the chunk.
 */
/** @var size_t RectificationInfoBlock::_block_size
 * Total size of _block_chunk, includes type agnostic header, type specific
 * header and data.
 */
/** @var bool RectificationInfoBlock::_free_block_chunk
 * If true the memory chunk is freed using the standard C free() function,
 * otherwise it is left untouched on destruction.
 */
/** @var rectinfo_block_header_t RectificationInfoBlock::_block_header
 * Pointer to type agnostic block header.
 */

/** Basic constructor.
 * Use this constructor if you want to maintain everything regarding the memory
 * *and* the block header by yourself. Only use this rarely and think your
 * scenario through twice before going along this road.
 */
RectificationInfoBlock::RectificationInfoBlock()
{
  _block_chunk      = NULL;
  _block_size       = 0;
  _block_header     = NULL;
  _block_data       = NULL;
  _free_block_chunk = false;
}


/** Recommended constructor.
 * With this constructor a chunk of memory is allocated that is sufficient
 * to hold the internal block header and the data of the given size. Note
 * that the size you give is only meant to hold your type specific header
 * and data. Some extra bytes are internally added for the type agnostic
 * block header.
 * @param block_type type of the block as defined per rectinfo_block_type_t
 * @param camera camera identifier
 * @param block_data_size size of the data block, this means only the sum of
 * the size of the type specific header and the data itself, NOT including
 * the type agnostic block header.
 */
RectificationInfoBlock::RectificationInfoBlock(uint8_t block_type,
					       uint8_t camera,
					       size_t block_data_size)
{
  _block_size = block_data_size + sizeof(rectinfo_block_header_t);

  if ( _block_size > UINT32_MAX ) {
    throw OutOfBoundsException("RectInfoBlock: block_data_size is too large",
			       block_data_size, 0, UINT32_MAX);
  }

  _block_chunk = malloc(_block_size);
  if ( ! _block_chunk) {
    throw OutOfMemoryException("Cannot allocate memory for rectinfo block");
  }
  _free_block_chunk = true;

  memset(_block_chunk, 0, _block_size);

  _block_header = (rectinfo_block_header_t *)_block_chunk;
  _block_data   = (char *)_block_chunk + sizeof(rectinfo_block_header_t);

  _block_header->type   = block_type;
  _block_header->camera = camera;
  _block_header->size   = block_data_size;
}


/** Read constructor.
 * This constructor uses the given memory chunk. It takes the ownership of this
 * chunk. This constructor is meant to be used if data is read. Obey it for your
 * own implementation of a block format!
 * @param chunk memory chunk, has to include the type agnostic block header and
 * any type specific headers and data.
 * @param chunk_size size of the chunk. Has to include the size of the type
 * agnostic block header, the type specific header and the data
 */
RectificationInfoBlock::RectificationInfoBlock(void * chunk,
					       size_t chunk_size)
{
  _block_size  = chunk_size;
  _block_chunk = chunk;
  _free_block_chunk = true;

  _block_header = (rectinfo_block_header_t *)_block_chunk;
  _block_data   = (char *)_block_chunk + sizeof(rectinfo_block_header_t);
}


/** Destructor.
 * Destructs the chunk, if and only if _free_block_chunk is true.
 */
RectificationInfoBlock::~RectificationInfoBlock()
{
  if ( _free_block_chunk ) {
    if ( _block_chunk != NULL ) {
      free(_block_chunk);
    }
  }
  _block_chunk  = NULL;
  _block_size   = 0;
  _block_data   = NULL;
  _block_header = NULL;
}


/** Get pointer to memory chunk.
 * @return pointer to internal memory chunk. This chunk has to include
 * the type agnostic header, the (optional) type specific header and
 * the data size.
 */
void *
RectificationInfoBlock::block_memptr() const
{
  return _block_chunk;
}


/** Get chunk size.
 * @return size of internal memory chunk. This chunk has to include
 * the type agnostic header, the (optional) type specific header and
 * the data size.
 */
size_t
RectificationInfoBlock::block_size() const
{
  return _block_size;
}


/** Get block type.
 * @return block type
 * @see rectinfo_block_header_t
 */
uint8_t
RectificationInfoBlock::type() const
{
  if ( _block_header == NULL ) {
    throw NullPointerException("No memory chunk loaded for rectinfo block");
  }
  return _block_header->type;
}

/** Get block camera identifier.
 * @return camera identifier
 * @see rectinfo_block_header_t
 */
uint8_t
RectificationInfoBlock::camera() const
{
  if ( _block_header == NULL ) {
    throw NullPointerException("No memory chunk loaded for rectinfo block");
  }
  return _block_header->camera;
}


/** Get block size.
 * @return block size
 * @see rectinfo_block_header_t
 */
uint32_t
RectificationInfoBlock::size() const
{
  if ( _block_header == NULL ) {
    throw NullPointerException("No memory chunk loaded for rectinfo block");
  }
  return _block_header->size;
}
