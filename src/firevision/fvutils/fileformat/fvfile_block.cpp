
/***************************************************************************
 *  fvfile_block.cpp - FireVision file block
 *
 *  Created: Fri Mar 28 11:52:45 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/fileformat/fvfile_block.h>

#include <cstdlib>
#include <cstring>


namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FireVisionDataFileBlock <fvutils/fileformat/fvfile_block.h>
 * FireVision File Format data block.
 * This class describes one data block inside a FVFF file.
 * @author Tim Niemueller
 */

/** @var void * FireVisionDataFileBlock::_data
 * Pointer to the internal data segment.
 * Never free or modify the pointer, but only deal with the data it points to.
 */
/** @var size_t FireVisionDataFileBlock::_data_size
 * Size of _data in bytes.
 */
/** @var void * FireVisionDataFileBlock::_spec_header
 * Pointer to the content specific block header.
 * Never free or modify the pointer, but only deal with the data it points to.
 */

/** Constructor.
 * @param type block type, content specific
 * @param data_size size of the data segment
 * @param spec_header content-specific header
 * @param spec_header_size size of spec_header in bytes
 */
FireVisionDataFileBlock::FireVisionDataFileBlock(unsigned int type, size_t data_size,
						 void *spec_header, size_t spec_header_size)
{
  constructor(type, data_size, spec_header, spec_header_size);
}


/** Constructor.
 * @param type block type, content specific
 * @param data_size size of the data segment
 * @param spec_header_size a specific header of the given size is created internally
 */
FireVisionDataFileBlock::FireVisionDataFileBlock(unsigned int type, size_t data_size,
						 size_t spec_header_size)
{
  constructor(type, data_size, NULL, spec_header_size);
}


/** Constructor.
 * Specific header is assumed to be unused.
 * @param type block type, content specific
 * @param data_size size of the data segment
 */
FireVisionDataFileBlock::FireVisionDataFileBlock(unsigned int type, size_t data_size)
{
  constructor(type, data_size, NULL, 0);
}


/** Shallow copy constructor.
 * This creates a shallow copy of the given block. "Shallow" means that the data is not
 * copied but referenced. This instance is only valid as long as the original instance
 * still exists.
 * @param block block to copy
 */
FireVisionDataFileBlock::FireVisionDataFileBlock(FireVisionDataFileBlock *block)
{
  _data_size         = block->_data_size;
  __spec_header_size = block->__spec_header_size;
  __block_size       = block->__block_size;
  __block_memptr     = block->__block_memptr;
  __block_header     = (fvff_block_header_t *)__block_memptr;
  _spec_header       = (char *)__block_memptr + sizeof(fvff_block_header_t);
  _data              = (char *)__block_memptr + sizeof(fvff_block_header_t) + __spec_header_size;
  __block_owner      = false;
}


/** Internal constructor.
 * @param type block type, content specific
 * @param data_size size of the data segment
 * @param spec_header content-specific header
 * @param spec_header_size size of spec_header in bytes
 */
void
FireVisionDataFileBlock::constructor(unsigned int type, size_t data_size,
				     void *spec_header, size_t spec_header_size)
{
  _data_size         = data_size;
  __spec_header_size = spec_header_size;
  __block_size       = _data_size + sizeof(fvff_block_header_t) + spec_header_size;

  __block_memptr = calloc(1, __block_size);
  __block_owner  = true;
  __block_header = (fvff_block_header_t *)__block_memptr;
  _spec_header   = (char *)__block_memptr + sizeof(fvff_block_header_t);
  _data          = (char *)__block_memptr + sizeof(fvff_block_header_t) + spec_header_size;

  if ( (spec_header != NULL) && (spec_header_size > 0) ) {
    memcpy((char *)__block_memptr + sizeof(fvff_block_header_t), spec_header, spec_header_size);
  }

  __block_header->type = type;
  __block_header->size = _data_size;
  __block_header->spec_head_size = spec_header_size;
}


/** Set content-specific header.
 * If necessary this re-creates internal buffers. To avoid this use the three-parameter
 * ctor to have it account for the expected header size.
 * @param spec_header content-specific header
 * @param spec_header_size size of spec_header in bytes
 */
void
FireVisionDataFileBlock::set_spec_header(void *spec_header, size_t spec_header_size)
{
  if( spec_header_size != __spec_header_size ) {
    // we need to re-create the memory and copy old data
    __spec_header_size = spec_header_size;
    __block_size       = _data_size + sizeof(fvff_block_header_t) + spec_header_size;

    void *newblock = calloc(1, __block_size);

    memcpy(newblock, __block_memptr, sizeof(fvff_block_header_t));
    memcpy((char *)newblock + sizeof(fvff_block_header_t) + spec_header_size, _data, _data_size);

    free(__block_memptr);
    __block_memptr = newblock;
    __block_header = (fvff_block_header_t *)__block_memptr;
    _spec_header = (char *)__block_memptr + sizeof(fvff_block_header_t);
    _data = (char *)__block_memptr + sizeof(fvff_block_header_t) + spec_header_size;

    __block_header->spec_head_size = spec_header_size;
  }

  if ( (spec_header != NULL) && (spec_header_size > 0) ) {
    memcpy((char *)__block_memptr + sizeof(fvff_block_header_t), spec_header, spec_header_size);
  }
}


/** Destructor. */
FireVisionDataFileBlock::~FireVisionDataFileBlock()
{
  if ( __block_owner) {
    free(__block_memptr);
  }
}


/** Get block type.
 * @return block type ID, content specific
 */
unsigned int
FireVisionDataFileBlock::type() const
{
  return __block_header->type;
}


/** Pointer to the whole block.
 * @return pointer to whole block, including headers
 */
void *
FireVisionDataFileBlock::block_memptr() const
{
  return __block_memptr;
}


/** Size of blocks.
 * @return size of blocks in bytes.
 */
size_t
FireVisionDataFileBlock::block_size() const
{
  return __block_size;
}


/** Get data pointer.
 * @return pointer to the data segment of the block
 */
void *
FireVisionDataFileBlock::data_ptr() const
{
  return _data;
}


/** Size of data chunk.
 * @return size of data in bytes.
 */
size_t
FireVisionDataFileBlock::data_size() const
{
  return _data_size;
}

} // end namespace firevision
