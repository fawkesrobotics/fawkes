
/***************************************************************************
 *  fvfile_block.h - FireVision file block
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

#ifndef __FIREVISION_FVUTILS_FILEFORMAT_FVFILE_BLOCK_H_
#define __FIREVISION_FVUTILS_FILEFORMAT_FVFILE_BLOCK_H_

#include <fvutils/fileformat/fvff.h>
#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FireVisionDataFileBlock
{
 public:
  FireVisionDataFileBlock(unsigned int type, size_t data_size,
			  void *spec_header, size_t spec_header_size);
  FireVisionDataFileBlock(unsigned int type, size_t data_size,
			  size_t spec_header_size);
  FireVisionDataFileBlock(unsigned int type, size_t data_size);
  FireVisionDataFileBlock(FireVisionDataFileBlock *block);
  virtual ~FireVisionDataFileBlock();

  unsigned int  type() const;
  void *        block_memptr() const;
  size_t        block_size() const;
  void *        data_ptr() const;
  size_t        data_size() const;

 protected:
  void set_spec_header(void *spec_header, size_t spec_header_size);

  void   *_data;
  size_t  _data_size;
  void   *_spec_header;

 private:
  void constructor(unsigned int type, size_t data_size,
		   void *spec_header, size_t spec_header_size);

  fvff_block_header_t *__block_header;
  void                *__block_memptr;
  size_t               __block_size;
  bool                 __block_owner;

  size_t               __spec_header_size;
};

} // end namespace firevision

#endif
