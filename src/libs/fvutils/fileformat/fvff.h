
/***************************************************************************
 *  fvff.h - FireVision file format
 *
 *  Created: Fri Mar 28 11:12:38 2008
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

#ifndef __FIREVISION_FVUTILS_FILEFORMAT_FVFF_H_
#define __FIREVISION_FVUTILS_FILEFORMAT_FVFF_H_

#pragma pack(push,4)

#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif
#include <stdint.h>

#define FVFF_COMMENT_SIZE 256

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Header for a FireVision file format file.
 * The header defines the basic parameters needed to correctly interpret the
 * following file contents.
 *
 * The header defines a magic by which a rectinfo can be identified. This is
 * defined by the actual content of the file.
 * The version is stored as a sequential number. This version has to be changed
 * whenever either the header or the file data format changes. The version is set
 * by the concrete data implementation.
 * The file defines the endianess of the supplied data.
 * There are several reserved bits that may be used later to store flags. The field
 * num_blocks define how many info blocks there are in this file.
 *
 * Directly following the header is the content specific header. It has to be exactly
 * the size given in spec_head_size.
 */
typedef struct _fvff_header_t {
  uint16_t magic_token;		/**< magic token */
  uint16_t version    :  4;	/**< version of the data file, this header defines version 1 */
  uint16_t endianess  :  1;	/**< endianess of the file, 0 means little endian, 1 means big endian */
  uint16_t reserved   : 11;	/**< reserved for future use */
  uint16_t num_blocks;		/**< number of rectification info blocks in this file */
  uint32_t spec_head_size;	/**< data specific header size */
  uint64_t created_sec;		/**< creation unix timestamp, seconds */
  uint64_t created_usec;	/**< creation unix timestamp, useconds */
  char     comment[FVFF_COMMENT_SIZE];	/**< optional comment */
} fvff_header_t;


/** Block header.
 * Each block in a FvFF file has a block header. This header defines only the basic
 * characteristics that are needed to parse the file.
 * Directly following the header is the content specific block header. The size has to
 * be set in spec_head_size.
 */
typedef struct _fvff_block_header_t {
  uint32_t type;		/**< The type of the block, content-specific */
  uint32_t size;		/**< size in bytes of this block, does not include any headers */
  uint32_t spec_head_size;	/**< the size of the following content specific block header */
} fvff_block_header_t;

} // end namespace firevision


#pragma pack(pop)
#endif
