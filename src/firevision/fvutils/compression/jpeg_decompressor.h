
/***************************************************************************
 *  imagedecompressor.h - image de-compressor interface
 *
 *  Created: July 2007 (Sci-Bono, South Africa, B&B)
 *  Copyright  2006-2007  Daniel Beck
 *             2007       Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_UTILS_COMPRESSION_JPEG_DECOMPRESSOR_H_
#define __FIREVISION_UTILS_COMPRESSION_JPEG_DECOMPRESSOR_H_

#include <fvutils/compression/imagedecompressor.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class JpegImageDecompressor : public ImageDecompressor
{
 public:
  JpegImageDecompressor();

  virtual void decompress();
};

} // end namespace firevision

#endif
