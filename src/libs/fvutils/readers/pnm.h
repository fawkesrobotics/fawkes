
/***************************************************************************
 *  pnm.h - PNM reader
 *
 *  Generated: Sun Jan 13 16:21:55 2008
 *  Copyright  2007  Daniel Beck
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

#ifndef __FIREVISION_UTILS_READERS_BITMAP_H_
#define __FIREVISION_UTILS_READERS_BITMAP_H_

#include <fvutils/readers/reader.h>

#include <cstdio>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class PNMReader : public Reader {

 public:
  PNMReader(const char* filename);
  virtual ~PNMReader();

  virtual void             set_buffer(unsigned char* yuv422planar_buffer);
  virtual colorspace_t     colorspace();
  virtual unsigned int     pixel_width();
  virtual unsigned int     pixel_height();
  virtual void             read();

 private:
  FILE* m_pnmfile;
  unsigned char* m_yuv_buffer;
  unsigned char* m_pnm_buffer;

  char* m_filename;

  unsigned int m_img_width;
  unsigned int m_img_height;
  unsigned int m_img_depth;
};

} // end namespace firevision

#endif
