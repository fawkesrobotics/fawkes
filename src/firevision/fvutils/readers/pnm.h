
/***************************************************************************
 *  pnm.h - PNM reader
 *
 *  Generated: Sun Jan 13 16:21:55 2008
 *  Copyright  2007  Daniel Beck
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

#ifndef __FIREVISION_UTILS_READERS_BITMAP_H_
#define __FIREVISION_UTILS_READERS_BITMAP_H_

#include "fvutils/readers/reader.h"

#include <cstdio>

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
  bool m_opened;
  unsigned char* m_yuv_buffer;
  unsigned char* m_pnm_buffer;

  char* m_filename;

  unsigned int m_img_width;
  unsigned int m_img_height;
  unsigned int m_img_depth;
};

#endif
