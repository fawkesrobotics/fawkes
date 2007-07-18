
/***************************************************************************
 *  jpeg_writer.h - JPEG writer
 *
 *  Generated: Wed Jun 28 11:33:50 2006 (my brother's 18th birthday)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_UTILS_WRITERS_JPEG_H_
#define __FIREVISION_UTILS_WRITERS_JPEG_H_

#include <fvutils/writers/writer.h>

#include <cstdio>

class JpegWriter : public Writer {

 public:
  JpegWriter(int quality = 80);
  JpegWriter(const char *filename, int quality = 80);
  virtual ~JpegWriter();

  virtual void             set_buffer(colorspace_t cspace, unsigned char *buffer);
  virtual void             write();
  
 private:
  unsigned char *row_buffer;
  int            quality;

  FILE *outfile;
};


#endif
