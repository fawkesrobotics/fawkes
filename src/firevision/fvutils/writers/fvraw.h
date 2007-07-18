
/***************************************************************************
 *  fvraw.h - writer for FireVision raw files
 *
 *  Generated: Sat Mar 25 00:05:26 2006
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

#ifndef __FIREVISION_FVUTILS_WRITERS_FVRAW_H_
#define __FIREVISION_FVUTILS_WRITERS_FVRAW_H_

#include <fvutils/writers/writer.h>

class FvRawWriter : public Writer {

 public:
  FvRawWriter();
  FvRawWriter(const char *filename,
	      unsigned int width, unsigned int height);
  FvRawWriter(const char *filename,
	      unsigned int width, unsigned int height,
	      colorspace_t colorspace, unsigned char *buffer);
  virtual ~FvRawWriter();

  virtual void             set_dimensions(unsigned int width, unsigned int height);
  virtual void             set_buffer(colorspace_t cspace, unsigned char *buffer);
  virtual void             write();
  virtual unsigned char *  get_write_buffer();

  static const unsigned int FILE_IDENTIFIER;

  /** FvRaw image file header. */
  typedef struct {
    unsigned int file_id;	/**< file id */
    colorspace_t colorspace;	/**< color space */
    unsigned int width;		/**< width of image in pixels */
    unsigned int height;	/**< height of image in pixels */
  } FvRawHeader;

 private:
  FvRawHeader header;
  unsigned char *buffer;

};



#endif
