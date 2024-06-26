
/***************************************************************************
 *  jpeg.h - JPEG Reader
 *
 *  Generated: Sun Jun 04 23:18:06 2006 (watching Terminator 2)
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

#ifndef _FIREVISION_UTILS_READERS_JPEG_H_
#define _FIREVISION_UTILS_READERS_JPEG_H_

#include <fvutils/readers/reader.h>

#include <cstdio>
extern "C" {
#include <jpeglib.h>
}

namespace firevision {

class JpegReader : public Reader
{
public:
	JpegReader(const char *filename);
	virtual ~JpegReader();

	virtual void         set_buffer(unsigned char *yuv422planar_buffer);
	virtual colorspace_t colorspace();
	virtual unsigned int pixel_width();
	virtual unsigned int pixel_height();
	virtual void         read();

private:
	bool opened;

	unsigned char *buffer;
	unsigned char *row_buffer;

	FILE                         *infile;
	int                           row_stride;
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr         jerr;
};

} // end namespace firevision

#endif
