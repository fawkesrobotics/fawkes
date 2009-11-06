
/***************************************************************************
 *  pnm.h - Header for tool to write PNM,
 *  for more information on the different available image formats see the
 *  NetPBM documentation.
 *
 *  Generated: Mon Feb 06 19:18:03 2006
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

#ifndef __FIREVISION_FVUTILS_WRITERS_PNM_H_
#define __FIREVISION_FVUTILS_WRITERS_PNM_H_


#include <fvutils/color/colorspaces.h>
#include <fvutils/writers/writer.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** PNM subtype. */
typedef enum {
  PNM_PBM,		/**< PBM, B/W */
  PNM_PBM_ASCII,	/**< PBM, B/W, ASCII */
  PNM_PGM,		/**< PGM, grey */
  PNM_PGM_ASCII,	/**< PGM, grey, ASCII */
  PNM_PPM,		/**< PPM, color */
  PNM_PPM_ASCII		/**< PPM, color, ASCII */
} PNMFormat;


class PNMWriter : public Writer
{
 public:
  PNMWriter(PNMFormat format);
  PNMWriter(PNMFormat format, const char *filename, unsigned int width, unsigned int height);

  virtual void set_buffer(colorspace_t cspace, unsigned char *buffer);
  virtual void write();

 private:

  unsigned int calc_buffer_size();

  unsigned int write_header(bool simulate = false);
  const char * format2string(PNMFormat format);

  PNMFormat      format;
  unsigned int   buffer_size;
  unsigned char *buffer;
  unsigned char *buffer_start;
  unsigned int   width;
  unsigned int   height;
};

} // end namespace firevision

#endif
