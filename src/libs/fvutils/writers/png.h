
/***************************************************************************
 *  png.h - Header for tool to write PNGs
 *
 *  Generated: Thu Jun 02 15:21:58 2005
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

#ifndef __FIREVISION_FVUTILS_WRITERS_PNG_H_
#define __FIREVISION_FVUTILS_WRITERS_PNG_H_

#include <fvutils/writers/writer.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class PNGWriter : public Writer
{
 private:
  colorspace_t colorspace_;
 public:
  PNGWriter();
  PNGWriter(const char *filename, unsigned int width, unsigned int height);
  ~PNGWriter();

  virtual void set_buffer(colorspace_t cspace, unsigned char *buffer);
  virtual void write();
};

} // end namespace firevision

#endif
