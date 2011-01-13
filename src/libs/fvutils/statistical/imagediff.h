
/***************************************************************************
 *  imagediff.h - check if two images differ, allow probabilistic scanning
 *                by using scanlines.
 *
 *  Generated: Tue Jun 06 10:08:36 2006
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

#ifndef __FIREVISION_FVUTILS_STATISTICAL_IMAGEDIFF_H_
#define __FIREVISION_FVUTILS_STATISTICAL_IMAGEDIFF_H_

#include <fvmodels/scanlines/scanlinemodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ImageDiff {
 public:
  ImageDiff(ScanlineModel *scanline_model);
  ImageDiff();
  ~ImageDiff();

  void setBufferA(unsigned char *yuv422planar_buffer,
		  unsigned int width, unsigned int height);

  void setBufferB(unsigned char *yuv422planar_buffer,
		  unsigned int width, unsigned int height);

  bool different();
  unsigned int numDifferingPixels();

 private:
  ScanlineModel *scanline_model;

  unsigned char *buffer_a;
  unsigned char *buffer_b;

  unsigned int width_a;
  unsigned int height_a;
  unsigned int width_b;
  unsigned int height_b;

};

} // end namespace firevision

#endif
