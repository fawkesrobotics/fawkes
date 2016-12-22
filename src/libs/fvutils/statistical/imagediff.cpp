
/***************************************************************************
 *  imagediff.cpp - check images if they are different
 *
 *  Generated: Tue Jun 06 10:22:49 2006
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

#include <fvutils/statistical/imagediff.h>
#include <fvutils/color/yuv.h>

#include <cstdlib>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ImageDiff <fvutils/statistical/imagediff.h>
 * Image difference checker.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param scanline_model scanlinemodel to use, if null all pixels
 * are compared.
 */
ImageDiff::ImageDiff(ScanlineModel *scanline_model)
{
  this->scanline_model = scanline_model;
}


/** Constructor.
 * Use this constructor to compare all pixels.
 */
ImageDiff::ImageDiff()
{
  scanline_model = NULL;
}


/** Destructor. */
ImageDiff::~ImageDiff()
{
}


/** Set first buffer.
 * @param yuv422planar_buffer buffer
 * @param width image width in pixels
 * @param height image height in pixels
 */
void
ImageDiff::setBufferA(unsigned char *yuv422planar_buffer,
		      unsigned int width, unsigned int height)
{
  buffer_a = yuv422planar_buffer;
  width_a  = width;
  height_a = height;
}


/** Set second buffer.
 * @param yuv422planar_buffer buffer
 * @param width image width in pixels
 * @param height image height in pixels
 */
void
ImageDiff::setBufferB(unsigned char *yuv422planar_buffer,
		      unsigned int width, unsigned int height)
{
  buffer_b = yuv422planar_buffer;
  width_b  = width;
  height_b = height;
}


/** Check if images are different.
 * This method will compare the two images. If any pixel marked by
 * the scanline or any pixel at all if no scanline model is given
 * differ the images are considered to be different. The same applies
 * if any buffer is unset or the widths or heights are not the same.
 * @return true if images are different, false otherwise
 */
bool
ImageDiff::different()
{
  if ( (buffer_a == NULL) && (buffer_b == NULL) ) return false;
  if ( (buffer_a == NULL) && (buffer_b != NULL) ) return true;
  if ( (buffer_a != NULL) && (buffer_b == NULL) ) return true;
  if ( (width_a != width_b) || (height_a != height_b) ) return true;

  if ( scanline_model != NULL ) {
    // use the supplied scanline model

    unsigned int x, y;
    unsigned char y_a, u_a, v_a, y_b, u_b, v_b;

    scanline_model->reset();
    while (! scanline_model->finished() ) {
      x = (*scanline_model)->x;
      y = (*scanline_model)->y;
      
      YUV422_PLANAR_YUV(buffer_a, width_a, height_a, x, y, y_a, u_a, v_a);
      YUV422_PLANAR_YUV(buffer_b, width_b, height_b, x, y, y_b, u_b, v_b);
      
      if ( (y_a != y_b) || (u_a != u_b) || (v_a != v_b) ) {
	return true;
      }
    }
  } else {
    // no scanline model, check every single pixel

    unsigned char *ypa = buffer_a;
    unsigned char *ypb = buffer_b;

    for ( unsigned int i = 0; i < (width_a * height_a); ++i) {
      if ( *ypa != *ypb ) {
	return true;
      }
      ++ypa;
      ++ypb;
    }
  }

  return false;
}


/** Number of differing pixels.
 * Executes the same routine as different(). But instead of just saying that
 * the images are different will tell how many pixels differ.
 * @return number of different pixels
 */
unsigned int
ImageDiff::numDifferingPixels()
{
  if ( (buffer_a == NULL) && (buffer_b == NULL) ) return 0;
  if ( (buffer_a == NULL) && (buffer_b != NULL) ) return (width_b * height_b);
  if ( (buffer_a != NULL) && (buffer_b == NULL) ) return (width_a * height_a);
  if ( (width_a != width_b) || (height_a != height_b) ) {
    return std::abs((long)width_a - (long)width_b) * std::abs((long)height_a - (long)height_b);
  }

  unsigned int num = 0;
  if ( scanline_model != NULL ) {
    // use the supplied scanline model

    unsigned int x, y;
    unsigned char y_a, u_a, v_a, y_b, u_b, v_b;

    scanline_model->reset();
    while (! scanline_model->finished() ) {
      x = (*scanline_model)->x;
      y = (*scanline_model)->y;

      YUV422_PLANAR_YUV(buffer_a, width_a, height_a, x, y, y_a, u_a, v_a);
      YUV422_PLANAR_YUV(buffer_b, width_b, height_b, x, y, y_b, u_b, v_b);
      
      if ( (y_a != y_b) || (u_a != u_b) || (v_a != v_b) ) {
	++num;
      }
    }
  } else {
    // no scanline model, check every single pixel

    unsigned char *ypa = buffer_a;
    unsigned char *ypb = buffer_b;

    for ( unsigned int i = 0; i < (width_a * height_a); ++i) {
      if ( *ypa++ != *ypb++ ) ++num;
    }
  }
  return num;
}

} // end namespace firevision
