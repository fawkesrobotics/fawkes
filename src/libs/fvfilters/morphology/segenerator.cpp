
/***************************************************************************
 *  segenerator.cpp - Class that helps to create some standard structuring
 *                    elements
 *
 *  Created: Wed Jun 07 11:23:03 2006
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

#include <fvfilters/morphology/segenerator.h>

#include <utils/math/angle.h>

#include <fvutils/draw/drawer.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/writers/png.h>
#include <fvutils/writers/fvraw.h>

#include <cstdlib>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SEGenerator <fvfilters/morphology/segenerator.h>
 * Basic generators for structuring elements for morphological filters.
 * @author Tim Niemueller
 */


/** Generate linear structuring element.
 * @param width width of structuring element
 * @param height height of structuring element
 * @param proposed_center_x contains the proposed x coordinate of the anchor
 * upon return
 * @param proposed_center_y contains the proposed y coordinate of the anchor
 * upon return
 * @param slope_angle_rad the slope of the line in radians
 * @return buffer with linear structuring element. The buffer has been allocated
 * using malloc(). Use free() to free the memory after you are done with it.
 */
unsigned char *
SEGenerator::linear(unsigned int width, unsigned int height,
		    unsigned int *proposed_center_x, unsigned int *proposed_center_y,
		    float slope_angle_rad)
{

  // we always start at (0,0) and then calculate the corrensponding
  // y of the linear functional
  // l: x -> mx + b, where b is 0.
  // by tan(slope_angle_rad) = y / x
  // => y = x * tan(slope_angle_rad)    with x = width

  if ( height == 0 ) return NULL;
  if ( width == 0) return NULL;

  
  unsigned char *tmp = (unsigned char *)malloc(colorspace_buffer_size(YUV422_PLANAR, width, height));
  memset(tmp, 0, colorspace_buffer_size(YUV422_PLANAR, width, height));
  Drawer *d = new Drawer();
  d->set_buffer(tmp, width, height);
  d->set_color(1, 0, 0);

  float a = fawkes::normalize_mirror_rad( slope_angle_rad );

  if ( (a == M_PI/2) || (a == -M_PI/2) ) {
    // It's just a vertical line
    // std::cout << "Drawing line from (0,0) -> (0," << height - 1 << ")" << std::endl;
    d->draw_line(0, 0, 0, height - 1);
  } else {

    // sectors 3 and 4 can be converted to sector 2 and 1 lines
    if ( a > M_PI / 2)    a -= M_PI;
    if ( a < - M_PI / 2)  a += M_PI;

    int y = (int)roundf(((float)width - 1.f) * tan( a ));

    if ( y < 0) {
      // std::cout << "Drawing line from (0,0) -> (" << width - 1 << "," << -y << ")" << std::endl;
      d->draw_line( 0, 0, width - 1, -y );
    } else {
      // std::cout << "Drawing line from (0," << y << ") -> (" << width - 1 << ",0)" << std::endl;
      d->draw_line( 0, y, width - 1, 0 );
    }
  }

  delete d;

  unsigned char *se = (unsigned char *)malloc(width * height);
  memcpy(se, tmp, width * height);

  PNGWriter *png = new PNGWriter();
  png->set_dimensions( width, height );
  png->set_buffer(YUV422_PLANAR, tmp);
  png->set_filename("se_test.png");
  png->write();
  delete png;

  FvRawWriter *fvraw = new FvRawWriter("se_test.raw", width, height, YUV422_PLANAR, tmp);
  fvraw->write();
  delete fvraw;

  free( tmp );

  if ( (proposed_center_x != NULL) && (proposed_center_y != NULL) ) {
    unsigned int min_x = width;
    unsigned int max_x = 0;
    unsigned int min_y = height;
    unsigned int max_y = 0;
    for (unsigned int h = 0; h < height; ++h) {
      for (unsigned int w = 0; w < width; ++w) {
	if ( se[ h * width + w ] != 0 ) {
	  if ( w < min_x ) min_x = w;
	  if ( w > max_x ) max_x = w;
	  if ( h < min_y ) min_y = h;
	  if ( h > max_y ) max_y = h;
	}
      }
    }

    *proposed_center_x = min_x + (max_x - min_x) / 2;
    *proposed_center_y = min_y + (max_y - min_y) / 2;
  }

  return se;
}


/** Generate square structuring element.
 * @param width width of structuring element
 * @param height height of structuring element
 * @return buffer with square structuring element. The buffer has been allocated
 * using malloc(). Use free() to free the memory after you are done with it.
 */
unsigned char *
SEGenerator::square(unsigned int width, unsigned int height)
{
  unsigned char *se = (unsigned char *)malloc(width * height);
  memset(se, 1, width * height);
  return se;
}


/** Draw structuring element.
 * This draws the structuring element to an image buffer.
 * @param yuv422planar_buffer image buffer
 * @param mask structuring element
 * @param width width of structuring element
 * @param height height of structuring element
 */
void
SEGenerator::drawSE(unsigned char *yuv422planar_buffer, unsigned char *mask, unsigned int width, unsigned int height)
{
  memset(yuv422planar_buffer, 128, colorspace_buffer_size(YUV422_PLANAR, width, height) );
  for (unsigned int h = 0; h < height; ++h) {
    for (unsigned int w = 0; w < width; ++w) {
      if ( mask[ h * width + w ] != 0 ) {
	yuv422planar_buffer[ h * width + w ] = 255;
      }
    }
  }
}


/** Draw structuring element.
 * This draws the structuring element to a b/w image buffer.
 * @param yuv422planar_buffer image buffer
 * @param mask structuring element
 * @param width width of structuring element
 * @param height height of structuring element
 */
void
SEGenerator::drawSEbw(unsigned char *yuv422planar_buffer, unsigned char *mask, unsigned int width, unsigned int height)
{
  memset(yuv422planar_buffer, 128, colorspace_buffer_size(YUV422_PLANAR, width, height) );
  memset(yuv422planar_buffer, 255, width * height );
  for (unsigned int h = 0; h < height; ++h) {
    for (unsigned int w = 0; w < width; ++w) {
      if ( mask[ h * width + w ] != 0 ) {
	yuv422planar_buffer[ h * width + w ] = 0;
      }
    }
  }
}

} // end namespace firevision
