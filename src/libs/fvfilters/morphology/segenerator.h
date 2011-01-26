
/***************************************************************************
 *  segenerator.h - Class that helps to create some standard structuring
 *                  elements
 *
 *  Created: Wed Jun 07 10:36:55 2006
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

#ifndef __FIREVISION_FILTERS_MORPHOLOGY_SEGENERATOR_H_
#define __FIREVISION_FILTERS_MORPHOLOGY_SEGENERATOR_H_

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SEGenerator {
 private:
  SEGenerator();


 public:
  
  static unsigned char * linear(unsigned int width, unsigned int height,
				unsigned int *proposed_center_x,
				unsigned int *proposed_center_y,
				float slope_angle_rad);

  static unsigned char * square(unsigned int width, unsigned int height);


  static void drawSE(unsigned char *yuv422planar_buffer, unsigned char *mask,
		     unsigned int width, unsigned int height);
  static void drawSEbw(unsigned char *yuv422planar_buffer, unsigned char *mask,
		       unsigned int width, unsigned int height);

};

} // end namespace firevision

#endif
