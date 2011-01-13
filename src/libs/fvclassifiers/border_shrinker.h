
/***************************************************************************
 *  border_shrinker.h - Header for BorderShrinker
 *
 *  Created: Wed Feb 15 15:00:26 2006
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

#ifndef __FIREVISION_CLASSIFIER_BORDER_SHRINKER_H_
#define __FIREVISION_CLASSIFIER_BORDER_SHRINKER_H_

#include <fvclassifiers/shrinker.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ROI;

class BorderShrinker : public Shrinker
{
 public:

  /** CloseShrinker shrinks ROIs
   * It will make sure that any ROI that passes it will NOT be in the border regions
   * denoted by the pixels
   * @param border_left left pixel margin
   * @param border_right right pixel margin
   * @param border_top top pixel margin
   * @param border_bottom botto pixel margin
   */
  BorderShrinker(unsigned int border_left = 0, unsigned int border_right = 0,
		 unsigned int border_top = 0, unsigned int border_bottom = 10);

  virtual ~BorderShrinker();

  virtual void shrink( ROI *roi );

 private:
  unsigned int border_left;
  unsigned int border_right;
  unsigned int border_top;
  unsigned int border_bottom;

};

} // end namespace firevision

#endif
