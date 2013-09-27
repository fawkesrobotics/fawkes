
/***************************************************************************
 *  mirrormodel.h - Abstract class defining a mirror model
 *
 *  Created: Tue Jul 19 11:55:29 2005
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

#ifndef __FIREVISION_MODELS_MIRRORMODEL_H_
#define __FIREVISION_MODELS_MIRRORMODEL_H_

#include <utils/math/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MirrorModel
{
 public:

  virtual ~MirrorModel();

  virtual void warp2unwarp(unsigned int warp_x, unsigned int warp_y,
                           unsigned int *unwarp_x, unsigned int *unwarp_y)     = 0;

  virtual void unwarp2warp(unsigned int unwarp_x, unsigned int unwarp_y,
                           unsigned int *warp_x, unsigned int *warp_y)         = 0;

  virtual const char * getName()                                               = 0;


  virtual fawkes::polar_coord_2d_t getWorldPointRelative(unsigned int image_x,
                                                         unsigned int image_y) const = 0;

  virtual fawkes::cart_coord_2d_t getWorldPointGlobal(unsigned int image_x,
                                                      unsigned int image_y,
                                                      float pose_x, float pose_y,
                                                      float pose_ori) const    = 0;

  virtual void reset()                                                         = 0;

  virtual fawkes::upoint_t getCenter() const                                    = 0;
  virtual void setCenter(unsigned int image_x, unsigned int image_y  )         = 0;
  virtual void setOrientation(float angle)                                     = 0;
  virtual float getOrientation() const                                         = 0;

  virtual bool isValidPoint(unsigned int image_x, unsigned int image_y ) const = 0;

};

} // end namespace firevision

#endif
