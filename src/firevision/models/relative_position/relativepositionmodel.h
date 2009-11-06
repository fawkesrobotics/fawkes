
/***************************************************************************
 *  relativepositionmodel.h - Abstract class defining a position model for
 *                            calculation of relative position
 *
 *  Created: Tue May 31 13:50:12 2005
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_RELATIVEPOSITIONMODEL_H_
#define __FIREVISION_RELATIVEPOSITIONMODEL_H_

#include <fvutils/base/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RelativePositionModel
{

 public:
  virtual ~RelativePositionModel();

  virtual const char *  get_name(void) const		            = 0;
  virtual void		set_radius(float r)		            = 0;
  virtual void		set_center(float x, float y)	            = 0;
  virtual void		set_center(const center_in_roi_t& c)        = 0;

  virtual void		set_pan_tilt(float pan, float tilt)         = 0;
  virtual void		get_pan_tilt(float *pan, float *tilt) const = 0;

  virtual void          set_cam_rotation(float pan, float tilt, float roll = 0.f);
  virtual void          get_cam_rotation(float &pan, float &tilt, float &roll) const;

  virtual void          set_cam_translation(float height, float rel_x = 0.f, float rel_y = 0.f);
  virtual void          get_cam_translation(float &height, float &rel_x, float &rel_y) const;

  virtual void          calc()                                      = 0;
  virtual void          calc_unfiltered()                           = 0;

  virtual void          reset()                                     = 0;

  virtual float		get_distance() const		            = 0;
  virtual float		get_bearing() const	  	            = 0;
  virtual float		get_slope() const                           = 0;
  virtual float		get_x() const			            = 0;
  virtual float		get_y() const			            = 0;

  virtual bool          is_pos_valid() const                        = 0;

};

} // end namespace firevision

#endif
