
/***************************************************************************
 *  omni_global.h - Global position model that operating on a MirrorModel
 *
 *  Created: Thu Mar 23 22:30:28 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_MODELS_GLOBAL_POSITION_OMNI_GLOBAL_H_
#define __FIREVISION_MODELS_GLOBAL_POSITION_OMNI_GLOBAL_H_

#include <fvmodels/global_position/globalpositionmodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MirrorModel;

class OmniGlobal : public GlobalPositionModel
{
 public:
  // constructor
  OmniGlobal(MirrorModel *mirror_model);

  virtual void		set_robot_position(float x, float y, float ori);
  virtual void          set_position_in_image(unsigned int x, unsigned int y);

  virtual float		get_x() const;
  virtual float		get_y() const;

  virtual void          calc();

  virtual bool          is_pos_valid() const;

private:
  float	                pose_x;
  float			pose_y;
  float			pose_ori;

  float                 ball_x;
  float                 ball_y;

  MirrorModel          *mirror_model;

  unsigned int          image_x;
  unsigned int          image_y;

};

} // end namespace firevision

#endif
