
/***************************************************************************
 *  globfromrel.h - A simple implementation of the global position model for
 *                 a ball
 *
 *  Created: Fri Jun 03 22:56:22 2005
 *  Copyright  2005  Hu Yuxiao <Yuxiao.Hu@rwth-aachen.de>
 *                   Tim Niemueller [www.niemueller.de]
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
                                                                                
#ifndef __FIREVISION_MODELS_GLOBAL_POSITION_GLOBFROMREL_H_
#define __FIREVISION_MODELS_GLOBAL_POSITION_GLOBFROMREL_H_

#include <fvmodels/global_position/globalpositionmodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RelativePositionModel;

class GlobalFromRelativePos : public GlobalPositionModel
{
 public:
  GlobalFromRelativePos(RelativePositionModel* model);
  virtual void	set_robot_position(float x, float y, float ori);
  virtual void  set_position_in_image(unsigned int x, unsigned int y);
  virtual float	get_x(void) const;
  virtual float	get_y(void) const;

  virtual void  calc();

  virtual bool  is_pos_valid() const;

 private:
  RelativePositionModel *m_pRelaModel;
  float	                 m_fPosX;
  float			 m_fPosY;
  float			 m_fPhi;

};

} // end namespace firevision

#endif
