
/***************************************************************************
 *  relvelo.h - A simple velocity model using the relative coordinates and
 *              robot velocity
 *
 *  Created: Tue Oct 04 15:49:23 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_MODELS_VELOCITY_GLOBALFROMRELATIVE_H_
#define __FIREVISION_MODELS_VELOCITY_GLOBALFROMRELATIVE_H_

#include <fvmodels/velocity/velocitymodel.h>
#include <fvmodels/velocity/relvelo.h>
#include <fvmodels/relative_position/relativepositionmodel.h>

// include <utils/kalman_filter/ckalman_filter_2dim.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class VelocityGlobalFromRelative : public VelocityModel
{
 public:
  VelocityGlobalFromRelative(VelocityModel* rel_velo_model, RelativePositionModel *rel_pos_model);
  virtual ~VelocityGlobalFromRelative();

  virtual const char * getName() const;
  virtual coordsys_type_t getCoordinateSystem();

  virtual void	setRobotPosition(float x, float y, float ori, timeval t);
  virtual void  setRobotVelocity(float vel_x, float vel_y, timeval t);
  virtual void  setPanTilt(float pan, float tilt);
  virtual void  setTime(timeval t);
  virtual void  setTimeNow();
  virtual void  getTime(long int *sec, long int *usec);

  virtual void  getVelocity(float *vel_x, float *vel_y);

  virtual float getVelocityX();
  virtual float getVelocityY();

  virtual void  calc();
  virtual void  reset();

 private:
  VelocityModel         *relative_velocity;
  RelativePositionModel *relative_position;

  float                  robot_ori;
  float                  robot_poseage;

  float                  rel_vel_x;
  float                  rel_vel_y;
  float                  rel_dist;
  float                  cos_ori;
  float                  sin_ori;

  float                  velocity_x;
  float                  velocity_y;

  float                  avg_vx_sum;
  float                  avg_vy_sum;
  unsigned int           avg_vx_num;
  unsigned int           avg_vy_num;

  /*
  kalmanFilter2Dim      *kalman_filter;

  void                  applyKalmanFilter();
  */
};

} // end namespace firevision

#endif
