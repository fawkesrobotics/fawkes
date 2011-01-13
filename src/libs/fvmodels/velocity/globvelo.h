
/***************************************************************************
 *  globvelo.h - A simple velocity model using the global coordinates
 *
 *  Created: Mon Sep 05 17:06:54 2005
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

#ifndef __FIREVISION_MODELS_VELOCITY_GLOBAL_H_
#define __FIREVISION_MODELS_VELOCITY_GLOBAL_H_

#include <fvmodels/velocity/velocitymodel.h>
#include <fvmodels/global_position/globalpositionmodel.h>

#include <sys/time.h>
#include <vector>

// include <utils/kalman_filter/ckalman_filter_2dim.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class VelocityFromGlobal : public VelocityModel
{
 public:
  VelocityFromGlobal(GlobalPositionModel* model, unsigned int history_length, unsigned int calc_interval);
  virtual ~VelocityFromGlobal();

  virtual const char * getName() const;

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

  virtual coordsys_type_t getCoordinateSystem();

 private:
  GlobalPositionModel   *global_pos_model;

  float	                 robot_pos_x;
  float			 robot_pos_y;
  float                  robot_pos_ori;
  float                  robot_pos_age;

  timeval                now;
  std::vector<timeval>        last_time;

  unsigned int           history_length;
  unsigned int           calc_interval;

  int                    diff_sec;
  int                    diff_usec;

  float                  f_diff_sec;


  std::vector<float>	 last_x;
  std::vector<float>     last_y;

  float                  current_x;
  float                  current_y;

  float                  diff_x;
  float                  diff_y;

  float                  velocity_total_x;
  float                  velocity_total_y;
  float                  velocity_num;

  float                  velocity_x;
  float                  velocity_y;

  /*
  kalmanFilter2Dim      *kalman_filter;

  void                  applyKalmanFilter();
  */

};

} // end namespace firevision

#endif
