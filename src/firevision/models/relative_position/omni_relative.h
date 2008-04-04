
/***************************************************************************
 *  omni_ball_relative.h - A simple implementation of a relative omni
 *                         relative position model using a MirrorModel
 *
 *  Generated: Fri Jun 03 22:56:22 2005
 *  Copyright  2005  Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
 *                   Tim Niemueller [www.niemueller.de]
 *                   Martin Heracles <Martin.Heracles@rwth-aachen.de>
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FIREVISION_MODELS_RELATIVE_POSITION_OMNI_RELATIVE_H_
#define __FIREVISION_MODELS_RELATIVE_POSITION_OMNI_RELATIVE_H_

#include <models/relative_position/relativepositionmodel.h>
#include <models/mirror/mirrormodel.h>

// include <utils/kalman_filter/ckalman_filter_2dim.h>

class OmniRelative : public RelativePositionModel
{
 public:
  // constructor
  OmniRelative(MirrorModel *mirror_model);

  virtual const char *  getName() const;
  virtual void		setRadius(float r);
  virtual void		setCenter(float x, float y);
  virtual void		setCenter(const center_in_roi_t& c);

  virtual void		setPanTilt(float pan = 0.0f, float tilt = 0.0f);
  virtual void          getPanTilt(float *pan, float *tilt) const;

  /* Method "getDistance"
   *   returns projection of air-line-distance (on the ground)
   *   between ball center and motor of robot
   */
  virtual float		getDistance() const;

  /* Method "getX"
   *   returns x component of what method "getDistance" calculates
   */
  virtual float		getX() const;

  /* Method "getY"
   *   returns y component of what method "getDistance" calculates
   */
  virtual float		getY() const;

  virtual float		getBearing() const;
  virtual float		getSlope() const;
  virtual float         getRadius() const;

  virtual void          calc();
  virtual void          calc_unfiltered();
  virtual void          reset();

  virtual bool          isPosValid() const;

private:
  float                 DEFAULT_X_VARIANCE;
  float                 DEFAULT_Y_VARIANCE;

  MirrorModel          *mirror_model;

  unsigned int          image_width;
  unsigned int          image_height;

  unsigned int          image_x;
  unsigned int          image_y;

  float                 last_x;
  float                 last_y;
  bool                  last_available;
  float                 ball_x;
  float                 ball_y;
  float                 bearing;
  float                 slope;
  float                 distance_ball_motor;
  float                 distance_ball_cam;

  float                 avg_x;
  float                 avg_y;
  float                 avg_x_sum;
  float                 avg_y_sum;
  unsigned int          avg_x_num;
  unsigned int          avg_y_num;
  float                 rx;
  float                 ry;

  //kalmanFilter2Dim     *kalman_filter;

  //void                  applyKalmanFilter();
};

#endif

