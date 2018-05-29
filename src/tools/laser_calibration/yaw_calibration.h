/***************************************************************************
 *  yaw_calibration.h - Calibrate yaw transform of the back laser
 *
 *  Created: Tue 18 Jul 2017 16:58:12 CEST 16:58
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef YAW_CALIBRATION_H
#define YAW_CALIBRATION_H

#include "laser_calibration.h"

#include <random>

class YawCalibration : public LaserCalibration
{
public:
  YawCalibration(LaserInterface *laser,
      LaserInterface *front_laser,
      fawkes::tf::Transformer *tf_transformer,
      fawkes::NetworkConfiguration *config,
      std::string config_path);
  virtual void calibrate();
protected:
  float get_current_cost(float *new_yaw);
  float get_new_yaw(float current_cost, float last_yaw);

protected:
  /** The laser interface used to read the front laser data from */
  LaserInterface *front_laser_;
  /** The initial step size */
  const float init_step_ = 0.02;
  /** The current step size */
  float step_;
  /** Random number generator used to compute the random reset probability */
  std::mt19937 random_generator_;
  /** The distribution used to compute the random reset probability */
  std::uniform_real_distribution<float> random_float_dist_;
  /** A map of yaw config values to costs */
  std::map<float, float> costs_;
  /** The minimal cost */
  float min_cost_;
  /** A yaw configuration with the minimal cost */
  float min_cost_yaw_;
};

#endif /* !YAW_CALIBRATION_H */
