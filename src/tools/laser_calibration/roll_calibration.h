/***************************************************************************
 *  roll_calibration.h - Calibrate roll transform of the back laser
 *
 *  Created: Tue 18 Jul 2017 16:28:05 CEST 16:28
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

#ifndef ROLL_CALIBRATION_H
#define ROLL_CALIBRATION_H

#include "laser_calibration.h"

class RollCalibration : public LaserCalibration
{
public:
  RollCalibration(LaserInterface *laser,
      fawkes::tf::Transformer *tf_transformer,
      fawkes::NetworkConfiguration *config, std::string config_path);

  virtual void calibrate();

protected:
  float get_lr_mean_diff();
  float get_new_roll(float mean_error, float old_roll);
  PointCloudPtr filter_calibration_cloud(PointCloudPtr input);

protected:
  // TODO: make threshold and min_points configurable
  /** The threshold of the left-right difference to stop calibration */
  constexpr static float threshold = 0.00001;
};

#endif /* !ROLL_CALIBRATION_H */
