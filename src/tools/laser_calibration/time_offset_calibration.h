/***************************************************************************
 *  time_offset_calibration.h - Laser time offset calibration
 *
 *  Created: Tue 18 Jul 2017 17:21:50 CEST 17:21
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

#ifndef TIME_OFFSET_CALIBRATION_H
#define TIME_OFFSET_CALIBRATION_H

#include "laser_calibration.h"

class TimeOffsetCalibration : public LaserCalibration
{
public:
  TimeOffsetCalibration(LaserInterface *laser, fawkes::MotorInterface *motor,
      fawkes::tf::Transformer *tf_transformer,
      fawkes::NetworkConfiguration *config, std::string config_path);

  virtual void calibrate();

protected:
  PointCloudPtr get_lasercloud(LaserInterface *laser);

protected:
  /** Time in micro seconds to sleep after each iteration */
  const static long sleep_time_ = 2000000;
  /** The motor interface used to control the rotation of the robot */
  fawkes::MotorInterface *motor_;
  /** The angular velocity to use to rotate */
  constexpr static float omega_ = 2.0;
  /** The frequency for motor commands */
  const static unsigned int frequency_ = 100;
  /** The time to rotate */
  constexpr static float rotation_time_ = 1.;
  /** The current step size for the time offset */
  float step_;
};

#endif /* !TIME_OFFSET_CALIBRATION_H */
