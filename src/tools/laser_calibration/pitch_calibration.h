/***************************************************************************
 *  pitch_calibration.h - Calibrate pitch transform of the back laser
 *
 *  Created: Tue 18 Jul 2017 16:47:17 CEST 16:47
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

#ifndef PITCH_CALIBRATION_H
#define PITCH_CALIBRATION_H

#include "laser_calibration.h"

class PitchCalibration : public LaserCalibration
{
public:
  PitchCalibration(LaserInterface *laser,
      fawkes::tf::Transformer *tf_transformer,
      fawkes::NetworkConfiguration *config, std::string config_path);
  virtual void calibrate();
protected:
  float get_new_pitch(float z, float old_pitch);
protected:
  /** The threshold of the mean of z to stop calibration */
  constexpr static float threshold = 0.001;
};


#endif /* !PITCH_CALIBRATION_H */
