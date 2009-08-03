
/***************************************************************************
 *  evid100p.h - Sony EviD100P Visca wrapper
 *
 *  Created: Sun Jun 21 13:10:51 2009
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PANTILT_SONY_EVID100P_H_
#define __PLUGINS_PANTILT_SONY_EVID100P_H_

#include "visca.h"

#define SONY_EVID100P_NUM_PAN_SPEEDS 24
#define SONY_EVID100P_NUM_TILT_SPEEDS 20

class SonyEviD100PVisca : public Visca {

 public:
  SonyEviD100PVisca(const char *device_file, unsigned int def_timeout_ms = 30,
		   bool blocking = true);
  ~SonyEviD100PVisca();

  void  get_pan_tilt_rad(float &pan, float &tilt);
  void  set_pan_tilt_rad(float pan, float tilt);

  void set_speed_radsec(float pan_speed, float tilt_speed);
  void get_speed_radsec(float &pan_speed, float &tilt_speed);

  void get_speed_limits(float &pan_min, float &pan_max,
			float &tilt_min, float &tilt_max);

  static const int   MAX_PAN;
  static const int   MIN_PAN;
  static const int   MAX_TILT;
  static const int   MIN_TILT;

  static const float MAX_PAN_DEG;
  static const float MIN_PAN_DEG;
  static const float MAX_TILT_DEG;
  static const float MIN_TILT_DEG;

  static const float MAX_PAN_RAD;
  static const float MIN_PAN_RAD;
  static const float MAX_TILT_RAD;
  static const float MIN_TILT_RAD;

  static const float PAN_STEPS_PER_DEG;
  static const float TILT_STEPS_PER_DEG;

  static const float PAN_STEPS_PER_RAD;
  static const float TILT_STEPS_PER_RAD;

  static const unsigned int EFFECT_PASTEL;
  static const unsigned int EFFECT_NEGATIVE;
  static const unsigned int EFFECT_SEPIA;
  static const unsigned int EFFECT_BW;
  static const unsigned int EFFECT_SOLARIZE;
  static const unsigned int EFFECT_MOSAIC;
  static const unsigned int EFFECT_SLIM;
  static const unsigned int EFFECT_STRETCH;

  static const float        SPEED_TABLE_PAN[SONY_EVID100P_NUM_PAN_SPEEDS];
  static const float        SPEED_TABLE_TILT[SONY_EVID100P_NUM_TILT_SPEEDS];
};



#endif
