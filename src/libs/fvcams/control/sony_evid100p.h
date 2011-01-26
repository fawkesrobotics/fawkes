
/***************************************************************************
 *  sony_evid100p_control.h - Controller for Sony EVI-D100P
 *
 *  Created: Tue Jun 07 15:52:46 2005
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

#ifndef __FIREVISION_CONTROL_SONYEVID100P_H_
#define __FIREVISION_CONTROL_SONYEVID100P_H_

#include <fvcams/control/pantilt.h>
#include <fvcams/control/zoom.h>
#include <fvcams/control/effect.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;
class ViscaControl;

class SonyEviD100PControl
: public CameraControlPanTilt,
  public CameraControlZoom,
  public CameraControlEffect
{

 public:

  static const unsigned int EFFECT_PASTEL;
  static const unsigned int EFFECT_NEGATIVE;
  static const unsigned int EFFECT_SEPIA;
  static const unsigned int EFFECT_BW;
  static const unsigned int EFFECT_SOLARIZE;
  static const unsigned int EFFECT_MOSAIC;
  static const unsigned int EFFECT_SLIM;
  static const unsigned int EFFECT_STRETCH;

  SonyEviD100PControl(const CameraArgumentParser *cap);
  SonyEviD100PControl(const char *tty_port);
  virtual ~SonyEviD100PControl();

  void open();
  void close();
  void process_pantilt();

  // pan/tilt
  bool         supports_pan();
  bool         supports_tilt();
  void         set_pan(int pan);
  void         set_tilt(int tilt);
  void         set_pan_tilt(int pan, int tilt);
  void         set_pan_tilt_rad(float pan, float tilt);
  int          pan();
  int          tilt();
  void         start_get_pan_tilt();
  void         pan_tilt(int &pan, int &tilt);
  void         pan_tilt_rad(float &pan, float &tilt);
  int          min_pan();
  int          max_pan();
  int          min_tilt();
  int          max_tilt();
  void         reset_pan_tilt();
  void         set_pan_tilt_limit(int pan_left, int pan_right,
				  int tilt_up, int tilt_down);
  void         reset_pan_tilt_limit();

  // zoom
  void         reset_zoom();
  void         set_zoom(unsigned int zoom);
  unsigned int zoom();
  unsigned int zoom_max();
  unsigned int zoom_min();
  void         set_zoom_speed_tele(unsigned int speed);
  void         set_zoom_speed_wide(unsigned int speed);
  void         set_zoom_digital_enabled(bool enabled);

  unsigned int white_balance_mode();

  // effect
  bool         supports_effect(unsigned int effect);
  void         set_effect(unsigned int effect);
  unsigned int effect();
  void         reset_effect();


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

 private:
  ViscaControl *visca;
  char  *tty_port;
  bool   opened;

  int    pan_target;
  int    tilt_target;

  unsigned int _effect;

};

} // end namespace firevision

#endif
