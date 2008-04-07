
/***************************************************************************
 *  cameracontrol.h - Abstract class defining a camera controller
 *
 *  Generated: Tue Jun 07 15:45:57 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_CAMERACONTROL_H_
#define __FIREVISION_CAMERACONTROL_H_

class CameraControl
{
 public:

  virtual ~CameraControl();

  static const unsigned int EFFECT_NONE;

  virtual void         process_control();

  virtual bool         supports_focus();
  virtual bool         auto_focus();
  virtual void         set_auto_focus(bool enabled);
  virtual unsigned int focus();
  virtual void         set_focus(unsigned int focus);
  virtual unsigned int focus_min();
  virtual unsigned int focus_max();

  virtual bool         supports_pan();
  virtual bool         supports_tilt();
  virtual void         set_pan(int pan);
  virtual void         set_tilt(int tilt);
  virtual void         set_pan_tilt(int pan, int tilt);
  virtual void         set_pan_tilt_rad(float pan, float tilt);
  virtual int          pan();
  virtual int          tilt();
  virtual void         start_get_pan_tilt();
  virtual void         pan_tilt(int *pan, int *tilt);
  virtual void         pan_tilt_rad(float *pan, float *tilt);
  virtual int          min_pan();
  virtual int          max_pan();
  virtual int          min_tilt();
  virtual int          max_tilt();
  virtual void         reset_pan_tilt();
  virtual void         set_pan_tilt_limit(int pan_left, int pan_right,
					  int tilt_up, int tilt_down);
  virtual void         reset_pan_tilt_limit();


  virtual bool         supports_zoom();
  virtual void         reset_zoom();
  virtual void         set_zoom(unsigned int zoom);
  virtual unsigned int zoom();
  virtual unsigned int zoom_max();
  virtual unsigned int zoom_min();
  virtual void         set_zoom_speed_tele(unsigned int speed);
  virtual void         set_zoom_speed_wide(unsigned int speed);
  virtual void         set_zoom_digital_enabled(bool enabled);

  virtual bool         supports_effects();
  virtual bool         supports_effect(unsigned int effect);
  virtual void         set_effect(unsigned int effect);
  virtual unsigned int effect();
  virtual void         reset_effect();

  virtual unsigned int white_balance_mode();

};

#endif
