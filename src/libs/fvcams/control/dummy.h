
/***************************************************************************
 *  dummy.h - controller that controls nothing, sounds like a stupid
 *                    idea but this avoids NULL checks in software using
 *                    a camera controller
 *
 *  Created: Wed Jun 15 12:45:57 2005
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

#ifndef __FIREVISION_CAMS_DUMMY_CONTROL_H_
#define __FIREVISION_CAMS_DUMMY_CONTROL_H_

#include <fvcams/control/color.h>
#include <fvcams/control/image.h>
#include <fvcams/control/pantilt.h>
#include <fvcams/control/zoom.h>
#include <fvcams/control/effect.h>
#include <fvcams/control/focus.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Plain dummy control.
 * Does nothing, supports nothing. Use to avoid unecessary NULL checks
 */
class DummyCameraControl
: public CameraControlColor,
  public CameraControlImage,
  public CameraControlPanTilt,
  public CameraControlZoom,
  public CameraControlEffect,
  public CameraControlFocus
{
 public:
  // from CameraControlColor
  virtual bool         auto_gain();
  virtual void         set_auto_gain(bool enabled);
  virtual bool         auto_white_balance();
  virtual void         set_auto_white_balance(bool enabled);
  virtual unsigned int exposure_auto();
  virtual void         set_exposure_auto(unsigned int enabled);

  virtual int          red_balance();
  virtual void         set_red_balance(int red_balance);
  virtual int          blue_balance();
  virtual void         set_blue_balance(int blue_balance);

  virtual int          u_balance();
  virtual void         set_u_balance(int u_balance);
  virtual int          v_balance();
  virtual void         set_v_balance(int v_balance);

  virtual unsigned int brightness();
  virtual void         set_brightness(unsigned int brightness);
  virtual unsigned int contrast();
  virtual void         set_contrast(unsigned int contrast);
  virtual unsigned int saturation();
  virtual void         set_saturation(unsigned int saturation);
  virtual int          hue();
  virtual void         set_hue(int hue);
  virtual unsigned int exposure();
  virtual void         set_exposure(unsigned int exposure);
  virtual unsigned int gain();
  virtual void         set_gain(unsigned int gain);

  // From CameraControlImage
  virtual const char * format();
  virtual void         set_format(const char *format);
  virtual unsigned int width();
  virtual unsigned int height();
  virtual void         size(unsigned int &width, unsigned int &height);
  virtual void         set_size(unsigned int width, unsigned int height);
  virtual bool         horiz_mirror();
  virtual bool         vert_mirror();
  virtual void         mirror(bool &horiz, bool &vert);
  virtual void         set_horiz_mirror(bool enabled);
  virtual void         set_vert_mirror(bool enabled);
  virtual void         set_mirror(bool horiz, bool vert);

  virtual unsigned int fps();
  virtual void         set_fps(unsigned int fps);

  virtual unsigned int lens_x_corr();
  virtual unsigned int lens_y_corr();
  virtual void         lens_corr(unsigned int &x_corr, unsigned int &y_corr);
  virtual void         set_lens_x_corr(unsigned int x_corr);
  virtual void         set_lens_y_corr(unsigned int y_corr);
  virtual void         set_lens_corr(unsigned int x_corr, unsigned int y_corr);

  // From CameraControlPanTilt
  virtual void process_pantilt();

  virtual bool supports_pan();
  virtual bool supports_tilt();
  virtual void set_pan(int pan);
  virtual void set_tilt(int tilt);
  virtual void set_pan_tilt(int pan, int tilt);
  virtual void set_pan_tilt_rad(float pan, float tilt);
  virtual int  pan();
  virtual int  tilt();
  virtual void start_get_pan_tilt();
  virtual void pan_tilt(int &pan, int &tilt);
  virtual void pan_tilt_rad(float &pan, float &tilt);
  virtual int  min_pan();
  virtual int  max_pan();
  virtual int  min_tilt();
  virtual int  max_tilt();
  virtual void reset_pan_tilt();
  virtual void set_pan_tilt_limit(int pan_left, int pan_right,
                                  int tilt_up, int tilt_down);
  virtual void reset_pan_tilt_limit();

  // From CameraControlZoom
  virtual void         reset_zoom();
  virtual void         set_zoom(unsigned int zoom);
  virtual unsigned int zoom();
  virtual unsigned int zoom_max();
  virtual unsigned int zoom_min();
  virtual void         set_zoom_speed_tele(unsigned int speed);
  virtual void         set_zoom_speed_wide(unsigned int speed);
  virtual void         set_zoom_digital_enabled(bool enabled);

  // from CameraControlEffect
  virtual bool         supports_effect(unsigned int effect);
  virtual void         set_effect(unsigned int effect);
  virtual unsigned int effect();
  virtual void         reset_effect();

  // from CameraControlFocus
  virtual bool         auto_focus();
  virtual void         set_auto_focus(bool enabled);
  virtual unsigned int focus();
  virtual void         set_focus(unsigned int focus);
  virtual unsigned int focus_min();
  virtual unsigned int focus_max();

};

} // end namespace firevision

#endif
