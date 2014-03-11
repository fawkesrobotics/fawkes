
/***************************************************************************
 *  dummy.cpp - controller that controls nothing, sounds like a stupid
 *                    idea but this avoids NULL checks in software using
 *                    a camera controller
 *
 *  Created: Tue May 12 19:07:59 2009
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

#include <fvcams/control/dummy.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class DummyCameraControl <fvcams/control/dummy.h>
 * Dummy camera control.
 * This control supports every control type, but the operations are NOOPs.
 * This is useful to instantiate the dummy control to avoid NULL checks.
 * @author Tim Niemueller
 */

bool DummyCameraControl::auto_gain()
{
  return true;
}

void DummyCameraControl::set_auto_gain(bool enabled)
{
}

bool DummyCameraControl::auto_white_balance()
{
  return true;
}

void DummyCameraControl::set_auto_white_balance(bool enabled)
{
}

unsigned int DummyCameraControl::exposure_auto()
{
  return 0;
}

void DummyCameraControl::set_exposure_auto(unsigned int exposure_auto)
{
}


int DummyCameraControl::red_balance()
{
  return 0;
}

void DummyCameraControl::set_red_balance(int red_balance)
{
}

int DummyCameraControl::blue_balance()
{
  return 0;
}

void DummyCameraControl::set_blue_balance(int blue_balance)
{
}


int DummyCameraControl::u_balance()
{
  return 0;
}

void DummyCameraControl::set_u_balance(int u_balance)
{
}

int DummyCameraControl::v_balance()
{
  return 0;
}

void DummyCameraControl::set_v_balance(int v_balance)
{
}


unsigned int DummyCameraControl::brightness()
{
  return 0;
}

void DummyCameraControl::set_brightness(unsigned int brightness)
{
}

unsigned int DummyCameraControl::contrast()
{
  return 0;
}

void DummyCameraControl::set_contrast(unsigned int contrast)
{
}

unsigned int DummyCameraControl::saturation()
{
  return 0;
}

void DummyCameraControl::set_saturation(unsigned int saturation)
{
}

int DummyCameraControl::hue()
{
  return 0;
}

void DummyCameraControl::set_hue(int hue)
{
}

unsigned int DummyCameraControl::exposure()
{
  return 0;
}

void DummyCameraControl::set_exposure(unsigned int exposure)
{
}

unsigned int DummyCameraControl::gain()
{
  return 0;
}

void DummyCameraControl::set_gain(unsigned int gain)
{
}

  // From CameraControlImage
const char * DummyCameraControl::format()
{
  return "";
}

void DummyCameraControl::set_format(const char *format)
{
}

unsigned int DummyCameraControl::width()
{
  return 0;
}

unsigned int DummyCameraControl::height()
{
  return 0;
}

void DummyCameraControl::size(unsigned int &width, unsigned int &height)
{
  width = height = 0;
}

void DummyCameraControl::set_size(unsigned int width,
				unsigned int height)
{
}

bool DummyCameraControl::horiz_mirror()
{
  return false;
}

bool DummyCameraControl::vert_mirror()
{
  return false;
}

void DummyCameraControl::mirror(bool &horiz, bool &vert)
{
  horiz = vert = false;
}

void DummyCameraControl::set_horiz_mirror(bool enabled)
{
}

void DummyCameraControl::set_vert_mirror(bool enabled)
{
}

void DummyCameraControl::set_mirror(bool horiz, bool vert)
{
}


unsigned int DummyCameraControl::fps()
{
  return 0;
}

void DummyCameraControl::set_fps(unsigned int fps)
{
}


unsigned int DummyCameraControl::lens_x_corr()
{
  return 0;
}

unsigned int DummyCameraControl::lens_y_corr()
{
  return 0;
}

void DummyCameraControl::lens_corr(unsigned int &x_corr, unsigned int &y_corr)
{
  x_corr = y_corr = 0;
}

void DummyCameraControl::set_lens_x_corr(unsigned int x_corr)
{
}

void DummyCameraControl::set_lens_y_corr(unsigned int y_corr)
{
}

void DummyCameraControl::set_lens_corr(unsigned int x_corr, unsigned int y_corr)
{
}

void DummyCameraControl::process_pantilt()
{
}


bool DummyCameraControl::supports_pan()
{
  return false;
}

bool DummyCameraControl::supports_tilt()
{
  return false;
}

void DummyCameraControl::set_pan(int pan)
{
}

void DummyCameraControl::set_tilt(int tilt)
{
}

void DummyCameraControl::set_pan_tilt(int pan, int tilt)
{
}

void DummyCameraControl::set_pan_tilt_rad(float pan, float tilt)
{
}

int DummyCameraControl::pan()
{
  return 0;
}

int DummyCameraControl::tilt()
{
  return 0;
}

void DummyCameraControl::start_get_pan_tilt()
{
}

void DummyCameraControl::pan_tilt(int &pan, int &tilt)
{
  pan = tilt = 0;
}

void DummyCameraControl::pan_tilt_rad(float &pan, float &tilt)
{
  pan = tilt = 0.f;
}

int DummyCameraControl::min_pan()
{
  return 0;
}

int DummyCameraControl::max_pan()
{
  return 0;
}

int DummyCameraControl::min_tilt()
{
  return 0;
}

int DummyCameraControl::max_tilt()
{
  return 0;
}

void DummyCameraControl::reset_pan_tilt()
{
}

void DummyCameraControl::set_pan_tilt_limit(int pan_left, int pan_right,
					    int tilt_up, int tilt_down)
{
}

void DummyCameraControl::reset_pan_tilt_limit()
{
}

void DummyCameraControl::reset_zoom()
{
}

void DummyCameraControl::set_zoom(unsigned int zoom)
{
}

unsigned int DummyCameraControl::zoom()
{
  return 0;
}

unsigned int DummyCameraControl::zoom_max()
{
  return 0;
}

unsigned int DummyCameraControl::zoom_min()
{
  return 0;
}

void DummyCameraControl::set_zoom_speed_tele(unsigned int speed)
{
}

void DummyCameraControl::set_zoom_speed_wide(unsigned int speed)
{
}

void DummyCameraControl::set_zoom_digital_enabled(bool enabled)
{
}

bool DummyCameraControl::supports_effect(unsigned int effect)
{
  return false;
}

void DummyCameraControl::set_effect(unsigned int effect)
{
}

unsigned int DummyCameraControl::effect()
{
  return EFFECT_NONE;
}

void DummyCameraControl::reset_effect()
{
}

bool DummyCameraControl::auto_focus()
{
  return true;
}

void DummyCameraControl::set_auto_focus(bool enabled)
{
}

unsigned int DummyCameraControl::focus()
{
  return 0;
}

void DummyCameraControl::set_focus(unsigned int focus)
{
}

unsigned int DummyCameraControl::focus_min()
{
  return 0;
}

unsigned int DummyCameraControl::focus_max()
{
  return 0;
}

} // end namespace firevision
