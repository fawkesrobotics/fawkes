
/***************************************************************************
 *  sony_evid100p_control.cpp - Controller for Sony EVI-D100P
 *
 *  Created: Tue Jun 07 19:27:20 2005
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

#include <fvcams/control/sony_evid100p.h>
#include <fvcams/control/visca.h>
#include <fvutils/system/camargp.h>

#include <utils/math/angle.h>

#include <termios.h>
#include <cstring>
#include <cstdlib>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Maximum pan. */
const int   SonyEviD100PControl::MAX_PAN       =  1440;
/** Minimum pan. */
const int   SonyEviD100PControl::MIN_PAN       = -1439;
/** Max Tilt. */
const int   SonyEviD100PControl::MAX_TILT      =   360;
/** Min tilt .*/
const int   SonyEviD100PControl::MIN_TILT      = - 359;

/** Max pan in degrees. */
const float SonyEviD100PControl::MAX_PAN_DEG   =  100.f;
/** Min pan in degrees. */
const float SonyEviD100PControl::MIN_PAN_DEG   = -100.f;
/** Max tilt in degrees. */
const float SonyEviD100PControl::MAX_TILT_DEG  =   25.f;
/** Min tilt in degrees. */
const float SonyEviD100PControl::MIN_TILT_DEG  = - 25.f;

/** Max pan in rad. */
const float SonyEviD100PControl::MAX_PAN_RAD   = deg2rad(MAX_PAN_DEG);
/** Min pan in rad. */
const float SonyEviD100PControl::MIN_PAN_RAD   = deg2rad(MIN_PAN_DEG);
/** Max tilt in rad. */
const float SonyEviD100PControl::MAX_TILT_RAD  = deg2rad(MAX_TILT_DEG);
/** Min tilt in rad. */
const float SonyEviD100PControl::MIN_TILT_RAD  = deg2rad(MIN_TILT_DEG);

/** Pan steps per degree */
const float SonyEviD100PControl::PAN_STEPS_PER_DEG  = MAX_PAN  / MAX_PAN_DEG;
/** Tilt steps per degree */
const float SonyEviD100PControl::TILT_STEPS_PER_DEG = MAX_TILT / MAX_TILT_DEG;

/** Pan steps per rad */
const float SonyEviD100PControl::PAN_STEPS_PER_RAD  = MAX_PAN  / MAX_PAN_RAD;
/** Tilt steps per rad */
const float SonyEviD100PControl::TILT_STEPS_PER_RAD = MAX_TILT / MAX_TILT_RAD;

/** Pastel effect. */
const unsigned int SonyEviD100PControl::EFFECT_PASTEL   = 1;
/** Negative effect. */
const unsigned int SonyEviD100PControl::EFFECT_NEGATIVE = 2;
/** Sepia effect. */
const unsigned int SonyEviD100PControl::EFFECT_SEPIA    = 3;
/** B/W effect. */
const unsigned int SonyEviD100PControl::EFFECT_BW       = 4;
/** Solarize effect. */
const unsigned int SonyEviD100PControl::EFFECT_SOLARIZE = 5;
/** Mosaic effect. */
const unsigned int SonyEviD100PControl::EFFECT_MOSAIC   = 6;
/** Slim effect. */
const unsigned int SonyEviD100PControl::EFFECT_SLIM     = 7;
/** Stretch effect. */
const unsigned int SonyEviD100PControl::EFFECT_STRETCH  = 8;


/** @class SonyEviD100PControl <fvcams/control/sony_evid100p.h>
 * Sony Evi D100P pan/tilt control.
 * Internally uses Visca.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param tty_port serial port (e.g. /dev/ttyS0)
 */
SonyEviD100PControl::SonyEviD100PControl(const char *tty_port)
{
  this->tty_port = strdup(tty_port);
  visca = new ViscaControl( /* non-blocking */ false );
  opened = false;
  pan_target = 0;
  tilt_target = 0;
  _effect = EFFECT_NONE;

  open();
}


/** Constructor.
 * Uses camera argument parser to gather arguments. The ID that the camera argument
 * parser returns is used as the serial port (like /dev/ttyS0).
 * @param cap camera argument parser
 */
SonyEviD100PControl::SonyEviD100PControl(const CameraArgumentParser *cap)
{
  tty_port = strdup(cap->cam_id().c_str());

  visca = new ViscaControl( /* non-blocking */ false );
  opened = false;
  pan_target = 0;
  tilt_target = 0;
  _effect = EFFECT_NONE;

  open();
}


/** Destructor. */
SonyEviD100PControl::~SonyEviD100PControl()
{
  close();
  delete visca;
  free(tty_port);
}


/** Open visca device.
 * @return true on success
 */
void
SonyEviD100PControl::open()
{
  if (opened) return;

  try {
    visca->open(tty_port);
    visca->set_address(1);
    visca->clear();
  } catch (ViscaControlException &e) {
    visca->close();
    e.append("Sony EviD100PControl failed");
    throw;
  }

  opened = true;
}


/** Close Visca device.
 */
void
SonyEviD100PControl::close()
{
  if ( ! opened ) return;
  visca->close();
}


void
SonyEviD100PControl::process_pantilt()
{
  visca->process();
}


bool
SonyEviD100PControl::supports_pan()
{
  return true;
}


bool
SonyEviD100PControl::supports_tilt()
{
  return true;
}


void
SonyEviD100PControl::set_pan(int pan)
{
  pan_target = pan;
  visca->setPanTilt(pan, tilt_target);
}


void
SonyEviD100PControl::set_tilt(int tilt)
{
  tilt_target = tilt;
  visca->setPanTilt(pan_target, tilt);
}


void
SonyEviD100PControl::set_pan_tilt(int pan, int tilt)
{
  pan_target  = pan;
  tilt_target = tilt;
  visca->setPanTilt(pan, tilt);
}


void
SonyEviD100PControl::set_pan_tilt_rad(float pan, float tilt)
{
  int tpan = 0, ttilt = 0;

  tpan = (int)rint(  pan  * PAN_STEPS_PER_RAD  );
  ttilt = (int)rint( tilt * TILT_STEPS_PER_RAD );

  set_pan_tilt(tpan, ttilt);
}


void
SonyEviD100PControl::start_get_pan_tilt()
{
  visca->startGetPanTilt();
}


void
SonyEviD100PControl::pan_tilt(int &pan, int &tilt)
{
  int tpan, ttilt;
  visca->getPanTilt(&tpan, &ttilt);
  pan  = tpan;
  tilt = ttilt;
}


void
SonyEviD100PControl::pan_tilt_rad(float &pan, float &tilt)
{
  int tpan = 0, ttilt = 0;
  visca->getPanTilt(&tpan, &ttilt);

  pan  = tpan  / PAN_STEPS_PER_RAD;
  tilt = ttilt / PAN_STEPS_PER_RAD;
}


int
SonyEviD100PControl::pan()
{
  int pan = 0, tilt = 0;
  visca->getPanTilt(&pan, &tilt);
  return pan;
}


int
SonyEviD100PControl::tilt()
{
  int pan = 0, tilt = 0;
  visca->getPanTilt(&pan, &tilt);
  return tilt;
}


int
SonyEviD100PControl::max_pan()
{
  return MAX_PAN;
}


int
SonyEviD100PControl::min_pan()
{
  return MIN_PAN;
}


int
SonyEviD100PControl::max_tilt()
{
  return MAX_TILT;
}


int
SonyEviD100PControl::min_tilt()
{
  return MIN_TILT;
}


void
SonyEviD100PControl::reset_pan_tilt()
{
  visca->resetPanTilt();
}


void
SonyEviD100PControl::set_pan_tilt_limit(int pan_left, int pan_right,
				     int tilt_up, int tilt_down)
{
  visca->setPanTiltLimit(pan_left, pan_right, tilt_up, tilt_down);
}


void
SonyEviD100PControl::reset_pan_tilt_limit()
{
  visca->resetPanTiltLimit();
}


void
SonyEviD100PControl::reset_zoom()
{
  visca->resetZoom();
}


void
SonyEviD100PControl::set_zoom(unsigned int zoom)
{
  visca->setZoom(zoom);
}


unsigned int
SonyEviD100PControl::zoom()
{
  unsigned int zoom;
  visca->getZoom(&zoom);
  return zoom;
}


unsigned int
SonyEviD100PControl::zoom_min()
{
  return 0;
}


unsigned int
SonyEviD100PControl::zoom_max()
{
  return 0x4000;
}


void
SonyEviD100PControl::set_zoom_speed_tele(unsigned int speed)
{
  visca->setZoomSpeedTele(speed);
}


void
SonyEviD100PControl::set_zoom_speed_wide(unsigned int speed)
{
  visca->setZoomSpeedWide(speed);
}


void
SonyEviD100PControl::set_zoom_digital_enabled(bool enabled)
{
  visca->setZoomDigitalEnabled(enabled);
}


bool
SonyEviD100PControl::supports_effect(unsigned int __effect)
{
  if ( __effect == EFFECT_NONE ) {
    return true;
  }

  switch (__effect) {
  case EFFECT_PASTEL:
  case EFFECT_NEGATIVE:
  case EFFECT_SEPIA:
  case EFFECT_BW:
  case EFFECT_SOLARIZE:
  case EFFECT_MOSAIC:
  case EFFECT_SLIM:
  case EFFECT_STRETCH:
    return true;
    break;
  default:
    return false;
  }
}


void
SonyEviD100PControl::set_effect(unsigned int __effect)
{
  this->_effect = __effect;
  if ( __effect == EFFECT_NONE ) {
    visca->resetEffect();
  }
  switch (__effect) {
  case EFFECT_PASTEL:
    visca->applyEffectPastel();
    break;
  case EFFECT_NEGATIVE:
    visca->applyEffectNegArt();
    break;
  case EFFECT_SEPIA:
    visca->applyEffectSepia();
    break;
  case EFFECT_BW:
    visca->applyEffectBnW();
    break;
  case EFFECT_SOLARIZE:
    visca->applyEffectSolarize();
    break;
  case EFFECT_MOSAIC:
    visca->applyEffectMosaic();
    break;
  case EFFECT_SLIM:
    visca->applyEffectSlim();
    break;
  case EFFECT_STRETCH:
    visca->applyEffectStretch();
    break;
  default:
    break;
  }

}


unsigned int
SonyEviD100PControl::effect()
{
  return _effect;
}


void
SonyEviD100PControl::reset_effect()
{
  visca->resetEffect();
}


/** Get current white balance mode.
 * @return white balance mode
 */
unsigned int
SonyEviD100PControl::white_balance_mode()
{
  return visca->getWhiteBalanceMode();
}

} // end namespace firevision
