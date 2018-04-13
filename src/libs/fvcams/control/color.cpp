
/***************************************************************************
 *  color.cpp - Abstract class defining a camera color controller
 *
 *  Created: Wed Apr 22 11:19:04 2009
 *  Copyright  2009      Tobias Kellner
 *             2005-2009 Tim Niemueller [www.niemueller.de]
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

#include <fvcams/control/color.h>
#include <core/exceptions/software.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControlColor <fvcams/control/color.h>
 * Camera color control interface.
 * Some cameras feature adjustable color controls
 * like white balance, brightness etc.
 * In general methods might throw an NotImplementedException if a particular
 * method if not available.
 *
 * This interface shall be implemented by such cameras.
 *
 * @author Tobias Kellner
 * @author Tim Niemueller
 *
 *
 * @fn bool CameraControlColor::auto_gain() = 0
 * Return whether auto gain is enabled.
 * @return true if auto gain is enabled
 *
 * @fn void CameraControlColor::set_auto_gain(bool enabled) = 0
 * Enable/disable auto gain.
 * @param enabled whether auto gain should be enabled
 *
 * @fn bool CameraControlColor::auto_white_balance() = 0
 * Return whether auto white balance is enabled.
 * @return true if auto white balance is enabled
 *
 * @fn void CameraControlColor::set_auto_white_balance(bool enabled) = 0
 * Enable/disable auto white balance.
 * @param enabled whether auto white balance should be enabled
 *
 * @fn bool CameraControlColor::exposure_auto() = 0
 * Return whether auto exposure is enabled.
 * @return true if auto exposure is enabled
 *
 * @fn void CameraControlColor::set_exposure_auto(bool enabled) = 0
 * Enable/disable auto exposure.
 * @param enabled whether auto exposure should be enabled
 *
 * @fn int CameraControlColor::red_balance() = 0
 * Get current red balance.
 * @return current red balance value
 *
 * @fn int CameraControlColor::set_red_balance(int red_balance) = 0
 * Set red balance.
 * @param red_balance new red balance
 *
 * @fn int CameraControlColor::blue_balance() = 0
 * Get current blue balance.
 * @return current blue balance value
 *
 * @fn void CameraControlColor::set_blue_balance(int blue_balance) = 0
 * Set blue balance.
 * @param blue_balance new blue balance
 *
 * @fn int CameraControlColor::u_balance() = 0
 * Get current u balance.
 * @return current u balance value
 *
 * @fn void CameraControlColor::set_u_balance(int u_balance) = 0
 * Set u balance.
 * @param u_balance new u balance
 *
 * @fn int CameraControlColor::v_balance() = 0
 * Get current v balance.
 * @return current v balance value
 *
 * @fn void CameraControlColor::set_v_balance(int v_balance) = 0
 * Set v balance.
 * @param v_balance new v balance
 *
 * @fn unsigned int CameraControlColor::brightness() = 0
 * Get current brightness.
 * @return current brightness value
 *
 * @fn void CameraControlColor::set_brightness(unsigned int brightness) = 0
 * Set new brightness.
 * @param brightness new brightness
 *
 * @fn unsigned int CameraControlColor::contrast() = 0
 * Get current contrast.
 * @return current contrast value
 *
 * @fn void CameraControlColor::set_contrast(unsigned int contrast) = 0
 * Set new contrast.
 * @param contrast new contrast
 *
 * @fn unsigned int CameraControlColor::saturation() = 0
 * Get current saturation.
 * @return current saturation value
 *
 * @fn void CameraControlColor::set_saturation(unsigned int saturation) = 0
 * Set new saturation.
 * @param saturation new saturation
 *
 * @fn int CameraControlColor::hue() = 0
 * Get current hue.
 * @return current hue value
 *
 * @fn void CameraControlColor::set_hue(int hue) = 0
 * Set new hue.
 * @param hue new hue
 *
 * @fn unsigned int CameraControlColor::exposure() = 0
 * Get current exposure
 * @return current exposure value
 *
 * @fn void CameraControlColor::set_exposure(unsigned int exposure) = 0
 * Set new exposure.
 * @param exposure new exposure
 *
 * @fn unsigned int CameraControlColor::gain() = 0
 * Get current gain.
 * @return current gain value
 *
 * @fn void CameraControlColor::set_gain(unsigned int gain) = 0
 * Set new gain.
 * @param gain new gain
 */

using fawkes::NotImplementedException;

/** Empty virtual destructor. */
CameraControlColor::~CameraControlColor()
{
}


/** Enable/disable all automatic settings.
 * Most of the time, you'll want to disable all of them.
 * @param enabled whether the automatic settings should be enabled or disabled
 */
void
CameraControlColor::set_auto_all(bool enabled)
{
  try {
    set_auto_gain(enabled);
  } catch (NotImplementedException &e) {}
  try {
    set_auto_white_balance(enabled);
  } catch (NotImplementedException &e) {}
  try {
    set_exposure_auto(enabled);
  } catch (NotImplementedException &e) {}
}

} // end namespace firevision
