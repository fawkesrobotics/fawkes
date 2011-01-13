
/***************************************************************************
 *  effect.cpp - Abstract class defining a camera effect controller
 *
 *  Created: Wed Apr 22 11:01:18 2009
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

#include <fvcams/control/effect.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControlEffect <fvcams/control/effect.h>
 * Camera effect control interface.
 * Some cameras feature camera effects.
 *
 * This interface shall be implemented by such cameras.
 *
 * @author Tim Niemueller
 * @author Tobias Kellner
 *
 * @fn bool CameraControlEffect::supports_effect(unsigned int effect) = 0
 * Check if camera control supports desired effect.
 * Use camera-specific constants.
 * @param effect supported effect
 * @return true, if effect is supported, false otherwise
 *
 * @fn void CameraControlEffect::set_effect(unsigned int effect) = 0
 * Enable effect.
 * @param effect camera-specific effect.
 *
 * @fn unsigned int CameraControlEffect::effect() = 0
 * Current effect.
 * @return current effect.
 *
 * @fn void CameraControlEffect::reset_effect() = 0
 * Reset effect.
 * Disable all effects.
 */

/** No effect constant.
 * This is the only effect constant defined in the interface. All others that
 * may exist are specific for each camera control implementation.
 */
const unsigned int CameraControlEffect::EFFECT_NONE = 0;


/** Empty virtual destructor. */
CameraControlEffect::~CameraControlEffect()
{
}

} // end namespace firevision
