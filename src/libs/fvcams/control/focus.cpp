
/***************************************************************************
 *  focus.cpp - Abstract class defining a camera focus controller
 *
 *  Created: Wed Apr 22 10:37:16 2009
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

#include <fvcams/control/focus.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControlFocus <fvcams/control/focus.h>
 * Camera focus control interface.
 * Some cameras feature an adjustable focus.
 *
 * This interface shall be implemented by such cameras.
 *
 * @author Tim Niemueller
 * @author Tobias Kellner
 *
 * @fn bool CameraControlFocus::auto_focus() = 0
 * Check if auto focus is enabled.
 * @return true, if the camera is in auto focus mode, false otherwise
 * @throws NotImplementedException Not implemented by this control
 *
 * @fn void CameraControlFocus::set_auto_focus(bool enabled) = 0
 * Enable or disable auto focus.
 * @param enabled if true, enable auto focus, otherwise disable
 * @throws NotImplementedException Not implemented by this control
 *
 * @fn unsigned int CameraControlFocus::focus()
 * Get current focus value.
 * @return current focus value.
 *
 * @fn void CameraControlFocus::set_focus(unsigned int focus)
 * Set new focus value.
 * @param focus new focus value
 *
 * @fn unsigned int CameraControlFocus::focus_min()
 * Get minimum focus value.
 * @return minimum focus value.
 *
 * @fn unsigned int CameraControlFocus::focus_max()
 * Get maximum focus value.
 * @return maximum focus value.
 */

/** Empty virtual destructor. */
CameraControlFocus::~CameraControlFocus()
{
}

} // end namespace firevision
