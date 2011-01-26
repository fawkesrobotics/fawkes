
/***************************************************************************
 *  source.cpp - Abstract class defining a camera source controller
 *
 *  Created: Mon Jun 29 15:47:11 2009
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

#include <fvcams/control/source.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControlSource <fvcams/control/source.h>
 * Camera source control interface.
 * Some cameras have multiple image sources; with this control, it is
 * possible to switch between them.
 *
 * @author Tobias Kellner
 *
 *
 * @fn unsigned char CameraControlSource::source() = 0
 * Return the currently selected image source.
 * @return id of the currently selected source
 *
 * @fn void CameraControlSource::set_source(unsigned char source) = 0
 * Set the current image source.
 * @param source id of the new source
 */

 /** Empty virtual destructor. */
CameraControlSource::~CameraControlSource()
{
}

} // end namespace firevision
