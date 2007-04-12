
/***************************************************************************
 *  dummy_control.h - controller that controls nothing, sounds like a stupid
 *                    idea but this avoids NULL checks in software using
 *                    a camera controller
 *
 *  Created: Wed Jun 15 12:45:57 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_CAMS_DUMMY_CONTROL_H_
#define __FIREVISION_CAMS_DUMMY_CONTROL_H_

#include <cams/cameracontrol.h>

/** Plain dummy control.
 * Does nothing, supports nothing. Use to avoid unecessary NULL checks
 */
class DummyControl : public CameraControl
{
};

#endif
