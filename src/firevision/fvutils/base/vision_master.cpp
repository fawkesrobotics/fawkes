
/***************************************************************************
 *  vision_master.cpp - FireVision Vision Master
 *
 *  Created: Wed May 30 10:52:08 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/base/vision_master.h>

/** @class VisionMaster <fvutils/base/vision_master.h>
 * Vision Master.
 * The vision master shall be the entry point for vision plugins. It shall
 * allow for requesting cameras that are opened in a central place such that
 * the very same camera can be used in multiple plugins.
 *
 * It shall also be responsible for the central timing of all vision threads.
 *
 * @author Tim Niemueller
 *
 * @fn Camera *  VisionMaster::request_camera(const char *camera_string) = 0
 * Get a camera.
 * @param camera_string camera that can be used by CameraFactory to open a
 * camera.
 * @return a reference to the requested camera. Note that this must not be
 * of the C++ type that you may expect for the requested camera, but it may
 * have layers of indirection.
 */

/** Virtual empty destructor. */
VisionMaster::~VisionMaster()
{
}
