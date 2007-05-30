
/***************************************************************************
 *  logging.h - Logging aspect for Fawkes
 *
 *  Created: Tue May 29 14:47:43 2007
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <aspect/vision.h>

/** @class VisionAspect aspect/vision.h
 * Thread aspect to use FireVision.
 *
 * It is guaranteed that if used properly from within plugins that
 * initVisionAspect() is called before the thread is started and that
 * you can access the logger via the logger member.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** Virtual empty Destructor. */
VisionAspect::~VisionAspect()
{
}


/** Set vision master.
 * @param vision_master vision master
 * It is guaranteed that this is called for a logging thread before
 * Thread::start() is called (when running regularly inside Fawkes).
 * @see VisionMaster
 */
void
VisionAspect::initVisionAspect(VisionMaster *vision_master)
{
  this->vision_master = vision_master;
}
