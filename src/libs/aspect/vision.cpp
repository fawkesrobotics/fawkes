
/***************************************************************************
 *  vision.cpp - Vision aspect for Fawkes
 *
 *  Created: Tue May 29 14:47:43 2007
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/vision.h>

namespace fawkes {

/** @class VisionAspect <aspect/vision.h>
 * Thread aspect to use in FireVision apps.
 *
 * It is guaranteed that if used properly from within plugins that
 * initVisionAspect() is called before the thread is started and that
 * you can access the vision master via the vision_master member.
 *
 * A vision thread can be called either cyclic, which means that in every
 * loop the vision master will wait for this vision thread to finish before
 * the next loop. This also means that the thread has to operate in
 * wait-for-wakeup mode. The thread is woken up when a new camera image is
 * available. In general the vision thread should be very fast and under no
 * conditions it should take longer to process an image than to aquire it.
 * The thread can also operate in continuous mode, in this case also the
 * thread has to operate in continuous mode. In this mode the vision
 * application should keep running and the processing is independent from
 * the camera speed. Make sure that you use strict logging on the shared
 * memory camera to ensure healthy pictures.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** Constructor.
 * @param mode mode to operate in
 */
VisionAspect::VisionAspect(VisionThreadMode mode)
{
  add_aspect("VisionAspect");
  __vision_thread_mode = mode;
}


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
VisionAspect::init_VisionAspect(firevision::VisionMaster *vision_master)
{
  this->vision_master = vision_master;
}


/** Get the vision thread mode of this thread.
 * @return vision thread mode
 */
VisionAspect::VisionThreadMode
VisionAspect::vision_thread_mode()
{
  return __vision_thread_mode;
}

} // end namespace fawkes
