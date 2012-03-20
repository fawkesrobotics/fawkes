
/***************************************************************************
 *  vision.cpp - Fawkes VisionAspect initializer/finalizer
 *
 *  Created: Wed Nov 24 00:13:36 2010
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

#include <aspect/inifins/vision.h>
#include <aspect/inifins/vision_master.h>
#include <aspect/vision.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class VisionAspectIniFin <aspect/inifins/vision.h>
 * Initializer/finalizer for the VisionAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param master_inifin vision master aspect inifin to get master from
 */
VisionAspectIniFin::VisionAspectIniFin(VisionMasterAspectIniFin *master_inifin)
  : AspectIniFin("VisionAspect")
{
  __master_inifin = master_inifin;
}


void
VisionAspectIniFin::init(Thread *thread)
{
  VisionAspect *vision_thread;
  vision_thread = dynamic_cast<VisionAspect *>(thread);
  if (vision_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "VisionAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  try {
    if ( (vision_thread->vision_thread_mode() == VisionAspect::CONTINUOUS) &&
	 (thread->opmode() != Thread::OPMODE_CONTINUOUS) ) {
      throw CannotInitializeThreadException("Vision thread '%s' operates in "
					    "continuous mode but thread does not",
					    thread->name());
    }
    if ( (vision_thread->vision_thread_mode() == VisionAspect::CYCLIC) &&
	 (thread->opmode() != Thread::OPMODE_WAITFORWAKEUP) ) {
      throw CannotInitializeThreadException("Vision thread '%s' operates in "
					    "cyclic mode but thread does not "
					    "operate in wait-for-wakeup mode.",
					    thread->name());
    }

    __master_inifin->add_vision_thread(vision_thread);
    vision_thread->init_VisionAspect(__master_inifin->vision_master());
  } catch (DependencyViolationException &e) {
    CannotInitializeThreadException ce("Dependency violation for "
				       "VisionAspect detected");
    ce.append(e);
    throw ce;
  }

}


bool
VisionAspectIniFin::prepare_finalize(Thread *thread)
{
  VisionAspect *vision_thread;
  vision_thread = dynamic_cast<VisionAspect *>(thread);
  if (vision_thread == 0) {
    return true;
  }

  if ( ! __master_inifin->can_remove_vision_thread(vision_thread) ) {
    //__logger->log_warn("AspectIniFin", "Cannot remove vision master, there are "
    //		"still vision threads that depend on it");
    return false;
  }

  return true;
}

void
VisionAspectIniFin::finalize(Thread *thread)
{
  VisionAspect *vision_thread;
  vision_thread = dynamic_cast<VisionAspect *>(thread);
  if (vision_thread == 0) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"VisionAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  try {
    __master_inifin->remove_vision_thread(vision_thread);
  } catch (DependencyViolationException &e) {
    CannotFinalizeThreadException ce("Dependency violation for "
				     "VisionAspect detected");
    ce.append(e);
    throw ce;
  }
}

} // end namespace fawkes
