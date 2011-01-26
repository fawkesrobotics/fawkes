
/***************************************************************************
 *  vision_master.cpp - Fawkes VisionMasterAspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:06:13 2010
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

#include <aspect/inifins/vision_master.h>
#include <aspect/vision_master.h>
#include <fvutils/base/vision_master.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class VisionMasterAspectIniFin <aspect/inifins/vision_master.h>
 * Initializer/finalizer for the VisionMasterAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
VisionMasterAspectIniFin::VisionMasterAspectIniFin()
  : AspectIniFin("VisionMasterAspect")
{
}


void
VisionMasterAspectIniFin::init(Thread *thread)
{
  VisionMasterAspect *vision_master_thread;
  vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread);
  if (vision_master_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "VisionMasterAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  try {
    __vision_dependency.add(vision_master_thread);
  } catch (DependencyViolationException &e) {
    CannotInitializeThreadException ce("Dependency violation for "
				       "VisionMasterAspect detected");
    ce.append(e);
    throw ce;
  }
}


bool
VisionMasterAspectIniFin::prepare_finalize(Thread *thread)
{
  VisionMasterAspect *vision_master_thread;
  vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread);
  if (vision_master_thread == 0) {
    return true;
  }

  if ( ! __vision_dependency.can_remove(vision_master_thread) ) {
    //__logger->log_warn("AspectIniFin", "Cannot remove vision master, there are "
    //		"still vision threads that depend on it");
    return false;
  }

  return true;
}

void
VisionMasterAspectIniFin::finalize(Thread *thread)
{
  VisionMasterAspect *vision_master_thread;
  vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread);
  if (vision_master_thread == 0) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"VisionMasterAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  try {
    __vision_dependency.remove(vision_master_thread);
  } catch (DependencyViolationException &e) {
    CannotFinalizeThreadException ce("Dependency violation for "
				     "VisionMasterAspect detected");
    ce.append(e);
    throw ce;
  }
}


/** Get vision master.
 * @return vision master
 */
firevision::VisionMaster *
VisionMasterAspectIniFin::vision_master()
{
  return __vision_dependency.provider()->vision_master();
}


/** Add a vision thread.
 * @param thread thread to add
 */
void
VisionMasterAspectIniFin::add_vision_thread(VisionAspect *thread)
{
  __vision_dependency.add(thread);
}

/** Remove a vision thread.
 * @param thread thread to remove
 */
void
VisionMasterAspectIniFin::remove_vision_thread(VisionAspect *thread)
{
  __vision_dependency.remove(thread);
}

/** Query if vision thread can be removed.
 * @param thread thread to query for
 * @return true if the thread can be removed, false otherwise
 */
bool
VisionMasterAspectIniFin::can_remove_vision_thread(VisionAspect *thread)
{
  return __vision_dependency.can_remove(thread);
}


} // end namespace fawkes
