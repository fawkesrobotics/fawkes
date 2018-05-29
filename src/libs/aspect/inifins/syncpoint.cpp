/***************************************************************************
 *  syncpoint.cpp - SyncPoint Aspect initializer/finalizer
 *
 *  Created: Thu Feb 19 14:39:42 2015
 *  Copyright  2015  Till Hofmann
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <aspect/inifins/syncpoint.h>
#include <aspect/syncpoint.h>
#include <syncpoint/syncpoint_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SyncPointAspectIniFin <aspect/inifins/syncpoint.h>
 * Initializer/finalizer for the SyncPointAspect
 * @author Till Hofmann
 */

/** Constructor.
 * @param syncpoint_manager SyncPointManager instance to pass to threads
 */
SyncPointAspectIniFin::SyncPointAspectIniFin(SyncPointManager *syncpoint_manager)
: AspectIniFin("SyncPointAspect")
{
  __syncpoint_manager = syncpoint_manager;
}

void
SyncPointAspectIniFin::init(Thread *thread)
{
  SyncPointAspect *syncpoint_thread;
  syncpoint_thread = dynamic_cast<SyncPointAspect *>(thread);
  if (syncpoint_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
        "SyncPointManagerAspect, but RTTI says it "
        "has not. ", thread->name());
  }

  syncpoint_thread->init_SyncPointAspect(thread, __syncpoint_manager);

}

void
SyncPointAspectIniFin::finalize(Thread *thread)
{
  SyncPointAspect *syncpoint_thread;
  syncpoint_thread = dynamic_cast<SyncPointAspect *>(thread);
  if (syncpoint_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
        "SyncPointManagerAspect, but RTTI says it "
        "has not. ", thread->name());
  }

  syncpoint_thread->finalize_SyncPointAspect(thread, __syncpoint_manager);

}


} // end namespace fawkes


