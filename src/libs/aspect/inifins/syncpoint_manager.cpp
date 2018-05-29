/***************************************************************************
 *  syncpoint_manager.cpp - SyncPointManager Aspect initializer/finalizer
 *
 *  Created: Wed Jan 15 13:19:22 2014
 *  Copyright  2014  Till Hofmann
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

#include <aspect/inifins/syncpoint_manager.h>
#include <aspect/syncpoint_manager.h>
#include <syncpoint/syncpoint_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SyncPointManagerAspectIniFin <aspect/inifins/syncpoint_manager.h>
 * Initializer/finalizer for the SyncPointManagerAspect
 * @author Till Hofmann
 */

/** Constructor.
 * @param syncpoint_manager SyncPointManager instance to pass to threads
 */
SyncPointManagerAspectIniFin::SyncPointManagerAspectIniFin(SyncPointManager *syncpoint_manager)
: AspectIniFin("SyncPointManagerAspect")
{
  __syncpoint_manager = syncpoint_manager;
}

void
SyncPointManagerAspectIniFin::init(Thread *thread)
{
  SyncPointManagerAspect *syncpoint_thread;
  syncpoint_thread = dynamic_cast<SyncPointManagerAspect *>(thread);
  if (syncpoint_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
        "SyncPointManagerAspect, but RTTI says it "
        "has not. ", thread->name());
  }

  syncpoint_thread->init_SyncPointManagerAspect(__syncpoint_manager);

}


void
SyncPointManagerAspectIniFin::finalize(Thread *thread)
{

}


} // end namespace fawkes


