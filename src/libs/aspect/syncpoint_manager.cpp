/***************************************************************************
 *  syncpoint_manager.cpp - SyncPointManager Aspect
 *
 *  Created: Thu Jan 09 12:25:13 2014
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

#include <aspect/syncpoint_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SyncPointManagerAspect <aspect/syncpoint_manager.h>
 * Thread aspect to acces to SyncPoints
 * Give this aspect to your thread to manage SyncPoints,
 * i.e. wait for SyncPoints and emit SyncPoints
 * @ingroup Aspects
 * @author Till Hofmann
 */

/** @var SyncPoint * SyncPointManagerAspect::syncpoint_manager
 * This is the SyncPointManager instance you can use to manage syncpoints.
 */

/** Constructor. */
SyncPointManagerAspect::SyncPointManagerAspect()
{
  add_aspect("SyncPointManagerAspect");
  syncpoint_manager = 0;
}

SyncPointManagerAspect::~SyncPointManagerAspect()
{
}

/** Init SyncPointManager aspect.
 * This sets the SyncPointManager that can be used to manage SyncPoints.
 * @param manager SyncPointManager to use
 */
void
SyncPointManagerAspect::init_SyncPointManagerAspect(SyncPointManager *manager)
{
  syncpoint_manager = manager;
}

} // end namespace fawkes
