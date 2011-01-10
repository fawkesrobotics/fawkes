
/***************************************************************************
 *  rrd_inifin.cpp - Fawkes RRDAspect initializer/finalizer
 *
 *  Created: Mon Dec 06 22:33:03 2010
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

#include <plugins/rrd/aspect/rrd_inifin.h>
#include <plugins/rrd/aspect/rrd.h>
#include <plugins/rrd/aspect/rrd_manager.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RRDAspectIniFin <plugins/rrd/aspect/rrd_inifin.h>
 * RRDAspect initializer/finalizer.
 * This initializer/finalizer will provide the RRDManager to threads with
 * the RRDAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param rrd_manager RRD manager to pass on to threads
 */
RRDAspectIniFin::RRDAspectIniFin(RRDManager *rrd_manager)
  : AspectIniFin("RRDAspect")
{
  __rrd_manager = rrd_manager;
}

void
RRDAspectIniFin::init(Thread *thread)
{
  RRDAspect *rrd_thread;
  rrd_thread = dynamic_cast<RRDAspect *>(thread);
  if (rrd_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "RRDAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  rrd_thread->init_RRDAspect(__rrd_manager);
}

void
RRDAspectIniFin::finalize(Thread *thread)
{
}



} // end namespace fawkes
