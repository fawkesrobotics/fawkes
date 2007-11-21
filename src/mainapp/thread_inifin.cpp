
/***************************************************************************
 *  thread_inifin.h - Fawkes thread initializer/finalizer
 *
 *  Created: Thu Nov 20 00:53:50 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <mainapp/thread_inifin.h>
#include <core/threading/thread.h>
#include <blackboard/blackboard.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>

/** @class FawkesThreadIniFin mainapp/thread_inifin.h
 * Fawkes Thread Initializer/Finalizer.
 * Initializes threads that are added to the thread manager if needed.
 * Calls AspectIniFin::init() to initialize aspect threads.
 * @see AspectInitializer
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard BlackBoard
 * @param collector Thread collector
 * @param config Configuration
 * @param logger Logger
 * @param clock Clock
 */
FawkesThreadIniFin::FawkesThreadIniFin(BlackBoard *blackboard,
				       ThreadCollector *collector,
				       Configuration *config,
				       Logger *logger,
				       Clock *clock)
  : AspectIniFin(blackboard, collector, config, logger, clock)
{
}


void
FawkesThreadIniFin::init(Thread *thread)
{
  try {
    AspectIniFin::init(thread);
  } catch (Exception &e) {
    throw;
  }

  // put any special non-aspect initialization and checks here
  // currently there are non
}


void
FawkesThreadIniFin::finalize(Thread *thread)
{
  try {
    AspectIniFin::finalize(thread);
  } catch (Exception &e) {
    throw;
  }

  // put any special non-aspect finalization and checks here
  // currently there are non
}
