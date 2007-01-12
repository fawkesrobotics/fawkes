
/***************************************************************************
 *  thread_initializer.h - Fawkes thread initializer
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

#include <mainapp/thread_initializer.h>
#include <core/threading/thread.h>
#include <blackboard/blackboard.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>

/** @class FawkesThreadInitializer mainapp/thread_initializer.h
 * Fawkes Thread Initializer.
 * Initializes threads that are added to the thread manager if needed.
 * All aspects defined in the Fawkes tree are supported and properly
 * initialized such that guarantees are met.
 * @see Aspects
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard BlackBoard
 * @param config Configuration
 */
FawkesThreadInitializer::FawkesThreadInitializer(BlackBoard *blackboard,
						 Configuration *config)
{
  this->blackboard = blackboard;
  this->config     = config;
}


/** Initialize thread.
 * @param thread thread to initialize
 */
void
FawkesThreadInitializer::init(Thread *thread)
{
  // printf("Initializing thread %s\n", thread->name());

  BlockedTimingAspect *blocked_timing_thread;
  if ( (blocked_timing_thread = dynamic_cast<BlockedTimingAspect *>(thread)) != NULL ) {
    if ( thread->opmode() != Thread::OPMODE_WAITFORWAKEUP ) {
      throw CannotInitializeThreadException("Thread not in WAITFORWAKEUP mode (required for BlockedTimingAspect)");
    }
  }

  BlackBoardAspect *blackboard_thread;
  if ( (blackboard_thread = dynamic_cast<BlackBoardAspect *>(thread)) != NULL ) {
    blackboard_thread->setInterfaceManager( blackboard->getInterfaceManager() );
  }

  ConfigurableAspect *configurable_thread;
  if ( (configurable_thread = dynamic_cast<ConfigurableAspect *>(thread)) != NULL ) {
    configurable_thread->setConfiguration(config);
  }

}
