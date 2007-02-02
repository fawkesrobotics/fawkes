
/***************************************************************************
 *  aspect_initializer.h - Fawkes Aspect initializer
 *
 *  Created: Tue Jan 30 13:36:42 2007
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

#include <aspect/initializer.h>

#include <core/threading/thread.h>
#include <blackboard/blackboard.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>

/** @class AspectInitializer aspect/initializer.h
 * Fawkes Aspect Initializer.
 * Initializes certain thread aspects.
 * All aspects defined in the Fawkes tree are supported and properly
 * initialized such that guarantees are met.
 * @see Aspects
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard BlackBoard
 * @param config Configuration
 */
AspectInitializer::AspectInitializer(BlackBoard *blackboard,
				     Configuration *config)
{
  this->blackboard = blackboard;
  this->config     = config;
}


/** Initialize thread.
 * @param thread thread to initialize
 */
void
AspectInitializer::init(Thread *thread)
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
    blackboard_thread->initBlackBoardAspect( blackboard->getInterfaceManager() );
  }

  ConfigurableAspect *configurable_thread;
  if ( (configurable_thread = dynamic_cast<ConfigurableAspect *>(thread)) != NULL ) {
    configurable_thread->initConfigurableAspect(config);
  }

}
