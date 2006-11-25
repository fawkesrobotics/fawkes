
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
#include <blackboard/bbthread.h>

/** @class FawkesThreadInitializer mainapp/thread_initializer.h
 * Fawkes Thread Initializer.
 * Initializes threads that are added to the thread manager if needed.
 * Some special thread types are recognized and appropriately
 * initialized. These types are:
 * - BlackBoardThread
 *   The interface manager is set for this type of threads. It is guaranteed
 *   that this happens before the thread is started.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard BlackBoard
 */
FawkesThreadInitializer::FawkesThreadInitializer(BlackBoard *blackboard)
{
  this->blackboard = blackboard;
}


/** Initialize thread.
 * @param thread thread to initialize
 */
void
FawkesThreadInitializer::init(Thread *thread)
{
  BlackBoardThread *bb_thread;
  if ( (bb_thread = dynamic_cast<BlackBoardThread *>(thread)) != NULL ) {
    bb_thread->setInterfaceManager( blackboard->getInterfaceManager() );
  }
}
