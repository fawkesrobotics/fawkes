
/***************************************************************************
 *  thread.cpp - Fawkes WorldModel Plugin Thread
 *
 *  Created: Fri Jun 29 11:56:48 2007 (on flight to RoboCup 2007, Atlanta)
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/worldmodel/thread.h>

#include <interfaces/object.h>

#include <string>

using namespace std;

/** @class WorldModelThread <plugins/worldmodel/thread.h>
 * Main thread of worldmodel plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 */
WorldModelThread::WorldModelThread()
  : Thread("WorldModelThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}


/** Destructor. */
WorldModelThread::~WorldModelThread()
{
}


void
WorldModelThread::init()
{
  logger->log_info(name(), "init() called");
}


void
WorldModelThread::finalize()
{
  logger->log_info(name(), "finalize() called");
}


void
WorldModelThread::loop()
{
  /*
   * 1. Collect data (i.e. update local reader interfaces, collect data from network thread)
   * 2. Fusion network data (if available)
   * 3. Do calculations (to build up a consistent world model carry out general calculations
   *                     like the old best interceptor here at a central place
   * 4. Update written interfaces with new data
   *
   */
}
