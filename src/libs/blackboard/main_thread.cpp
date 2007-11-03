
/***************************************************************************
 *  main_thread.cpp - BlackBoard main thread
 *
 *  Generated: Tue Oct 31 18:37:49 2006
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

#include <blackboard/main_thread.h>
#include <blackboard/interface_manager.h>


/** @class BlackBoardMainThread blackboard/main_thread.h
 * Main thread of BlackBoard.
 * The main thread will create the shared memory segment and instantiate the
 * interface and message managers.
 */

/** Constructor. */
BlackBoardMainThread::BlackBoardMainThread()
  : Thread("BlackBoardMainThread", Thread::OPMODE_WAITFORWAKEUP)
{
  im = new BlackBoardInterfaceManager(/* master */ true);
}

/** Destructor. */
BlackBoardMainThread::~BlackBoardMainThread()
{
  delete im;
}


/** Get interface manager.
 * This is a special property of this thread to get access to the interface
 * manager.
 * @return interface manager
 */
BlackBoardInterfaceManager *
BlackBoardMainThread::interface_manager() const
{
  return im;
}


/** Loop.
 * Nothing is done.
 */
void
BlackBoardMainThread::loop()
{
}
