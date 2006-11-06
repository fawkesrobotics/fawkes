
/***************************************************************************
 *  bbthread.cpp - BlackBoard thread
 *
 *  Created: Thu Nov  2 14:59:48 2006
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

#include <blackboard/bbthread.h>
#include <core/exceptions/software.h>

/** @class BlackBoardThread blackboard/bbthread.h
 * Thread with access to the BlackBoard.
 * This is the base class for threads that need access to the BlackBoard.
 * It is guaranteed that if used properly from within plugins that
 * setInterfaceManager() is called before the thred is started.
 * The thread will also be called at defined hook for best integration into
 * the Fawkes processing loop (see FawkesThread).
 * @see FawkesThread
 * @ingroup Threading
 * @author Tim Niemueller
 */

/** @var BlackBoardInterfaceManager *  BlackBoardThread::interface_manager
 * This is the interface manager you can use to interact with the
 * BlackBoard.
 */


/** Constructor. */
BlackBoardThread::BlackBoardThread()
  : FawkesThread()
{
}


/** Destructor. */
BlackBoardThread::~BlackBoardThread()
{
}


/** Set the interface manager.
 * It is guaranteed that this is called for a BlackBoardThread before start
 * is called (when running regularly inside Fawkes).
 * @param im interface manager to use
 */
void
BlackBoardThread::setInterfaceManager(BlackBoardInterfaceManager *im)
{
  interface_manager = im;
}
