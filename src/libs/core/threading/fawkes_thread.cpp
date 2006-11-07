
/***************************************************************************
 *  fawkes_thread.cpp - Fawkes thread
 *
 *  Created: Thu Nov  5 18:13:48 2006
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

#include <core/threading/fawkes_thread.h>


/** @class FawkesThread core/threading/fawkes_thread.h
 * Thread with wakup hooks.
 * This is the base class for threads that are run within Fawkes and that
 * work inside the Fawkes main lopp.
 * The thread will be called at a pre-defined hook for best integration into
 * the Fawkes processing loop.
 *
 * Note that run() has been hidden in FawkesThread. You have to override
 * loop() to implement the desired behaviour.
 *
 * Fawkes threads operate in wait-for-wakeup mode (see Thread).
 * If you want a continuously running thread you most likely want to have
 * @code
 * YourBBThread::YourBBThread()
 *   : FawkesThread()
 * {
 *   // ...
 * }
 *
 * FawkesThread::WakeupHook
 * YourBBThread::hook()
 * {
 *   return WAKEUP_HOOK_NONE;
 * }
 * @endcode
 * in your class.
 * @author Tim Niemueller
 * @ingroup Threading
 * @see Thread
 */

/** @fn FawkesThread::WakeupHook FawkesThread::hook() const = 0
 * Override this method to return the desired wakeup hook.
 */

/** @fn const char * FawkesThread::name() const = 0
 * Override this method to return the name of the thread. Note that the
 * name must be globally unique! In general this should not be a problem
 * if you give it a descriptive name closely related to the class name.
 */


/** Constructor. */
FawkesThread::FawkesThread()
  : Thread(Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
FawkesThread::~FawkesThread()
{
}
