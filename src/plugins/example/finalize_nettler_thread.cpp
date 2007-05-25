
/***************************************************************************
 *  finalize_nettler_thread.cpp - Fawkes Example Plugin Finalize Nettler Thread
 *
 *  Created: Thu May 24 00:35:06 2007
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

#include <plugins/example/finalize_nettler_thread.h>

#include <unistd.h>

/** @class ExampleFinalizeNettlerThread plugins/example/finalize_nettler_thread.h
 * Thread of example plugin.
 * This thread does nothing but nagging once on finalize. On the first call to
 * prepare finalize it returns false that it cannot be finalized,
 * on the second time it allows finalization.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name thread name
 */
ExampleFinalizeNettlerThread::ExampleFinalizeNettlerThread(const char *name)
  : Thread(name, Thread::OPMODE_WAITFORWAKEUP)
{
  nagged = false;
}


/** Destructor. */
ExampleFinalizeNettlerThread::~ExampleFinalizeNettlerThread()
{
  logger->log_info("ExampleFinalizeNettlerThread", "Destroying thread %s", name());
}


/** Thread loop.
 * If num iterations module modc is 0 print out messaege, otherwise do nothing.
 */
void
ExampleFinalizeNettlerThread::loop()
{
}


void
ExampleFinalizeNettlerThread::init()
{
  logger->log_info("ExampleFinalizeNettlerThread", "init() called");
}


void
ExampleFinalizeNettlerThread::finalize()
{
  logger->log_info("ExampleFinalizeNettlerThread", "finalize() called");
}


bool
ExampleFinalizeNettlerThread::prepare_finalize_user()
{
  if ( nagged ) {
    logger->log_warn("ExampleFinalizeNettlerThread", "Allowing Finalization");
    return true;
  } else {
    logger->log_warn("ExampleFinalizeNettlerThread", "NOT allowing Finalization");
    nagged = true;
    return false;
  }
}
