
/***************************************************************************
 *  mainloop.cpp - Fawkes MainLoopAspect initializer/finalizer
 *
 *  Created: Wed Nov 24 00:44:55 2010
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

#include <aspect/inifins/mainloop.h>
#include <aspect/mainloop.h>
#include <aspect/mainloop/employer.h>
#include <aspect/blocked_timing/executor.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MainLoopAspectIniFin <aspect/inifins/mainloop.h>
 * Initializer/finalizer for the MainLoopAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param employer main loop employer to register main loop to
 * @param btexec blocked timing executor to pass to thread
 */
MainLoopAspectIniFin::MainLoopAspectIniFin(MainLoopEmployer *employer,
					   BlockedTimingExecutor *btexec)
  : AspectIniFin("MainLoopAspect")
{
  __employer = employer;
  __btexec   = btexec;
}


void
MainLoopAspectIniFin::init(Thread *thread)
{
  MainLoopAspect *mainloop_thread;
  mainloop_thread = dynamic_cast<MainLoopAspect *>(thread);
  if (mainloop_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "MainLoopAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  if (thread->opmode() != Thread::OPMODE_WAITFORWAKEUP) {
    throw CannotInitializeThreadException("MainLoopAspect thread must operate "
					  "in wait-for-wakeup mode.");  
  }

  try {
    __mainloop_uc.add(mainloop_thread);
    mainloop_thread->init_MainLoopAspect(__btexec);
    thread->add_notification_listener(this);
  } catch (Exception &e) {
    CannotInitializeThreadException ce("Main loop thread failed to initialize");
    ce.append(e);
    throw ce;
  }
}


void
MainLoopAspectIniFin::finalize(Thread *thread)
{
  MainLoopAspect *mainloop_thread;
  mainloop_thread = dynamic_cast<MainLoopAspect *>(thread);
  if (mainloop_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "MainLoopAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  try {
    __employer->set_mainloop_thread(NULL);
    __mainloop_uc.remove(mainloop_thread);
  } catch (Exception &e) {
    CannotFinalizeThreadException ce("Failed to remove time source");
    ce.append(e);
    throw;
  }
}


bool
MainLoopAspectIniFin::thread_started(Thread *thread) throw()
{
  MainLoopAspect *mainloop_thread;
  if ( (mainloop_thread = dynamic_cast<MainLoopAspect *>(thread)) != NULL ) {
    try {
      __employer->set_mainloop_thread(thread);
    } catch (Exception &e) {
      //__logger->log_error("AspectIniFin", "Main loop thread started successfully "
      //		  "but could not add main loop thread's main loop");
    }
  }

  return false;
}


bool
MainLoopAspectIniFin::thread_init_failed(Thread *thread) throw()
{
  MainLoopAspect *mainloop_thread;
  if ( (mainloop_thread = dynamic_cast<MainLoopAspect *>(thread)) != NULL ) {
    try {
      __mainloop_uc.remove(mainloop_thread);
    } catch (Exception &e) {
      //__logger->log_error("AspectIniFin", "Failed to remove main loop from "
      //		  "uniqueness constraint on thread init fail of %s",
      //		  thread->name());
    }
  }

  try {
    finalize(thread);
  } catch (Exception &e) {
    /*
    __logger->log_error("AspectIniFin", "Initialization of thread '%s' failed, but "
			"the thread thread could not be internally finalized",
			thread->name());
    __logger->log_error("AspectIniFin", e);
    */
  }

  return false;
}


} // end namespace fawkes
