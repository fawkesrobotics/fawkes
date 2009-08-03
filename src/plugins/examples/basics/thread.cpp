
/***************************************************************************
 *  thread.cpp - Fawkes Example Plugin Thread
 *
 *  Generated: Wed Nov 22 17:13:57 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/examples/basics/thread.h>

#include <unistd.h>

using namespace std;
using namespace fawkes;

/** @class ExampleThread thread.h <plugins/examples/basics/thread.h>
 * Thread of example plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param hook hook to register this thread for
 * @param name thread name
 * @param modc modulo count, every modc iterations a message is printed to stdout
 */
ExampleThread::ExampleThread(BlockedTimingAspect::WakeupHook hook, const char *name,
			     unsigned int modc)
  : Thread(name, Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(hook)
{
  this->modc = modc;
  m = 0;
}


/** Destructor. */
ExampleThread::~ExampleThread()
{
  /** We cannot do the following:
   * logger->log_info("ExampleThread", "Destroying thread %s", name());
   *
   * The reason: We do not know if this thread has been successfully initialized.
   * It could be, that any other thread that is in the same thread list as this
   * thread failed to initialize, before the current thread has been initialized.
   * In this case the LoggingAspect has not been initialized and thus logger is
   * undefined and this would cause a fatal segfault.
   */
}


void
ExampleThread::init()
{

  /* Try this code to see a failing init in the middle of the thread list.
    if ( blockedTimingAspectHook() == WAKEUP_HOOK_WORLDSTATE ) {
      throw Exception("Boom!");
    }
  */
  logger->log_info("ExampleThread", "%s::init() called", name());
}


void
ExampleThread::finalize()
{
  logger->log_info("ExampleThread", "%s::finalize() called", name());
}


/** Thread loop.
 * If num iterations module modc is 0 print out messaege, otherwise do nothing.
 */
void
ExampleThread::loop()
{
  if ( (m % modc) == 0 ) {
    logger->log_info("ExampleThread", "ExampleThread %s called %u times", name(), m);
  }
  ++m;
  usleep(0);
}
