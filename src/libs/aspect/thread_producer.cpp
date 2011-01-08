
/***************************************************************************
 *  thread_producer.cpp - Thread producer aspect for Fawkes
 *
 *  Created: Tue Nov 20 11:26:24 2007
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

#include <aspect/thread_producer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ThreadProducerAspect <aspect/thread_producer.h>
 * Aspect for thread producing threads.
 * Some threads have to be started by a plugin after it is loaded. Thus
 * they produce threads while they run. They may also stop this thread
 * at any one time. To have all these threads registered with a central
 * instance for easier instrumentation and performance assessment these
 * threads should be registered with a central thread collector.
 *
 * Additionally the threads that are produced can have aspects that are
 * then initialized by the thread collector (if running inside Fawkes).
 * Note that initializing an aspect may fail and then an exception is
 * thrown to indicate the error. You have to catch this exception and
 * you may never start a thread in that case or unpredictable behavior
 * will happen.
 *
 * This is possible with the ThreadProducerAspect. With this aspect you
 * get access to a thread collector instance to register threads with.
 *
 * Remember to unregister the produced threads if they are cancelled,
 * joined or even deleted!
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** @var ThreadCollector * ThreadProducerAspect::thread_collector
 * Thread collector.
 * Use this thread collector to register/unregister threads as they are
 * created/deleted. It is set when the thread starts.
 */

/** Constructor. */
ThreadProducerAspect::ThreadProducerAspect()
{
  add_aspect("ThreadProducerAspect");
}

/** Virtual empty destructor. */
ThreadProducerAspect::~ThreadProducerAspect()
{
}


/** Init thread producer aspect.
 * This set the thread collector.
 * It is guaranteed that this is called for a thread with the ThreadProducerAspect
 * before start is called (when running regularly inside Fawkes).
 * @param collector thread collector
 */
void
ThreadProducerAspect::init_ThreadProducerAspect(ThreadCollector *collector)
{
  thread_collector = collector;
}

} // end namespace fawkes
