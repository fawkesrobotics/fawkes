
/***************************************************************************
 *  thread_producer.cpp - Fawkes Thread producer Aspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:29:29 2010
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

#include <aspect/inifins/thread_producer.h>
#include <aspect/thread_producer.h>
#include <core/threading/thread_collector.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ThreadProducerAspectIniFin <aspect/inifins/thread_producer.h>
 * Initializer/finalizer for the ThreadProducerAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param collector collector instance to pass to threads
 */
ThreadProducerAspectIniFin::ThreadProducerAspectIniFin(ThreadCollector *collector)
  : AspectIniFin("ThreadProducerAspect")
{
  __collector = collector;
}


void
ThreadProducerAspectIniFin::init(Thread *thread)
{
  ThreadProducerAspect *thread_producer_thread;
  thread_producer_thread = dynamic_cast<ThreadProducerAspect *>(thread);
  if (thread_producer_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "ThreadProducerAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  thread_producer_thread->init_ThreadProducerAspect(__collector);
}


void
ThreadProducerAspectIniFin::finalize(Thread *thread)
{
}


} // end namespace fawkes
