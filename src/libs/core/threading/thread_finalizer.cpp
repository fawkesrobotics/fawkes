
/***************************************************************************
 *  thread_finalizer.cpp - Thread finalizer interface
 *
 *  Created: Fri Jan 12 13:29:29 2007
 *  Copyright  2006-2007 Tim Niemueller [www.niemueller.de]
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

#include <core/threading/thread_finalizer.h>

namespace fawkes {

/** @class CannotFinalizeThreadException core/threading/thread_finalizer.h
 * Thread cannot be finalized.
 * Thrown if a thread could not be finalized for whatever reason.
 * @ingroup Exceptions
 */

/** Constructor.
 * @param format message format (reason or symptom of failure)
 */
CannotFinalizeThreadException::CannotFinalizeThreadException(const char *format, ...)
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}

/** Constructor.
 * @param e exception to copy messages from
 */
CannotFinalizeThreadException::CannotFinalizeThreadException(Exception &e)
  : Exception(e)
{
}


/** @class ThreadFinalizer core/threading/thread_finalizer.h
 * Thread finalizer interface.
 * This interface is used by the ThreadManager. The finalize() method is called
 * for each thread that is about to be removed. If there are any special needs
 * that have to be finalized before the thread is stopped on the given real
 * classes of the thread this is the way to do it.
 *
 * The finalizer may abort the stopping of a thread by throwing a
 * CannotFinalizeThreadException. This can for example be used if you have two
 * threads A and B. A depends on B in that B is needed for A to run properly.
 * Now both threads are running and then B is called to stop. The finalize will
 * call threads B finalize() method, which fails (because it knows about the
 * dependency of A for example by some kind of register pattern). This tells the
 * thread manager not to stop B, because this would break A.
 *
 * See Fawkes main application for
 * an example.
 * @author Tim Niemueller
 *
 * @fn bool ThreadFinalizer::prepare_finalize(Thread *thread) = 0
 * Prepare finalization of a thread.
 * If the finalizer needs to do anything to prepare a maybe following finalize()
 * can do so here. This is also the only place where it proclaim that finalizing
 * the given thread at the given time is unsafe.
 * The finalizer shall NOT call Thread::prepare_finalize().
 * @param thread thread to prepare finalization for
 * @return true if nothing prevents finalization, false otherwise
 * @see Thread::prepare_finalize()
 * 
 *
 * @fn void ThreadFinalizer::finalize(Thread *thread) = 0
 * Finalize a thread.
 * This method is called by the ThreadManager for each Thread that is to be
 * stopped and removed from the list of running threads.
 * The finalizer shall NOT call Thread::finalize().
 * @param thread thread to finalize.
 * @exception CannotFinalizeThread thrown if thread can for not b finalized
 * @see Thread::finalize()
 */

/** Virtual empty destructor. */
ThreadFinalizer::~ThreadFinalizer()
{
}


} // end namespace fawkes
