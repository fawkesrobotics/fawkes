
/***************************************************************************
 *  thread_initializer.cpp - Thread initializer interface
 *
 *  Created: Fri Jan 12 13:29:29 2007
 *  Copyright  2006-2007 Tim Niemueller [www.niemueller.de]
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

#include <core/threading/thread_initializer.h>

/** @class CannotInitializeThreadException core/threading/thread_initializer.h
 * Thread cannot be initialized.
 * Thrown if a thread could not be initialized for whatever reason.
 * @ingroup Exceptions
 */

/** Constructor.
 * @param msg message (reason or symptom of failure)
 */
CannotInitializeThreadException::CannotInitializeThreadException(const char *msg)
  : Exception(msg)
{
}


/** @class ThreadInitializer core/threading/thread_initializer.h
 * Thread initializer interface.
 * This interface is used by the ThreadManager. The init() method is called
 * for each added thread. If there are any special needs that have to be
 * initialized before the thread is started on the given real classes of
 * the thread this is the way to do it. See Fawkes main application for
 * an example.
 * @author Tim Niemueller
 *
 * @fn void ThreadInitializer::init(Thread *thread) = 0
 * This method is called by the ThreadManager for each newly added Thread.
 * @param thread thread to initialize.
 * @exception CannotInitializeThread thrown if thread can for not be
 * initialized
 */

/** Virtual empty destructor. */
ThreadInitializer::~ThreadInitializer()
{
}
