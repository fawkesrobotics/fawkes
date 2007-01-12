
/***************************************************************************
 *  thread_collector.pp - Fawkes thread collector interface
 *                       based on previous ThreadManager
 *
 *  Created: Thu Jan 11 17:53:44 2007
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <core/threading/thread_collector.h>

/** @class ThreadCollector core/threading/thread_collector.h
 * Thread collector.
 * This interface is meant to provide a central place where to put threads
 * to have them referenced somewhere. Threads my be added and removed at
 * will. The main purpose of the collector is to tear down all threads if
 * the collector is deleted thus providing a clean exit.
 *
 * Additional functionality and aspect-specific behavior may be added in
 * implementations.
 *
 * @author Tim Niemueller
 *
 * @fn void ThreadCollector::add(ThreadList &tl) = 0
 * Add multiple threads.
 * Adds all the threads in the list to the thread list. Implementations may
 * throw an exception if this fails for whatever reason, read implementation
 * documentation for details. The operation shall be atomic, either all
 * threads are added successfully or none is added at all.
 * @param tl list of threads to add
 *
 * @fn void ThreadCollector::add(Thread *t) = 0
 * Add single thread.
 * Adds the single thread to the internal (implementation specific) thread
 * list.
 * @param t thread to add
 *
 * @fn ThreadCollector::remove(ThreadList &tl) = 0
 * Remove multiple threads.
 * Remove all threads in the thread list from this collector. If there is
 * a thread in the supplied thread list that has never been collected no
 * error shall be thrown but this just be silently ignored.
 * @param tl list of threads to remove
 *
 * @fn ThreadCollector::remove(Thread *t) = 0
 * Remove single thread.
 * Remove the thread from the internal thread list. If the thread has never
 * been collected no error shall be thrown but just be silently ignored.
 * @param t Thread to remove.
 */

/** Empty virtual destructor. */
ThreadCollector::~ThreadCollector()
{
}
