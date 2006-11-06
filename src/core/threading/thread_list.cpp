
/***************************************************************************
 *  thread_list.cpp - Thread list
 *
 *  Created: Tue Oct 31 18:20:59 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <core/threading/thread_list.h>
#include <core/threading/thread.h>

/** @class ThreadList core/threading/thread_list.h
 * List of threads.
 * This is a list of threads derived from stl::list. It features special
 * wakeup methods that will wakeup all threads in the list. The list can
 * and must be locked in iterator operations and when adding or deleting
 * elements from the list.
 * @author Tim Niemueller
 */

/** Constructor. */
ThreadList::ThreadList()
{
  clear();
}


/** Destructor. */
ThreadList::~ThreadList()
{
}


/** Wakeup all threads in list. */
void
ThreadList::wakeup()
{
  lock();
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->wakeup();
  }
  unlock();
}


/** Wakeup all threads in list and have them wait for the barrier.
 * @param barrier Barrier to wait for after loop
 */
void
ThreadList::wakeup(Barrier *barrier)
{
  lock();
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->wakeup(barrier);
  }
  unlock();
}
