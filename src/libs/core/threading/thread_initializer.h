
/***************************************************************************
 *  thread_initializer.h - Thread initializer interface
 *
 *  Created: Mon Nov 20 00:52:18 2006
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

#ifndef __CORE_THREADING_THREAD_INITIALIZER_H_
#define __CORE_THREADING_THREAD_INITIALIZER_H_

class Thread;

/** @class ThreadInitializer core/threading/thread_initializer.h
 * Thread initializer interface.
 * This interface is used by the ThreadManager. The init() method is called
 * for each added thread. If there are any special needs that have to be
 * initialized before the thread is started on the given real classes of
 * the thread this is the way to do it. See Fawkes main application for
 * an example.
 *
 * @author Tim Niemueller
 */

class ThreadInitializer
{
 public:
  /** Virtual empty destructor. */
  virtual ~ThreadInitializer() {}

  /** Thread initializer method.
   * This method is called by the ThreadManager for each newly added Thread.
   * @param thread thread to initialize.
   */
  virtual void init(Thread *thread) = 0;
};

#endif
