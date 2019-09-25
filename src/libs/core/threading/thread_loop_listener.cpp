
/***************************************************************************
 *  thread_loop_listener.cpp - thread notification listener interface
 *
 *  Created: Thu Feb 19 13:50:42 2015
 *  Copyright  2015 Till Hofmann
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

#include <core/threading/thread_loop_listener.h>

namespace fawkes {

/** @class ThreadLoopListener <core/threading/thread_loop_listener.h>
 * Thread loop listener interface.
 * A thread loop listener can be added to a thread to define pre and post loop
 * tasks, which are executed before and after every loop.
 *
 * @author Till Hofmann
 */

/** Virtual empty destructor. */
ThreadLoopListener::~ThreadLoopListener()
{
}

/** Empty stub for the pre loop function of the loop listener.
 * This function is called by the thread every time before loop() is called.
 * Provide a stub such that not every derived class must implement the function.
 * @param thread thread this loop listener belongs to
 */
void
ThreadLoopListener::pre_loop(Thread *thread)
{
}

/** Empty stub for the post loop function of the loop listener.
 * This function is called by the thread every time after loop() returned.
 * Provide a stub such that not every derived class must implement the function.
 * @param thread thread this loop listener belongs to
 */
void
ThreadLoopListener::post_loop(Thread *thread)
{
}

} // end namespace fawkes
