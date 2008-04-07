
/***************************************************************************
 *  thread_notification_listener.cpp - thread notification listener interface
 *
 *  Created: Fri Jun 08 16:39:20 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <core/threading/thread_notification_listener.h>

/** @class ThreadNotificationListener <core/threading/thread_notification_listener.h>
 * Thread notification listener interface.
 * A thread notification listener can be added to a thread to be notified of a
 * successful startup of if the initialization fails (and hence the thread is
 * never started).
 *
 * @author Tim Niemueller
 *
 * @fn void ThreadNotificationListener::thread_started(Thread *thread)
 * Thread started successfully.
 * This is called by the thread itself when the thread started successfully.
 * @param thread thread that started successfully
 *
 * @fn void ThreadNotificationListener::thread_init_failed(Thread *thread)
 * Thread initialization failed.
 * This method is called by ThreadList if one of the threads in the list failed
 * to initialize. This is not necessarily the thread that you registered the
 * notification for. However, the argument is always the thread that you
 * registered for, no matter which thread in the list failed.
 * @param thread thread that you registered for
 */

/** Virtual empty destructor. */
ThreadNotificationListener::~ThreadNotificationListener()
{
}
