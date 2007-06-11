
/***************************************************************************
 *  thread_notification_listener.h - thread notification listener interface
 *
 *  Created: Fri Jun 08 15:59:20 2007
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

#ifndef __CORE_THREADING_THREAD_NOTIFICATION_LISTENER_H_
#define __CORE_THREADING_THREAD_NOTIFICATION_LISTENER_H_

class Thread;

class ThreadNotificationListener
{
 public:
  virtual ~ThreadNotificationListener();

  virtual void thread_started(Thread *thread)                            = 0;
  virtual void thread_init_failed(Thread *thread)                        = 0;

};

#endif
