
/***************************************************************************
 *  thread_notification_listener.h - thread notification listener interface
 *
 *  Created: Fri Jun 08 15:59:20 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_THREADING_THREAD_NOTIFICATION_LISTENER_H_
#define __CORE_THREADING_THREAD_NOTIFICATION_LISTENER_H_

namespace fawkes {


class Thread;

class ThreadNotificationListener
{
 public:
  virtual ~ThreadNotificationListener();

  virtual bool thread_started(Thread *thread) throw()                    = 0;
  virtual bool thread_init_failed(Thread *thread) throw()                = 0;

};


} // end namespace fawkes

#endif
