 
/***************************************************************************
 *  on_message_waker.h - wake a thread whenever a message is received
 *
 *  Created: Thu Dec 06 12:05:14 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/utils/on_message_waker.h>
#include <blackboard/blackboard.h>
#include <interface/interface.h>
#include <interface/message.h>
#include <core/threading/thread.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BlackBoardOnMessageWaker <blackboard/utils/on_message_waker.h>
 * Wake threads on receiving a blackboard message.
 * This utility class registers as a BlackBoardInterfaceListener and
 * if a message is received it wakes the given thread.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param bb blackboard to register with
 * @param interface Interface to monitor for incoming messages
 * @param thread thread to wake
 */
BlackBoardOnMessageWaker::BlackBoardOnMessageWaker(BlackBoard *bb,
						   Interface *interface,
						   Thread *thread)
  : BlackBoardInterfaceListener("OnMessageWaker[%s]", interface->uid()),
    bb_(bb), thread_(thread)
{
  bbil_add_message_interface(interface);
  bb_->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
}


/** Destructor.
 * Unregisters from the blackboard.
 */
BlackBoardOnMessageWaker::~BlackBoardOnMessageWaker()
{
  bb_->unregister_listener(this);
}


bool
BlackBoardOnMessageWaker::bb_interface_message_received(Interface *interface,
							Message *message) throw()
{
  try {
    interface->msgq_append(message);
    thread_->wakeup();
    return false;
  } catch (Exception &e) {
    return true;
  }
}


} // end namespace fawkes

