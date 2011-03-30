
/***************************************************************************
 *  or_message_handler_thread.cpp - OpenRAVE Thread
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "or_message_handler_thread.h"

#include <interfaces/OpenRAVEInterface.h>

using namespace fawkes;

/** @class OpenRAVEMessageHandlerThread "or_message_handler_thread.h"
 * OpenRAVE Thread.
 * This thread handles incoming messages for the OpenRAVEInterface.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param or_thread OpenRAVEThread, main thread. */
OpenRAVEMessageHandlerThread::OpenRAVEMessageHandlerThread(OpenRAVEThread* or_thread)
  : Thread("OpenRAVEMessageHandlerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
  __or_thread( or_thread ),
  __if_openrave( 0 )
{
}


/** Destructor. */
OpenRAVEMessageHandlerThread::~OpenRAVEMessageHandlerThread()
{
}


void
OpenRAVEMessageHandlerThread::init()
{
  try {
    __if_openrave = blackboard->open_for_writing<OpenRAVEInterface>("OpenRAVE");
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not open OpenRAVE interface for writing. Er:%s", e.what());
  }
}


void
OpenRAVEMessageHandlerThread::finalize()
{
  try {
    blackboard->close(__if_openrave);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close OpenRAVE interface");
  }
}


void
OpenRAVEMessageHandlerThread::loop()
{
  while( !__if_openrave->msgq_empty() ) {
    __if_openrave->set_success(false);
    __if_openrave->set_final(false);
    Message *m = __if_openrave->msgq_first(m);
    __if_openrave->set_msgid(m->id());
    __if_openrave->write();

    if (__if_openrave->msgq_first_is<OpenRAVEInterface::AddObjectMessage>()) {
      OpenRAVEInterface::AddObjectMessage *msg = __if_openrave->msgq_first(msg);
      if( __or_thread->add_object(msg->name(), msg->path()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRAVEInterface::DeleteObjectMessage>()) {
      OpenRAVEInterface::DeleteObjectMessage *msg = __if_openrave->msgq_first(msg);
      if( __or_thread->delete_object(msg->name()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRAVEInterface::MoveObjectMessage>()) {
      OpenRAVEInterface::MoveObjectMessage *msg = __if_openrave->msgq_first(msg);
      if( __or_thread->move_object(msg->name(), msg->x(), msg->y(), msg->z()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRAVEInterface::RotateObjectMessage>()) {
      OpenRAVEInterface::RotateObjectMessage *msg = __if_openrave->msgq_first(msg);
      if( __or_thread->rotate_object(msg->name(), msg->x(), msg->y(), msg->z()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRAVEInterface::RenameObjectMessage>()) {
      OpenRAVEInterface::RenameObjectMessage *msg = __if_openrave->msgq_first(msg);
      if( __or_thread->rename_object(msg->name(), msg->newName()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __if_openrave->msgq_pop();
  }

  __if_openrave->write();
}