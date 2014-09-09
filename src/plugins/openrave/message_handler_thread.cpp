
/***************************************************************************
 *  message_handler_thread.cpp - OpenRAVE Thread
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

#include "message_handler_thread.h"

#include <interfaces/OpenRaveInterface.h>

using namespace fawkes;

/** @class OpenRaveMessageHandlerThread "message_handler_thread.h"
 * OpenRAVE Thread.
 * This thread handles incoming messages for the OpenRaveInterface.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param or_thread OpenRaveThread, main thread. */
OpenRaveMessageHandlerThread::OpenRaveMessageHandlerThread(OpenRaveThread* or_thread)
  : Thread("OpenRaveMessageHandlerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
  __or_thread( or_thread ),
  __if_openrave( 0 )
{
}


/** Destructor. */
OpenRaveMessageHandlerThread::~OpenRaveMessageHandlerThread()
{
}


void
OpenRaveMessageHandlerThread::init()
{
  try {
    __if_openrave = blackboard->open_for_writing<OpenRaveInterface>("OpenRAVE");
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not open OpenRAVE interface for writing. Er:%s", e.what());
  }
}


void
OpenRaveMessageHandlerThread::finalize()
{
  try {
    blackboard->close(__if_openrave);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close OpenRAVE interface");
  }
}


void
OpenRaveMessageHandlerThread::loop()
{
  while( !__if_openrave->msgq_empty() ) {
    __if_openrave->set_success(false);
    __if_openrave->set_final(false);
    Message *m = __if_openrave->msgq_first(m);
    __if_openrave->set_msgid(m->id());
    __if_openrave->write();

    if (__if_openrave->msgq_first_is<OpenRaveInterface::StartViewerMessage>()) {
      logger->log_debug(name(), "StartViewer message received");
      __or_thread->start_viewer();
      __if_openrave->set_success(true);
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::AddObjectMessage>()) {
      OpenRaveInterface::AddObjectMessage *msg = __if_openrave->msgq_first(msg);
      logger->log_debug(name(), "AddObject message received: name=%s, path=%s", msg->name(), msg->path());
      if( __or_thread->add_object(msg->name(), msg->path()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::DeleteObjectMessage>()) {
      OpenRaveInterface::DeleteObjectMessage *msg = __if_openrave->msgq_first(msg);
            logger->log_debug(name(), "DeleteObjectMessage received: name=%s", msg->name());
      if( __or_thread->delete_object(msg->name()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::DeleteAllObjectsMessage>()) {
      logger->log_debug(name(), "DeleteAllObjectsMessage received");
      if( __or_thread->delete_all_objects() )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::AttachObjectMessage>()) {
      OpenRaveInterface::AttachObjectMessage *msg = __if_openrave->msgq_first(msg);
      logger->log_debug(name(), "AttachObjectMessage received: name=%s, manip_name=%s", msg->name(), msg->manip_name());
      bool success = false;
      if( strcmp(msg->manip_name(), "") == 0 ) {
        success = __or_thread->attach_object(msg->name());
      } else {
        success = __or_thread->attach_object(msg->name(), msg->manip_name());
      }
      __if_openrave->set_success(success);
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::ReleaseObjectMessage>()) {
      OpenRaveInterface::ReleaseObjectMessage *msg = __if_openrave->msgq_first(msg);
      logger->log_debug(name(), "ReleaseObjectMessage received: name=%s", msg->name());
      if( __or_thread->release_object(msg->name()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::ReleaseAllObjectsMessage>()) {
      OpenRaveInterface::ReleaseAllObjectsMessage *msg = __if_openrave->msgq_first(msg);
      logger->log_debug(name(), "ReleaseAllObjectsMessage received");
      if( __or_thread->release_all_objects() )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::MoveObjectMessage>()) {
      OpenRaveInterface::MoveObjectMessage *msg = __if_openrave->msgq_first(msg);
      logger->log_debug(name(), "MoveObjectMessage received: name=%s, x=%f, y=%f, z=%f",
                                 msg->name(), msg->x(), msg->y(), msg->z());
      if( __or_thread->move_object(msg->name(), msg->x(), msg->y(), msg->z()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::RotateObjectQuatMessage>()) {
      OpenRaveInterface::RotateObjectQuatMessage *msg = __if_openrave->msgq_first(msg);
      logger->log_debug(name(), "RotateObjectQuatMessage received: name=%s, x=%f, y=%f, z=%f, w=%f",
                                 msg->name(), msg->x(), msg->y(), msg->z(), msg->w());
      if( __or_thread->rotate_object(msg->name(), msg->x(), msg->y(), msg->z(), msg->w()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::RotateObjectMessage>()) {
      OpenRaveInterface::RotateObjectMessage *msg = __if_openrave->msgq_first(msg);
      logger->log_debug(name(), "RotateObjectMessage received: name=%s, x=%f, y=%f, z=%f",
                                 msg->name(), msg->x(), msg->y(), msg->z());
      if( __or_thread->rotate_object(msg->name(), msg->x(), msg->y(), msg->z()) )
        { __if_openrave->set_success(true); }
      __if_openrave->set_final(true);

    } else if (__if_openrave->msgq_first_is<OpenRaveInterface::RenameObjectMessage>()) {
      OpenRaveInterface::RenameObjectMessage *msg = __if_openrave->msgq_first(msg);
      logger->log_debug(name(), "RenameObjectMessage received: name=%s, new_name=%s", msg->name(), msg->newName());
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
