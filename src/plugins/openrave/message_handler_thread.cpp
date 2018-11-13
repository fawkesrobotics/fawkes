
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
  or_thread_( or_thread ),
  if_openrave_( 0 )
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
    if_openrave_ = blackboard->open_for_writing<OpenRaveInterface>("OpenRAVE");
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not open OpenRAVE interface for writing. Er:%s", e.what());
  }
}


void
OpenRaveMessageHandlerThread::finalize()
{
  try {
    blackboard->close(if_openrave_);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close OpenRAVE interface");
  }
}


void
OpenRaveMessageHandlerThread::loop()
{
  while( !if_openrave_->msgq_empty() ) {
    if_openrave_->set_success(false);
    if_openrave_->set_final(false);
    Message *m = if_openrave_->msgq_first(m);
    if_openrave_->set_msgid(m->id());
    if_openrave_->write();

    if (if_openrave_->msgq_first_is<OpenRaveInterface::StartViewerMessage>()) {
      logger->log_debug(name(), "StartViewer message received");
      or_thread_->start_viewer();
      if_openrave_->set_success(true);
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::AddObjectMessage>()) {
      OpenRaveInterface::AddObjectMessage *msg = if_openrave_->msgq_first(msg);
      logger->log_debug(name(), "AddObject message received: name=%s, path=%s", msg->name(), msg->path());
      if( or_thread_->add_object(msg->name(), msg->path()) )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::DeleteObjectMessage>()) {
      OpenRaveInterface::DeleteObjectMessage *msg = if_openrave_->msgq_first(msg);
            logger->log_debug(name(), "DeleteObjectMessage received: name=%s", msg->name());
      if( or_thread_->delete_object(msg->name()) )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::DeleteAllObjectsMessage>()) {
      logger->log_debug(name(), "DeleteAllObjectsMessage received");
      if( or_thread_->delete_all_objects() )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::AttachObjectMessage>()) {
      OpenRaveInterface::AttachObjectMessage *msg = if_openrave_->msgq_first(msg);
      logger->log_debug(name(), "AttachObjectMessage received: name=%s, manip_name=%s", msg->name(), msg->manip_name());
      bool success = false;
      if( strcmp(msg->manip_name(), "") == 0 ) {
        success = or_thread_->attach_object(msg->name());
      } else {
        success = or_thread_->attach_object(msg->name(), msg->manip_name());
      }
      if_openrave_->set_success(success);
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::ReleaseObjectMessage>()) {
      OpenRaveInterface::ReleaseObjectMessage *msg = if_openrave_->msgq_first(msg);
      logger->log_debug(name(), "ReleaseObjectMessage received: name=%s", msg->name());
      if( or_thread_->release_object(msg->name()) )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::ReleaseAllObjectsMessage>()) {
      OpenRaveInterface::ReleaseAllObjectsMessage *msg = if_openrave_->msgq_first(msg);
      logger->log_debug(name(), "ReleaseAllObjectsMessage received");
      if( or_thread_->release_all_objects() )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::MoveObjectMessage>()) {
      OpenRaveInterface::MoveObjectMessage *msg = if_openrave_->msgq_first(msg);
      logger->log_debug(name(), "MoveObjectMessage received: name=%s, x=%f, y=%f, z=%f",
                                 msg->name(), msg->x(), msg->y(), msg->z());
      if( or_thread_->move_object(msg->name(), msg->x(), msg->y(), msg->z()) )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::RotateObjectQuatMessage>()) {
      OpenRaveInterface::RotateObjectQuatMessage *msg = if_openrave_->msgq_first(msg);
      logger->log_debug(name(), "RotateObjectQuatMessage received: name=%s, x=%f, y=%f, z=%f, w=%f",
                                 msg->name(), msg->x(), msg->y(), msg->z(), msg->w());
      if( or_thread_->rotate_object(msg->name(), msg->x(), msg->y(), msg->z(), msg->w()) )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::RotateObjectMessage>()) {
      OpenRaveInterface::RotateObjectMessage *msg = if_openrave_->msgq_first(msg);
      logger->log_debug(name(), "RotateObjectMessage received: name=%s, x=%f, y=%f, z=%f",
                                 msg->name(), msg->x(), msg->y(), msg->z());
      if( or_thread_->rotate_object(msg->name(), msg->x(), msg->y(), msg->z()) )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else if (if_openrave_->msgq_first_is<OpenRaveInterface::RenameObjectMessage>()) {
      OpenRaveInterface::RenameObjectMessage *msg = if_openrave_->msgq_first(msg);
      logger->log_debug(name(), "RenameObjectMessage received: name=%s, new_name=%s", msg->name(), msg->newName());
      if( or_thread_->rename_object(msg->name(), msg->newName()) )
        { if_openrave_->set_success(true); }
      if_openrave_->set_final(true);

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    if_openrave_->msgq_pop();
  }

  if_openrave_->write();
}
