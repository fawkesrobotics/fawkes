
/***************************************************************************
 *  messag_handler_thread.cpp - Colli Message Handler Thread
 *
 *  Created: Thu Oct 17 16:58:00 2013
 *  Copyright  2013  AllemaniACs
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

#include <interfaces/MotorInterface.h>
#include <interfaces/NavigatorInterface.h>

#include <utils/math/coord.h>

#include <string>

using namespace fawkes;
using namespace std;

/** @class ColliMessageHandlerThread "message_handler_thread.h"
 * This thread receives the messages of the main NavigatorInterface and
 * transforms them into appropriate commands/targets for the ColliThread.
 * This also includes setting the DriveMode etc (for now; should be more
 * adjustable in future releases)
 */

/** Constructor. */
ColliMessageHandlerThread::ColliMessageHandlerThread()
  : Thread("ColliMessageHandlerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

/** Desctructor. */
ColliMessageHandlerThread::~ColliMessageHandlerThread()
{
}

void
ColliMessageHandlerThread::init()
{
  std::string cfg_prefix = "/plugins/colli/";
  security_distance_ = config->get_float((cfg_prefix + "security_distance").c_str());
  max_velocity_      = config->get_float((cfg_prefix + "max_velocity").c_str());
  escaping_enabled_  = config->get_bool((cfg_prefix + "escaping_enabled").c_str());

  cfg_iface_navi_       = config->get_string((cfg_prefix + "interface/navigator").c_str());
  cfg_iface_motor_      = config->get_string((cfg_prefix + "interface/motor").c_str());

  if_navi_ = blackboard->open_for_writing<NavigatorInterface>(cfg_iface_navi_.c_str());
  if_motor_ = blackboard->open_for_reading<MotorInterface>(cfg_iface_motor_.c_str());

  if_colli_data_ = blackboard->open_for_reading<NavigatorInterface>("Colli data");
  if_colli_target_ = blackboard->open_for_writing<NavigatorInterface>("Colli target");
}


void
ColliMessageHandlerThread::finalize()
{
  blackboard->close( if_navi_ );
  blackboard->close( if_colli_data_ );
  blackboard->close( if_colli_target_ );
  blackboard->close( if_motor_ );
}

void
ColliMessageHandlerThread::loop()
{
  // update interfaces
  if_colli_data_->read();
  if_motor_->read();

  if( if_colli_data_->changed() )
    if_navi_->set_final(colli_final());

  // process interface messages
  while( !if_navi_->msgq_empty() ) {
    if (if_navi_->msgq_first_is<NavigatorInterface::StopMessage>()) {
      logger->log_debug(name(), "StopMessage received");
      colli_stop();

    } else if (if_navi_->msgq_first_is<NavigatorInterface::CartesianGotoMessage>()) {
      NavigatorInterface::CartesianGotoMessage *msg = if_navi_->msgq_first<NavigatorInterface::CartesianGotoMessage>();
      logger->log_debug(name(), "CartesianGotoMessage received, x:%f  y:%f  ori:%f", msg->x(), msg->y(), msg->orientation());
      // Converts from Fawkes Coord Sys -> RCSoftX Coord Sys, hence X_f = X_r, Y_f = -Y_r, Ori_f = -Ori_r
      if_navi_->set_msgid(msg->id());
      if_navi_->set_dest_x(msg->x());
      if_navi_->set_dest_y(msg->y());
      if_navi_->set_dest_ori(msg->orientation());
      if_navi_->set_final(false);

      colli_relgoto(msg->x(), msg->y(), msg->orientation(),
                    max_velocity_, escaping_enabled_,
                    security_distance_);

      //~ __colli_cmd_sent = true;

      //~ if (__pathplan_mode != PP_GOTO_NONE) {
        //~ m_pMonaco->pathplan_release_colli_control();
        //~ __pathplan_mode = PP_GOTO_NONE;
      //~ }

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::PolarGotoMessage>() ) {
      NavigatorInterface::PolarGotoMessage *msg = if_navi_->msgq_first<NavigatorInterface::PolarGotoMessage>();
      logger->log_debug(name(), "PolarGotoMessage received, phi:%f  dist:%f", msg->phi(), msg->dist());
      // Converts from Fawkes Coord Sys -> RCSoftX Coord Sys, hence D_f = D_r, Phi_f = - Phi_r, Ori_f = Ori_r

      float cart_x = 0, cart_y = 0;
      polar2cart2d(msg->phi(), msg->dist(), &cart_x, &cart_y);

      colli_relgoto(cart_x, cart_y, msg->orientation(),
                    max_velocity_, escaping_enabled_,
                    security_distance_);

      if_navi_->set_msgid(msg->id());
      if_navi_->set_dest_x(cart_x);
      if_navi_->set_dest_y(cart_y);
      if_navi_->set_dest_ori(msg->orientation());
      if_navi_->set_dest_dist(msg->dist());
      if_navi_->set_final(false);

      //~ __colli_cmd_sent = true;

      //~ if (__pathplan_mode != PP_GOTO_NONE) {
        //~ m_pMonaco->pathplan_release_colli_control();
        //~ __pathplan_mode = PP_GOTO_NONE;
      //~ }

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetMaxVelocityMessage>() ) {
      NavigatorInterface::SetMaxVelocityMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetMaxVelocityMessage>();

      logger->log_debug(name(), "setting max velocity to %f", msg->max_velocity());
      max_velocity_ = msg->max_velocity();
      if_navi_->set_max_velocity(max_velocity_);

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetEscapingMessage>() ) {
      NavigatorInterface::SetEscapingMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetEscapingMessage>();

      logger->log_debug(name(), "setting escaping allowed to %u", msg->is_escaping_enabled());
      escaping_enabled_ = msg->is_escaping_enabled();
      if_navi_->set_escaping_enabled(escaping_enabled_);

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetSecurityDistanceMessage>() ) {
      NavigatorInterface::SetSecurityDistanceMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetSecurityDistanceMessage>();

      logger->log_debug(name(), "setting security distance to %f", msg->security_distance());
      security_distance_ = msg->security_distance();
      if_navi_->set_security_distance(security_distance_);

    } else {
      logger->log_debug(name(), "Ignoring unhandled Navigator message");
    }

    if_navi_->msgq_pop();
  }
  if_navi_->write();

}



bool
ColliMessageHandlerThread::colli_final()
{
  return if_colli_data_->is_final();
  // RCSoftX had more. Full code(from libmonaco and navigator_server) :
  //
  // return (if_colli_data_->is_final() || m_pNavServer->isColliFinal() );
  // bbClients::Navigator_Server::isColliFinal()
  // {
  //  bool alive = (bool)m_ColliFeedback.GetValue( 8 /* magic value for alive index */ );
  //  bool final = (bool)m_ColliFeedback.GetValue( 0 /* magic value for final index */ );
  //  Timestamp cf(m_itsColliFeedback_sec.GetValue(), m_itsColliFeedback_usec.GetValue());
  //  Timestamp tp(m_itsTargetPoint_sec.GetValue(), m_itsTargetPoint_usec.GetValue());

  //  return alive && final && ( (cf - tp) == 0);
  // }
}

void
ColliMessageHandlerThread::colli_stop()
{
  if_colli_target_->set_dest_x( if_motor_->odometry_position_x() );
  if_colli_target_->set_dest_y( if_motor_->odometry_position_y() );
  if_colli_target_->set_dest_ori( if_motor_->odometry_orientation() );

  if_colli_target_->set_drive_mode( NavigatorInterface::MovingNotAllowed );
  if_colli_target_->set_stop_at_target( true );
  if_colli_target_->set_orient_at_target( false );
  if_colli_target_->set_escaping_enabled( false );

  if_colli_target_->write();
}

void
ColliMessageHandlerThread::colli_relgoto(float x, float y, float ori, float max_speed,
                                         bool escape_allowed, float security_distance,
                                         NavigatorInterface::DriveMode drivemode)
{
  // my Pose in motor coordinates
  float colliTargetO = if_motor_->odometry_orientation();

  // coord transformation: relative target -> (global) motor coordinates
  float colliTargetX = if_motor_->odometry_position_x()
                       + x * cos( colliTargetO )
                       - y * sin( colliTargetO );
  float colliTargetY = if_motor_->odometry_position_y()
                       + x * sin( colliTargetO )
                       + y * cos( colliTargetO );

  colliTargetO += ori;

  if_colli_target_->set_dest_x( colliTargetX );
  if_colli_target_->set_dest_y( colliTargetY );
  if_colli_target_->set_dest_ori( colliTargetO );

  if_colli_target_->set_drive_mode( drivemode );
  if_colli_target_->set_stop_at_target( true );
  if_colli_target_->set_orient_at_target( true );
  if_colli_target_->set_escaping_enabled( escape_allowed );

  if_colli_target_->write();
}
