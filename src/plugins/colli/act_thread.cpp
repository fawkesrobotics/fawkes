
/***************************************************************************
 *  act_thread.cpp - Colli Act Thread
 *
 *  Created: Thu Oct 17 16:58:00 2013
 *  Copyright  2013-2014  Bahram Maleki-Fard
 *                  2014  Tobias Neumann
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

#include "act_thread.h"
#include "colli_thread.h"

#include <interfaces/NavigatorInterface.h>

#include <utils/math/coord.h>

#include <string>

using namespace fawkes;
using namespace std;

/** @class ColliActThread "act_thread.h"
 * This thread hooks onto Fawkes main loop at the ACT hook. It is
 * resoponsible for receiving the messages of the main NavigatorInterface
 * and sending commands to the colli.
 */

/** Constructor.
 * @param colli_thread The continuous colli thread that handles the colli behavior
 */
ColliActThread::ColliActThread(ColliThread* colli_thread)
  : Thread("ColliActThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    thread_colli_( colli_thread )
{
}

/** Desctructor. */
ColliActThread::~ColliActThread()
{
}

void
ColliActThread::init()
{
  std::string cfg_prefix = "/plugins/colli/";
  cfg_security_distance_ = config->get_float((cfg_prefix + "security_distance").c_str());
  cfg_max_velocity_      = config->get_float((cfg_prefix + "max_velocity").c_str());
  cfg_max_rotation_      = config->get_float((cfg_prefix + "max_rotation").c_str());
  cfg_escaping_enabled_  = config->get_bool((cfg_prefix + "escaping_enabled").c_str());
  cfg_stop_at_target_    = config->get_bool((cfg_prefix + "stop_at_target").c_str());

  std::string cfg_orient_mode = config->get_string((cfg_prefix + "orient_mode/default").c_str());
  if ( cfg_orient_mode == "OrientAtTarget" ) {
    cfg_orient_mode_ = fawkes::NavigatorInterface::OrientationMode::OrientAtTarget;
  } else if ( cfg_orient_mode == "OrientDuringTravel" ) {
    cfg_orient_mode_ = fawkes::NavigatorInterface::OrientationMode::OrientDuringTravel;
  } else {
    cfg_orient_mode_ = fawkes::NavigatorInterface::OrientationMode::OrientAtTarget;
    throw fawkes::Exception("Default orient_mode is unknown");
  }

  std::string cfg_drive_mode = config->get_string((cfg_prefix + "drive_mode/default").c_str());
  if (        cfg_drive_mode == "MovingNotAllowed" ) {
    cfg_drive_mode_ = NavigatorInterface::MovingNotAllowed;
  } else if ( cfg_drive_mode == "Forward" ) {
    cfg_drive_mode_ = NavigatorInterface::Forward;
  } else if ( cfg_drive_mode == "AllowBackward" ) {
    cfg_drive_mode_ = NavigatorInterface::AllowBackward;
  } else if ( cfg_drive_mode == "Backward" ) {
    cfg_drive_mode_ = NavigatorInterface::Backward;
  } else if ( cfg_drive_mode == "ESCAPE" ) {
    cfg_drive_mode_ = NavigatorInterface::ESCAPE;
  } else {
    cfg_drive_mode_ = NavigatorInterface::MovingNotAllowed;
    throw fawkes::Exception("Default drive_mode is unknown");
  }

  logger->log_debug(name(), "Default drive_mode: %i (%s)", cfg_drive_mode_, if_navi_->tostring_DriveMode(cfg_drive_mode_));

  cfg_iface_navi_       = config->get_string((cfg_prefix + "interface/navigator").c_str());

  cfg_frame_odom_       = config->get_string((cfg_prefix + "frame/odometry").c_str());

  if_navi_ = blackboard->open_for_writing<NavigatorInterface>(cfg_iface_navi_.c_str());

  if_navi_->set_max_velocity(cfg_max_velocity_);
  if_navi_->set_max_rotation(cfg_max_rotation_);
  if_navi_->set_escaping_enabled(cfg_escaping_enabled_);
  if_navi_->set_security_distance(cfg_security_distance_);
  if_navi_->set_stop_at_target(cfg_stop_at_target_);
  if_navi_->set_orientation_mode(cfg_orient_mode_);
  if_navi_->set_drive_mode(cfg_drive_mode_);
  if_navi_->set_final(true);
  if_navi_->write();
}

void
ColliActThread::finalize()
{
  blackboard->close( if_navi_ );
}

void
ColliActThread::loop()
{
  // update interfaces
  if_navi_->set_final(colli_final());

  // process interface messages
  Message* motion_msg = NULL;
  while( !if_navi_->msgq_empty() ) {
    if (if_navi_->msgq_first_is<NavigatorInterface::StopMessage>()) {
      if( motion_msg )
        motion_msg->unref();
      motion_msg = if_navi_->msgq_first<NavigatorInterface::StopMessage>();
      motion_msg->ref();

    } else if (if_navi_->msgq_first_is<NavigatorInterface::CartesianGotoMessage>()) {
      if( motion_msg )
        motion_msg->unref();
      motion_msg = if_navi_->msgq_first<NavigatorInterface::CartesianGotoMessage>();
      motion_msg->ref();

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::PolarGotoMessage>() ) {
      if( motion_msg )
        motion_msg->unref();
      motion_msg = if_navi_->msgq_first<NavigatorInterface::PolarGotoMessage>();
      motion_msg->ref();

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetMaxVelocityMessage>() ) {
      NavigatorInterface::SetMaxVelocityMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetMaxVelocityMessage>();

      logger->log_debug(name(), "setting max velocity to %f", msg->max_velocity());
      if_navi_->set_max_velocity(msg->max_velocity());

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetMaxRotationMessage>() ) {
      NavigatorInterface::SetMaxRotationMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetMaxRotationMessage>();

      logger->log_debug(name(), "setting max rotation velocity to %f", msg->max_rotation());
      if_navi_->set_max_rotation(msg->max_rotation());

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetEscapingMessage>() ) {
      NavigatorInterface::SetEscapingMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetEscapingMessage>();

      logger->log_debug(name(), "setting escaping allowed to %u", msg->is_escaping_enabled());
      if_navi_->set_escaping_enabled(msg->is_escaping_enabled());

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetSecurityDistanceMessage>() ) {
      NavigatorInterface::SetSecurityDistanceMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetSecurityDistanceMessage>();

      logger->log_debug(name(), "setting security distance to %f", msg->security_distance());
      if_navi_->set_security_distance(msg->security_distance());

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetStopAtTargetMessage>() ) {
      NavigatorInterface::SetStopAtTargetMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetStopAtTargetMessage>();

      logger->log_debug(name(), "setting stop_at_target to %u", msg->is_stop_at_target());
      if_navi_->set_stop_at_target(msg->is_stop_at_target());

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetOrientationModeMessage>() ) {
      NavigatorInterface::SetOrientationModeMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetOrientationModeMessage>();

      logger->log_debug(name(), "setting orient_at_target to %s", if_navi_->tostring_OrientationMode( msg->orientation_mode() ) );
      if_navi_->set_orientation_mode( msg->orientation_mode() );

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetDriveModeMessage>() ) {
      NavigatorInterface::SetDriveModeMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetDriveModeMessage>();

      logger->log_debug(name(), "setting drive_mode to %s", if_navi_->tostring_DriveMode(msg->drive_mode()));
      if_navi_->set_drive_mode(msg->drive_mode());

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::ResetParametersMessage>() ) {

      logger->log_debug(name(), "resetting colli parameters to default values (from config)");
      if_navi_->set_max_velocity(cfg_max_velocity_);
      if_navi_->set_max_rotation(cfg_max_rotation_);
      if_navi_->set_escaping_enabled(cfg_escaping_enabled_);
      if_navi_->set_security_distance(cfg_security_distance_);
      if_navi_->set_stop_at_target(cfg_stop_at_target_);
      if_navi_->set_orientation_mode(cfg_orient_mode_);
      if_navi_->set_drive_mode(cfg_drive_mode_);

    } else {
      logger->log_debug(name(), "Ignoring unhandled Navigator message");
    }

    if_navi_->msgq_pop();
  }

  // process last motion message
  if( motion_msg ) {
    if( motion_msg->is_of_type<NavigatorInterface::StopMessage>() ) {
      logger->log_debug(name(), "StopMessage received");
      thread_colli_->colli_stop();

    } else if( motion_msg->is_of_type<NavigatorInterface::CartesianGotoMessage>() ) {
      NavigatorInterface::CartesianGotoMessage* msg = dynamic_cast<NavigatorInterface::CartesianGotoMessage*>(motion_msg);
      logger->log_debug(name(), "CartesianGotoMessage received, x:%f  y:%f  ori:%f", msg->x(), msg->y(), msg->orientation());
      // Converts from Fawkes Coord Sys -> RCSoftX Coord Sys, hence X_f = X_r, Y_f = -Y_r, Ori_f = -Ori_r
      if_navi_->set_msgid(msg->id());
      if_navi_->set_dest_x(msg->x());
      if_navi_->set_dest_y(msg->y());
      if_navi_->set_dest_ori(msg->orientation());
      if_navi_->set_dest_dist(sqrt(msg->x()*msg->x() + msg->y()*msg->y()));
      if_navi_->set_final(false);

      thread_colli_->colli_relgoto(msg->x(), msg->y(), msg->orientation(), if_navi_);

    } else if( motion_msg->is_of_type<NavigatorInterface::PolarGotoMessage>() ) {
      NavigatorInterface::PolarGotoMessage* msg = dynamic_cast<NavigatorInterface::PolarGotoMessage*>(motion_msg);
      logger->log_debug(name(), "PolarGotoMessage received, phi:%f  dist:%f", msg->phi(), msg->dist());
      // Converts from Fawkes Coord Sys -> RCSoftX Coord Sys, hence D_f = D_r, Phi_f = - Phi_r, Ori_f = Ori_r

      float cart_x = 0, cart_y = 0;
      polar2cart2d(msg->phi(), msg->dist(), &cart_x, &cart_y);

      if_navi_->set_msgid(msg->id());
      if_navi_->set_dest_x(cart_x);
      if_navi_->set_dest_y(cart_y);
      if_navi_->set_dest_ori(msg->orientation());
      if_navi_->set_dest_dist(msg->dist());
      if_navi_->set_final(false);

      thread_colli_->colli_relgoto(cart_x, cart_y, msg->orientation(), if_navi_);
    }

    motion_msg->unref();
  }

  if_navi_->write();
}

bool
ColliActThread::colli_final()
{
  return thread_colli_->is_final();
  // RCSoftX had more. Full code(from libmonaco and navigator_server) :
  //
  // return (thread_colli_->is_final() || m_pNavServer->isColliFinal() );
  // bbClients::Navigator_Server::isColliFinal()
  // {
  //  bool alive = (bool)m_ColliFeedback.GetValue( 8 /* magic value for alive index */ );
  //  bool final = (bool)m_ColliFeedback.GetValue( 0 /* magic value for final index */ );
  //  Timestamp cf(m_itsColliFeedback_sec.GetValue(), m_itsColliFeedback_usec.GetValue());
  //  Timestamp tp(m_itsTargetPoint_sec.GetValue(), m_itsTargetPoint_usec.GetValue());

  //  return alive && final && ( (cf - tp) == 0);
  // }
}
