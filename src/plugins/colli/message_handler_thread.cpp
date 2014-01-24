
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

#ifdef HAVE_ROS
 #include <ros/ros.h>
#endif

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
  cfg_security_distance_ = config->get_float((cfg_prefix + "security_distance").c_str());
  cfg_max_velocity_      = config->get_float((cfg_prefix + "max_velocity").c_str());
  cfg_max_rotation_      = config->get_float((cfg_prefix + "max_rotation").c_str());
  cfg_escaping_enabled_  = config->get_bool((cfg_prefix + "escaping_enabled").c_str());
  cfg_stop_at_target_    = config->get_bool((cfg_prefix + "stop_at_target").c_str());
  cfg_orient_at_target_  = config->get_bool((cfg_prefix + "orient_at_target").c_str());
  cfg_drive_mode_        = (NavigatorInterface::DriveMode)(config->get_int((cfg_prefix + "drive_mode").c_str()));
  logger->log_debug(name(), "Default drive_mode: %i (%s)", cfg_drive_mode_, if_navi_->tostring_DriveMode(cfg_drive_mode_));

  cfg_iface_navi_       = config->get_string((cfg_prefix + "interface/navigator").c_str());
  cfg_iface_motor_      = config->get_string((cfg_prefix + "interface/motor").c_str());

  cfg_frame_odom_       = config->get_string((cfg_prefix + "frame/odometry").c_str());

  if_navi_ = blackboard->open_for_writing<NavigatorInterface>(cfg_iface_navi_.c_str());
  if_motor_ = blackboard->open_for_reading<MotorInterface>(cfg_iface_motor_.c_str());

  if_colli_data_ = blackboard->open_for_reading<NavigatorInterface>("Colli data");
  if_colli_target_ = blackboard->open_for_writing<NavigatorInterface>("Colli target");

#ifdef HAVE_ROS
  std::string ros_target_topic = config->get_string((cfg_prefix + "ros/target_topic").c_str());
  sub_ = new ros::Subscriber();
  *sub_ = rosnode->subscribe(ros_target_topic.c_str(), 1, &ColliMessageHandlerThread::callbackSimpleGoal, this);
#endif

  security_distance_ = cfg_security_distance_;
  max_velocity_      = cfg_max_velocity_;
  max_rotation_      = cfg_max_rotation_;
  escaping_enabled_  = cfg_escaping_enabled_;
  stop_at_target_    = cfg_stop_at_target_;
  orient_at_target_  = cfg_orient_at_target_;
  drive_mode_        = cfg_drive_mode_;
}


void
ColliMessageHandlerThread::finalize()
{
  blackboard->close( if_navi_ );
  blackboard->close( if_colli_data_ );
  blackboard->close( if_colli_target_ );
  blackboard->close( if_motor_ );

#ifdef HAVE_ROS
  sub_->shutdown();
  delete(sub_);
#endif
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
      if_navi_->set_dest_dist(sqrt(msg->x()*msg->x() + msg->y()*msg->y()));
      if_navi_->set_final(false);

      colli_relgoto(msg->x(), msg->y(), msg->orientation());

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

      colli_relgoto(cart_x, cart_y, msg->orientation());

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

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetMaxRotationMessage>() ) {
      NavigatorInterface::SetMaxRotationMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetMaxRotationMessage>();

      logger->log_debug(name(), "setting max rotation velocity to %f", msg->max_rotation());
      max_velocity_ = msg->max_rotation();
      if_navi_->set_max_rotation(max_rotation_);

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

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetStopAtTargetMessage>() ) {
      NavigatorInterface::SetStopAtTargetMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetStopAtTargetMessage>();

      logger->log_debug(name(), "setting stop_at_target to %u", msg->is_stop_at_target());
      stop_at_target_ = msg->is_stop_at_target();
      if_navi_->set_stop_at_target(stop_at_target_);

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetOrientAtTargetMessage>() ) {
      NavigatorInterface::SetOrientAtTargetMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetOrientAtTargetMessage>();

      logger->log_debug(name(), "setting orient_at_target to %u", msg->is_orient_at_target());
      orient_at_target_ = msg->is_orient_at_target();
      if_navi_->set_orient_at_target(orient_at_target_);

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::SetDriveModeMessage>() ) {
      NavigatorInterface::SetDriveModeMessage *msg = if_navi_->msgq_first<NavigatorInterface::SetDriveModeMessage>();

      logger->log_debug(name(), "setting drive_mode to %f", if_navi_->tostring_DriveMode(msg->drive_mode()));
      drive_mode_ = msg->drive_mode();
      if_navi_->set_drive_mode(drive_mode_);

    } else if ( if_navi_->msgq_first_is<NavigatorInterface::ResetParametersMessage>() ) {

      logger->log_debug(name(), "resetting colli parameters to default values (from config)");
      max_velocity_      = cfg_max_velocity_;
      max_rotation_      = cfg_max_rotation_;
      escaping_enabled_  = cfg_escaping_enabled_;
      security_distance_ = cfg_security_distance_;
      stop_at_target_    = cfg_stop_at_target_;
      orient_at_target_  = cfg_orient_at_target_;
      drive_mode_        = cfg_drive_mode_;

      if_navi_->set_max_velocity(max_velocity_);
      if_navi_->set_max_rotation(max_rotation_);
      if_navi_->set_escaping_enabled(escaping_enabled_);
      if_navi_->set_security_distance(security_distance_);
      if_navi_->set_stop_at_target(stop_at_target_);
      if_navi_->set_orient_at_target(orient_at_target_);
      if_navi_->set_drive_mode(drive_mode_);

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
  if_colli_target_->set_dest_dist( 0.f );

  if_colli_target_->set_drive_mode( NavigatorInterface::MovingNotAllowed );
  if_colli_target_->set_stop_at_target( true );
  if_colli_target_->set_orient_at_target( false );
  if_colli_target_->set_escaping_enabled( false );

  if_colli_target_->write();
}

void
ColliMessageHandlerThread::colli_relgoto(float x, float y, float ori)
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

  this->colli_goto(colliTargetX, colliTargetY, colliTargetO);
}

void
ColliMessageHandlerThread::colli_goto(float x, float y, float ori)
{
  if_colli_target_->set_dest_x( x );
  if_colli_target_->set_dest_y( y );
  if_colli_target_->set_dest_ori( ori );

  // x and y are not needed anymore. use them for calculation of target distance
  x -= if_motor_->odometry_position_x();
  y -= if_motor_->odometry_position_y();
  float dist = sqrt(x*x + y*y);
  if_colli_target_->set_dest_dist(dist);

  if_colli_target_->set_drive_mode( drive_mode_ );
  if_colli_target_->set_security_distance( security_distance_ );
  if_colli_target_->set_max_velocity( max_velocity_ );
  if_colli_target_->set_max_rotation( max_rotation_ );
  if_colli_target_->set_escaping_enabled( escaping_enabled_ );
  if_colli_target_->set_stop_at_target( stop_at_target_ );
  if_colli_target_->set_orient_at_target( orient_at_target_ );

  if_colli_target_->write();
}

#ifdef HAVE_ROS
void
ColliMessageHandlerThread::callbackSimpleGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //calculate transform
  std::string from = msg->header.frame_id;  //maybe get this as well from the config ?? Should both be /map anyways
  std::string to = cfg_frame_odom_;

  float x = msg->pose.position.x;
  float y = msg->pose.position.y;
  float ori = tf::get_yaw(
      tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w)
  );

  bool world_frame_exists = tf_listener->frame_exists(from);
  bool robot_frame_exists = tf_listener->frame_exists(to);

  bool tf_ok = true;

  if (! world_frame_exists || ! robot_frame_exists) {
    logger->log_warn(name(), "Frame missing: %s %s   %s %s",
        from.c_str(), world_frame_exists ? "exists" : "missing",
        to.c_str(), robot_frame_exists ? "exists" : "missing");
  } else {
    fawkes::tf::StampedTransform transform;
      try {
        tf_listener->lookup_transform(to, from, transform);
      } catch (fawkes::tf::ExtrapolationException &e) {
        logger->log_debug(name(), "Extrapolation error");
        tf_ok = false;
      } catch (fawkes::tf::ConnectivityException &e) {
        logger->log_debug(name(), "Connectivity exception: %s", e.what());
        tf_ok = false;
      }

      if (tf_ok) {
        fawkes::tf::Point p = transform.getOrigin();
        x = x + p.getX();
        y = y + p.getY();

        tf::Quaternion q = transform.getRotation();
        ori = ori + tf::get_yaw(q);
      }
  }

  //send command
  this->colli_goto(x, y, ori);
}
#endif
