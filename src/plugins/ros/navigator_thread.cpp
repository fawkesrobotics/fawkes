
/***************************************************************************
 *  navigator_thread.cpp - Robotino ROS Navigator Thread
 *
 *  Created: Sat June 09 15:13:27 2012
 *  Copyright  2012  Sebastian Reuter
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

#include "navigator_thread.h"

using namespace fawkes;

/** @class RosNavigatorThread "ros_navigator_thread.h"
 * RosNavigator thread  links ros and fawkes
 * @author Sebastian Reuter
 */

/** Contructor. */
RosNavigatorThread::RosNavigatorThread()
  : Thread("RosNavigatorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

void
RosNavigatorThread::init()
{
  // navigator interface to give feedback of navigation task (writer)
  try {
    nav_if_ = blackboard->open_for_writing<NavigatorInterface>("Navigator");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open navigator "
             "interface for writing", name());
    logger->log_error(name(), e);
    throw;
  }

  //tell the action client that we want to spin a thread by default
  ac = new MoveBaseClient("move_base", false);

  logger->log_error( name(),"Change Interface (x,y) ");
  iterator = 0;
  isFirst = true;
  connected_history = false;
}

void
RosNavigatorThread::finalize()
{
  // close interfaces
  try {
    blackboard->close(nav_if_);
  } catch (Exception& e) {
    logger->log_error( name(), "Closing interface failed!" );
    logger->log_error( name(), e );
  }
  delete ac;
}

void
RosNavigatorThread::getStatus(){

	//DRECKIG!!
	if(!isFirst){
		if( ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			nav_if_->set_final(true);
			nav_if_->set_error_code(0);
		}
		else if( ac->getState() == actionlib::SimpleClientGoalState::ABORTED ||
				  ac->getState() == actionlib::SimpleClientGoalState::REJECTED){
			nav_if_->set_final(true);
			nav_if_->set_error_code(2);
		}
		else {
			nav_if_->set_final(false);
			nav_if_->set_error_code(0);
		}
	}
}

void
RosNavigatorThread::sendRosMessage(){

	//get goal from Navigation interface
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = nav_if_->dest_x();
	goal.target_pose.pose.position.y = nav_if_->dest_y();
	fawkes::tf::Quaternion q(nav_if_->dest_ori(), 0, 0);
	goal.target_pose.pose.orientation.x = q.x();
	goal.target_pose.pose.orientation.y = q.y();
	goal.target_pose.pose.orientation.z = q.z();
	goal.target_pose.pose.orientation.w = q.w();

	ac->sendGoal(goal);

	logger->log_info( name(),"Change Interface (x,y) = (%f,%f)", nav_if_->dest_x(), nav_if_->dest_y() );
}

void
RosNavigatorThread::loop()
{

	if( ac->isServerConnected() ){

		connected_history = true;
		// process incoming messages from fawkes
	  while ( ! nav_if_->msgq_empty()) {

		// stop
		if (NavigatorInterface::StopMessage *msg = nav_if_->msgq_first_safe(msg)) {
		  logger->log_info( name(),"Stop message received" );
		  nav_if_->set_dest_x( 0 );
		  nav_if_->set_dest_y( 0 );
		  nav_if_->set_dest_ori( 0 );

		  nav_if_->set_msgid( msg->id() );
		}

		// cartesian goto
		else if (NavigatorInterface::CartesianGotoMessage *msg = nav_if_->msgq_first_safe(msg)) {
		  logger->log_info( name(),"Cartesian goto message received (x,y) = (%f,%f)", msg->x(), msg->y() );
		  nav_if_->set_dest_x( msg->x() );
		  nav_if_->set_dest_y( msg->y() );
		  nav_if_->set_dest_ori( msg->orientation() );

		  nav_if_->set_msgid( msg->id() );
		}

		// polar goto
		else if (NavigatorInterface::PolarGotoMessage *msg = nav_if_->msgq_first_safe(msg)) {
		  logger->log_info( name(),"Polar goto message received (phi,dist) = (%f,%f)",msg->phi(), msg->dist() );
		  nav_if_->set_dest_x( msg->dist() * cos(msg->phi()) );
		  nav_if_->set_dest_y( msg->dist() * cos(msg->phi()) );
		  nav_if_->set_dest_ori( msg->phi() );

		  nav_if_->set_msgid( msg->id() );
		}

		// max velocity
		else if (NavigatorInterface::SetMaxVelocityMessage *msg = nav_if_->msgq_first_safe(msg)) {
		  logger->log_info( name(),"velocity message received %f",msg->max_velocity() );
		  nav_if_->set_max_velocity( msg->max_velocity() );

		  nav_if_->set_msgid( msg->id() );
		}

		else if (NavigatorInterface::SetSecurityDistanceMessage *msg = nav_if_->msgq_first_safe(msg)) {
		  logger->log_info( name(),"velocity message received %f",msg->security_distance () );
		  nav_if_->set_security_distance ( msg->security_distance () );

		  nav_if_->set_msgid( msg->id() );
		}

		nav_if_->msgq_pop();
		nav_if_->write();
		this->sendRosMessage();

		isFirst = false;
	  } // while

	  //get the status of the Ros-navigation
	  this->getStatus();
	  nav_if_->write();

	} // if
	else{
		logger->log_info( name(),"ROS-ActionServer is not up yet" );

		if (connected_history){
			delete ac;
			ac = new MoveBaseClient("move_base", false);
			connected_history = false;
		}
	}

} // function
