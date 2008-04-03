
/***************************************************************************
 *  navigator_net_thread.cpp - Navigator Plugin Network Thread
 *
 *  Generated: Thu May 31 20:38:50 2007
 *  Copyright  2007  Martin Liebenberg
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/navigator/navigator_net_thread.h>
#include <plugins/navigator/libnavi/navigator_messages.h>
#include <plugins/navigator/libnavi/npoint.h>
#include <plugins/navigator/libnavi/nline.h>
#include <plugins/navigator/libnavi/obstacle.h>
#include <netcomm/fawkes/component_ids.h>
#include <plugins/navigator/joystick_control.h>
#include <interfaces/motor.h>
#include <interfaces/navigator.h>
#include <interfaces/kicker.h>
#include <plugins/navigator/navigator_thread.h>

#include <cstdlib>
#include <unistd.h>
#include <algorithm>
#include <list>
#include <arpa/inet.h>
#include <cstring>

/** @class NavigatorNetworkThread plugins/navigator/navigator_net_thread.h
 * Network thread of the navigator plugin.
 * 
 * @author Martin Liebenberg
 */
/** @var NavigatorNetworkThread::motor_interface
 *   The interface to tell the motor thread what is allowed to control the motors.
 */

/** Constructor.
 * @param navigator_thread the navigator thread to get path informations directly from
 * the navigator
 */
NavigatorNetworkThread::NavigatorNetworkThread(NavigatorThread *navigator_thread)
    : Thread("NavigatorNetworkThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP),
    FawkesNetworkHandler(FAWKES_CID_NAVIGATOR_PLUGIN)
{
  this->navigator_thread = navigator_thread;
  connected_control_client = 0;
  logger_modulo_counter = 0;

  last_motor_controller = 0;
  last_motor_controller_thread_name = NULL;

  kicker_interface = NULL;
  motor_interface = NULL;
  navigator_interface = NULL;
  joystick_control = NULL;
  sending_pause = 0.;
}


/** Destructor. */
NavigatorNetworkThread::~NavigatorNetworkThread()
{
  if ( last_motor_controller_thread_name != NULL )
    {
      free( last_motor_controller_thread_name );
    }
}


void
NavigatorNetworkThread::finalize()
{
  logger->log_info("NavigatorNetworkThread", "Finalizing thread %s", name());

  logger->log_info("NavigatorNetworkThread", "Removing this thread from list of Fawkes network hub handlers");
  fnethub->remove_handler( this );

  delete joystick_control;

  try
    {
      blackboard->close(motor_interface);
      blackboard->close(navigator_interface);
      blackboard->close(kicker_interface);
    }
  catch (Exception& e)
    {
      logger->log_error("NavigatorNetworkThread", "Closing interface failed!");
      logger->log_error("NavigatorNetworkThread", e);
    }
}

void
NavigatorNetworkThread::init()
{

  try
    {
      motor_interface = blackboard->open_for_reading<MotorInterface>("Motor");
    }
  catch (Exception& e)
    {
      e.append("%s initialization failed, could not open motor interface for reading", name());
      logger->log_error("NavigatorNetworkThread", "Opening interface for reading failed!");
      logger->log_error("NavigatorNetworkThread", e);
      throw;
    }

  try
    {
      kicker_interface = blackboard->open_for_reading<KickerInterface>("Kicker");
    }
  catch (Exception& e)
    {
      e.append("%s initialization failed, could not open kicker interface for reading", name());
      logger->log_error("NavigatorNetworkThread", "Opening interface for reading failed!");
      logger->log_error("NavigatorNetworkThread", e);
      throw;
    }

  try
    {
      navigator_interface = blackboard->open_for_reading<NavigatorInterface>("Navigator");
    }
  catch (Exception& e)
    {
      e.append("%s initialization failed, could not open navigator interface for reading", name());
      logger->log_error("NavigatorNetworkThread", "Opening interface for reading failed!");
      logger->log_error("NavigatorNetworkThread", e);
      throw;
    }

  try
    {
      joystick_control = new JoystickControl(motor_interface, kicker_interface, logger, config, clock);
    }
  catch (Exception &e)
    {
      e.append("NavigatorNetworkThread could not initialize JoystickControl");
      throw;
    }

  sending_pause = config->get_float("/navigator/network/sending_pause");
  if(sending_pause < 0)
    {
      throw Exception("Navigator sending pause is negative");
    }
  sending_time = clock->now();


  // logger->log_debug("NavigatorNetworkThread", "Adding network handler");
  fnethub->add_handler( this );
}


void
NavigatorNetworkThread::process_network_message(FawkesNetworkMessage *msg)
{
  if(msg->msgid() == NAVIGATOR_MSGTYPE_JOYSTICK )
    {
      if (connected_control_client == msg->clid())
        {
          navigator_joystick_message_t *u = (navigator_joystick_message_t *)msg->payload();

          if ( motor_interface->controller() == navigator_interface->serial() )
            {
              joystick_control->enqueueCommand(u->forward, u->sideward, u->rotation, u->speed);
            }
          else
            {
              logger->log_warn("NavigatorNetworkThread", "Thread %s (%l) stole our motor control!",
                               motor_interface->controller_thread_name(),
                               motor_interface->controller());
            }
        }
      else
        {
          logger->log_warn("NavigatorNetworkThread", "Client %u sent joystick message while not subscribed", msg->clid());
        }
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_SUBSCRIBE)
    {
      navigator_subscribe_message_t *u = (navigator_subscribe_message_t *)msg->payload();

      if(u->sub_type_points_and_lines)
        {
          connected_points_and_lines_clients.push_back_locked(msg->clid());
          logger->log_debug("NavigatorNetworkThread", "Client %u subscribed as receiver of points and lines", connected_control_client);
        }

      if(u->sub_type_odometry)
        {
          connected_odometry_clients.push_back_locked(msg->clid());
        }

      if(u->sub_type_ball)
        {
          connected_ball_clients.push_back_locked(msg->clid());
        }

      if(u->sub_type_motor_control && connected_control_client == 0)
        {
          connected_control_client = msg->clid();
          motor_interface->read();
          last_motor_controller = motor_interface->controller();
          // this needs to be a strncpy, but currently the interfaces do not provide enough
          // functionality
          if ( last_motor_controller_thread_name != NULL )
            {
              free(last_motor_controller_thread_name);
            }
          last_motor_controller_thread_name = strdup(motor_interface->controller_thread_name());

          MotorInterface::AcquireControlMessage* acmsg = new MotorInterface::AcquireControlMessage();
          motor_interface->msgq_enqueue(acmsg);

          logger->log_debug("NavigatorNetworkThread", "Client %u subscribed as motor controller", connected_control_client);
        }
      else if(u->sub_type_navigator_control && connected_control_client == 0)
        {
          connected_control_client = msg->clid();

          logger->log_debug("NavigatorNetworkThread", "Client %u subscribed as navigator controller", connected_control_client);
        }
      else if((u->sub_type_motor_control || u->sub_type_navigator_control) && connected_control_client != 0 )
        {
          logger->log_warn("NavigatorNetworkThread", "Client %u tried to subscribe but there is already another subscriber (Fawkes)",
                           msg->clid());
          fnethub->send(msg->clid(),
                        FAWKES_CID_NAVIGATOR_PLUGIN,
                        NAVIGATOR_MSGTYPE_CONTROL_SUBERR);
        }
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_UNSUBSCRIBE)
    {
      if ( msg->clid() == connected_control_client )
        {
          logger->log_info("NavigatorNetworkThread", "Message of type unsubscribe message received");
          navigator_unsubscribe_message_t *u = (navigator_unsubscribe_message_t *)msg->payload();

          if(u->unsub_type_points_and_lines)
            {
              connected_points_and_lines_clients.remove_locked(msg->clid());
            }
          else if(u->unsub_type_odometry)
            {
              connected_odometry_clients.remove_locked(msg->clid());
            }
          else if(u->unsub_type_ball)
            {
              connected_ball_clients.remove_locked(msg->clid());
            }
          else if(u->unsub_type_motor_control)
            {
              connected_control_client = 0;
              MotorInterface::AcquireControlMessage* acmsg = new  MotorInterface::AcquireControlMessage();
              acmsg->set_controller(last_motor_controller);
              acmsg->set_controller_thread_name(last_motor_controller_thread_name);
              motor_interface->msgq_enqueue(acmsg);
              last_motor_controller = 0;
            }
          else if(u->unsub_type_navigator_control)
            {
              connected_control_client = 0;
            }
        }
      else
        {
          logger->log_error(name(), "Client %u tried to unsubscribe while not subscribed");
        }
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_TARGET
          && msg->clid() == connected_control_client)
    {
      logger->log_info("NavigatorNetworkThread", "Message of type target message received");
      navigator_target_message_t *u = (navigator_target_message_t *)msg->payload();

      NavigatorInterface::CartesianGotoMessage* tmsg = new NavigatorInterface::CartesianGotoMessage(u->x, u->y, u->orientation);
      navigator_interface->msgq_enqueue(tmsg);
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_VELOCITY
          && msg->clid() == connected_control_client)
    {
      logger->log_info("NavigatorNetworkThread", "Message of type velocity message received");
      navigator_velocity_message_t *u = (navigator_velocity_message_t *)msg->payload();

      NavigatorInterface::MaxVelocityMessage* vmsg = new  NavigatorInterface::MaxVelocityMessage();
      vmsg->set_velocity(u->value);
      navigator_interface->msgq_enqueue(vmsg);
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_TRANS_ROT
          && msg->clid() == connected_control_client)
    {
      logger->log_info("NavigatorNetworkThread", "Message of type trans rot message received");
      navigator_trans_rot_message_t *u = (navigator_trans_rot_message_t *)msg->payload();

      if(u->type_trans_rot)
        {
          MotorInterface::TransRotMessage* trmsg = new  MotorInterface::TransRotMessage();

          trmsg->set_vx(u->forward);
          trmsg->set_vy(u->sideward);
          trmsg->set_omega(u->rotation);
          motor_interface->msgq_enqueue(trmsg);
        }
      else if(u->type_trans)
        {
          MotorInterface::TransMessage* tmsg = new  MotorInterface::TransMessage();

          tmsg->set_vx(u->forward);
          tmsg->set_vy(u->sideward);
          motor_interface->msgq_enqueue(tmsg);
        }
      else if(u->type_rot)
        {
          MotorInterface::RotMessage* rmsg = new  MotorInterface::RotMessage();

          rmsg->set_omega(u->rotation);
          motor_interface->msgq_enqueue(rmsg);
        }
      else if(u->type_line_trans_rot)
        {
          MotorInterface::LinTransRotMessage* ltrmsg = new  MotorInterface::LinTransRotMessage();
          ltrmsg->set_vx(u->forward);
          ltrmsg->set_vy(u->sideward);
          ltrmsg->set_omega(u->rotation);
          motor_interface->msgq_enqueue(ltrmsg);
        }
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_RPM
          && msg->clid() == connected_control_client)
    {
      logger->log_info("NavigatorNetworkThread", "Message of type rpm message received");
      navigator_rpm_message_t *u = (navigator_rpm_message_t *)msg->payload();

      MotorInterface::DriveRPMMessage* drmsg = new  MotorInterface::DriveRPMMessage();
      drmsg->set_front_left(u->left);
      drmsg->set_rear(u->rear);
      drmsg->set_front_right(u->right);
      motor_interface->msgq_enqueue(drmsg);
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_ORBIT
          && msg->clid() == connected_control_client)
    {
      logger->log_info("NavigatorNetworkThread", "Message of type orbit message received");
      navigator_orbit_message_t *u = (navigator_orbit_message_t *)msg->payload();

      MotorInterface::OrbitMessage* ommsg = new MotorInterface::OrbitMessage();
      ommsg->set_px(u->orbit_center_x);
      ommsg->set_py(u->orbit_center_y);
      ommsg->set_omega(u->angular_velocity);
      motor_interface->msgq_enqueue(ommsg);
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_RESET_ODOMETRY
          && msg->clid() == connected_control_client)
    {
      MotorInterface::ResetOdometryMessage* romsg = new MotorInterface::ResetOdometryMessage();
      motor_interface->msgq_enqueue(romsg);
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_KICK
          && msg->clid() == connected_control_client)
    {
      navigator_kick_message_t *u = (navigator_kick_message_t *)msg->payload();
      //  logger->log_info("NavigatorNetworkThread", "kick message: %i, %i, %i", u->left, u->center, u->right);
      joystick_control->enqueueKick(u->left, u->center, u->right);
    }
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_OBSTACLE
          && msg->clid() == connected_control_client)
    {
      navigator_obstacle_msg_t *u = (navigator_obstacle_msg_t *)msg->payload();
      NavigatorInterface::ObstacleMessage* omsg = new  NavigatorInterface::ObstacleMessage();

      omsg->set_x(u->x);
      omsg->set_y(u->y);
      omsg->set_width(u->width);
      navigator_interface->msgq_enqueue(omsg);
    }
  else
    {
      if ( msg->clid() != connected_control_client )
	{
	  logger->log_warn("NavigatorNetworkThread", "Received message of client %u which is "
			   "not the controlling client (%u)", msg->clid(),
			   connected_control_client);
	}
      else
	{
	  logger->log_error("NavigatorNetworkThread", "Message of invalid type %u received",
			    msg->msgid());
	}
    }
}


void
NavigatorNetworkThread::loop()
{
  motor_interface->read();

  inbound_queue.lock();
  while ( ! inbound_queue.empty() )
    {
      FawkesNetworkMessage *msg = inbound_queue.front();
      process_network_message(msg);
      msg->unref();
      inbound_queue.pop();
    }
  inbound_queue.unlock();

  //wait some time to save resources
  if(clock->elapsed(&sending_time) > sending_pause)
    {
      sending_time = clock->now();

      for(std::list<unsigned int>::iterator iterator = connected_points_and_lines_clients.begin();
          iterator != connected_points_and_lines_clients.end();
          iterator++ )
        {

          //send lines with points and obstacles
          std::list<NLine *> *lines = navigator_thread->get_surface_lines();
          NavigatorSurfaceMessage *surface_msg = new NavigatorSurfaceMessage(lines);
          fnethub->send(*iterator, FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_SURFACE, surface_msg);

          //send path
          std::list<NPoint *> *path_points = navigator_thread->get_path_points();
          NavigatorPathListMessage *path_list_msg = new NavigatorPathListMessage(path_points);
          fnethub->send(*iterator, FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_PATH, path_list_msg);

          //delete lines
          for(std::list<NLine *>
              ::iterator line_iterator = lines->
                                         begin();
              line_iterator != lines->end();
              line_iterator++)
            {
              delete *line_iterator;
            }
          lines->clear();

          //delete path_points
          for(std::list<NPoint *>
              ::iterator path_iterator = path_points->begin();
              path_iterator != path_points->end();
              path_iterator++)
            {
              delete *path_iterator;
            }
          path_points->clear();
        }

      //send ball position
      for(std::list<unsigned int>::iterator iterator = connected_ball_clients.begin();
          iterator != connected_ball_clients.end();
          iterator++ )
        {
          navigator_ball_message_t *ball_msg= (navigator_ball_message_t *)malloc(sizeof(navigator_ball_message_t));
          ball_msg->x = navigator_thread->get_ball_position_x();
          ball_msg->y = navigator_thread->get_ball_position_y();

          fnethub->send(*iterator, FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_BALL, ball_msg, sizeof(navigator_ball_message_t));
        }


      //send odometry data
      for(std::list<unsigned int>::iterator iterator = connected_odometry_clients.begin();
          iterator != connected_odometry_clients.end();
          iterator++ )
        {
          navigator_odometry_message_t *odometry_msg= (navigator_odometry_message_t *)malloc(sizeof(navigator_odometry_message_t));

          odometry_msg->path_length = motor_interface->odometry_path_length();
          odometry_msg->position_x = motor_interface->odometry_position_x();
          odometry_msg->position_y = motor_interface->odometry_position_y();
          odometry_msg->orientation = motor_interface->odometry_orientation();
          odometry_msg->rpm_left = motor_interface->left_rpm();
          odometry_msg->rpm_rear = motor_interface->rear_rpm();
          odometry_msg->rpm_right = motor_interface->right_rpm();
          odometry_msg->velocity_x = motor_interface->vx();
          odometry_msg->velocity_y = motor_interface->vy();
          odometry_msg->velocity_rotation = motor_interface->omega();

          fnethub->send(*iterator, FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_ODOMETRY, odometry_msg, sizeof(navigator_odometry_message_t));
        }

    }//if(clock->elapsed(&sending_time) > sending_pause)

}


void
NavigatorNetworkThread::handle_network_message(FawkesNetworkMessage *msg)
{
  //logger->log_info("NavigatorNetworkThread", "Message of type %i received", msg->msgid());
  msg->ref();
  inbound_queue.push_locked(msg);
}


/** Client connected.
 * Ignored.
 * @param clid client ID
 */
void
NavigatorNetworkThread::client_connected(unsigned int clid)
{
  // logger->log_info("NavigatorNetworkThread", "Client %u connected", clid);
}


/** Client disconnected.
 * If the client was a subscriber it is removed.
 * @param clid client ID
 */
void
NavigatorNetworkThread::client_disconnected(unsigned int clid)
{
  // logger->log_info("NavigatorNetworkThread", "Client %u disconnected", clid);

  std::list<unsigned int>::iterator result;

  connected_points_and_lines_clients.lock();
  result = find( connected_points_and_lines_clients.begin(),
                 connected_points_and_lines_clients.end()  , clid );
  if(result != connected_points_and_lines_clients.end())
    {
      connected_points_and_lines_clients.remove(clid);
    }
  connected_points_and_lines_clients.unlock();

  connected_odometry_clients.lock();
  result = find( connected_odometry_clients.begin(),
                 connected_odometry_clients.end()  , clid );
  if(result != connected_odometry_clients.end())
    {
      connected_odometry_clients.remove(clid);
    }
  connected_odometry_clients.unlock();

  connected_ball_clients.lock();
  result = find( connected_ball_clients.begin(),
                 connected_ball_clients.end()  , clid );
  if(result != connected_ball_clients.end())
    {
      connected_ball_clients.remove(clid);
    }
  connected_ball_clients.unlock();

  if(connected_control_client == clid)
    {
      connected_control_client = 0;
      /*
      MotorInterface::AquireControlMessage* msg1 = new  MotorInterface::AquireControlMessage();
      motor_interface->msgq_enqueue(msg1);
          
          MotorInterface::DriveRPMMessage* msg2 = new  MotorInterface::DriveRPMMessage();
          msg2->setFrontLeft(0.);
          msg2->setRear(0.);
          msg2->setFrontRight(0.);
          motor_interface->msgq_enqueue(msg2);
      */
      logger->log_debug("NavigatorNetworkThread", "Client %u unsubscribed as controller", clid);
      if(last_motor_controller != 0)
        {
          MotorInterface::AcquireControlMessage* msg = new  MotorInterface::AcquireControlMessage();
          msg->set_controller(last_motor_controller);
          msg->set_controller_thread_name(last_motor_controller_thread_name);
          motor_interface->msgq_enqueue(msg);
        }
    }
}
