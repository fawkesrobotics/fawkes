
/***************************************************************************
 *  navigator_thread.cpp - Navigator Thread
 *
 *  Generated: Thu May 31 18:36:55 2007
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <plugins/navigator/navigator_thread.h>
#include <interfaces/navigator.h>
#include <interfaces/motor.h>
#include <interfaces/object.h>

#include <cmath>
#include <unistd.h>

/** Contructor. */
NavigatorThread::NavigatorThread()
    : Thread("NavigatorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  logger_modulo_counter = 0;
  old_velocity_x = 0;
  old_velocity_y = 0;
}

/** Destructor. */
NavigatorThread::~NavigatorThread()
{}


void
NavigatorThread::finalize()
{
  try
    {
      interface_manager->close(navigator_interface);
      interface_manager->close(motor_interface);
    //  interface_manager->close(object_interface);
    }
  catch (Exception& e)
    {
      logger->log_error("NavigatorThread", "Closing interface failed!");
      logger->log_error("NavigatorThread", e);
    }
}


void
NavigatorThread::init()
{
  try
    {
      navigator_interface = interface_manager->open_for_writing<NavigatorInterface>("Navigator");
    }
  catch (Exception& e)
    {
      e.append("%s initialization failed, could not open navigator interface for writing", name());
      logger->log_error("NavigatorThread", "Opening interface failed!");
      logger->log_error("NavigatorThread", e);
      throw;
    }

  try
    {
      motor_interface = interface_manager->open_for_reading<MotorInterface>("Motor");
    }
  catch (Exception& e)
    {
      e.append("%s initialization failed, could not open motor interface for reading", name());
      logger->log_error("NavigatorThread", "Opening interface for reading failed!");
      logger->log_error("NavigatorThread", e);
      throw;
    }

  try
    {
      object_interface_list = interface_manager->open_all_of_type_for_reading("ObjectPositionInterface");
    }
  catch (Exception& e)
    {
      e.append("%s initialization failed, could not open object interface for reading", name());
      logger->log_error("NavigatorThread", "Opening interface for reading failed!");
      logger->log_error("NavigatorThread", e);
      throw;
    }

}


void
NavigatorThread::once()
{
  MotorInterface::AcquireControlMessage *msg = new MotorInterface::AcquireControlMessage();
  motor_interface->msgq_enqueue(msg);
}


void
NavigatorThread::loop()
{

  motor_interface->read();
  

  if ( navigator_interface->msgq_first_is<NavigatorInterface::TargetMessage>() )
    {
      NavigatorInterface::TargetMessage* msg = navigator_interface->msgq_first<NavigatorInterface::TargetMessage>();

      logger->log_info("NavigatorThread", "target message received %f, %f", msg->x(), msg->y());

      if(motor_interface->controller_thread_id() == current_thread_id())
        {
          goTo_cartesian(msg->x(), msg->y());
        }
      navigator_interface->msgq_pop();

    }
  else if ( navigator_interface->msgq_first_is<NavigatorInterface::VelocityMessage>() )
    {
      NavigatorInterface::VelocityMessage* msg = navigator_interface->msgq_first<NavigatorInterface::VelocityMessage>();

      logger->log_info("NavigatorThread", "velocity message received %f", msg->velocity());

      if(motor_interface->controller_thread_id() == current_thread_id())
        {
          setVelocity(msg->velocity());
        }
      navigator_interface->msgq_pop();
    }
  else if ( navigator_interface->msgq_first_is<NavigatorInterface::ObstacleMessage>() )
    {
      NavigatorInterface::ObstacleMessage* msg = navigator_interface->msgq_first<NavigatorInterface::ObstacleMessage>();

      logger->log_info("NavigatorThread", "obstacle message received");

      if(motor_interface->controller_thread_id() == current_thread_id())
        {
          Obstacle o(msg->width(), msg->x(), msg->y(), 0.);
          add_obstacle(o);
        }
      navigator_interface->msgq_pop();
    }
    
  std::list<Interface *>::iterator i;
  for ( i = object_interface_list->begin(); i != object_interface_list->end(); ++i )
    {
      ObjectPositionInterface *object_interface = (ObjectPositionInterface *) *i;
      float distance = object_interface->distance();
      float yaw = object_interface->yaw();
      float width = 0.5;
      logger->log_info("NavigatorThread", "Object at distance = %f, yaw = %f", distance, yaw);
      std::vector<Obstacle> obstacle_list;
      obstacle_list.push_back(*(new Obstacle(width, distance * cos(yaw), distance * sin(yaw), 0)));
      setObstacles(obstacle_list);
    }
    
  //from navigator
  mainLoop();
  /*
  if((++logger_modulo_counter %= 10) == 0)
    {
      logger->log_info("NavigatorThread", "x = %f, y = %f, externel_control = %i, external_control_thread_id = %lu", getVelocityX(), getVelocityY(), external_control, external_control_thread_id);
    }
    */
  if(motor_interface->controller_thread_id() == current_thread_id())
    {
      if(old_velocity_x != getVelocityX() || old_velocity_y != getVelocityY())
        {
          old_velocity_x = getVelocityX();
          old_velocity_y = getVelocityY();
          MotorInterface::TransMessage* motor_msg = new  MotorInterface::TransMessage(getVelocityX(), getVelocityY());
          motor_interface->msgq_enqueue(motor_msg);

          //   if((++logger_modulo_counter % 5) == 0)
          {
            logger->log_info("NavigatorThread", "send x = %f, y = %f", getVelocityX(), getVelocityY());
          }
        }
    }
  /*
  if((++logger_modulo_counter %= 10) == 0)
    {
      logger->log_info("NavigatorThread", "NavigatorThread called: %lu, %lu", motor_interface->getControllerID(), this->current_thread_id());
    }
  */
  //usleep(100000);

}
