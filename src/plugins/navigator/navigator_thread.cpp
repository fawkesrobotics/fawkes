
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
#include <interfaces/navigator_interface.h>
#include <interfaces/motor_interface.h>

#include <unistd.h>

/** Contructor. */
NavigatorThread::NavigatorThread()
  : Thread("NavigatorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  logger_modulo_counter = 0;
}

/** Deconstructor. */
NavigatorThread::~NavigatorThread()
{
}


void
NavigatorThread::finalize()
{
  try
    {
      interface_manager->close(navigator_interface);
      interface_manager->close(motor_interface);
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
  logger->log_error("NavigatorThread", "Opening navigator interface");
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
}


void
NavigatorThread::loop()
{ 
        
  motor_interface->read();
        
  if ( navigator_interface->msgq_first_is<NavigatorInterface::TargetMessage>() )
    {
      NavigatorInterface::TargetMessage* msg = navigator_interface->msgq_first<NavigatorInterface::TargetMessage>();

      logger->log_info("NavigatorThread", "target message received %f, %f", msg->getX(), msg->getY());
      
      goTo_cartesian(msg->getX(), msg->getY());
      
      navigator_interface->msgq_pop();
      
    }
  else if ( navigator_interface->msgq_first_is<NavigatorInterface::VelocityMessage>() )
    {
      NavigatorInterface::VelocityMessage* msg = navigator_interface->msgq_first<NavigatorInterface::VelocityMessage>();

      logger->log_info("NavigatorThread", "velocity message received %f, %f", msg->getVelocity());
      
      setVelocity(msg->getVelocity());
      
      navigator_interface->msgq_pop();
      
    }
  //from navigator
  // mainLoop();
  
  
  if(motor_interface->getControllerID() == current_thread_id())
    {
      MotorInterface::NavigatorMessage* motor_msg = new  MotorInterface::NavigatorMessage(getVelocityY(), getVelocityX(), 0, getVelocity());
      //float iniCmdRotation, float iniCmdVelocity
      motor_interface->msgq_enqueue(motor_msg); 
      
      if((++logger_modulo_counter % 5) == 0)
        {
          logger->log_info("NavigatorThread", "send");
        }
    }
     
  if((++logger_modulo_counter %= 10) == 0)
    {
      logger->log_info("NavigatorThread", "NavigatorThread called: %lu, %lu", motor_interface->getControllerID(), this->current_thread_id());
    }
  //usleep(100000);
}
