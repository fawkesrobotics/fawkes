
/***************************************************************************
 *  joystick_control.cpp - Joystick Control
 *
 *  Generated: Tue Jun 05 14:52:10 2007
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
 
#include <plugins/navigator/joystick_control.h>
#include <blackboard/interface_manager.h>
#include <utils/logging/logger.h>
#include <interfaces/motor_interface.h>
#include <interfaces/kicker.h>
#include <config/config.h>

#include <cmath>


/** @class JoystickControl <plugins/navigator/joystick_control.h>
 *      The joystick control for the motors.
 *      It outreaches the incoming driving commands from the joystick tool
 *      to the motor thread and checks whether there is an obstale
 *      and stops the motors respectively.
 */
/** @var JoystickControl::motor_interface
 *      The interface to communicate with the motor thread.
 *      The joystick control gets it from the navigator network thread.
 */
/** @var JoystickControl::logger
 *      The logger to log something.
 *     The joystick control gets it from the navigator network thread.
 */

 
/** Constructor. 
 * @param motor_interface the motor interface from the navigator network thread
 * @param kicker_interface the kicker interface from the navigator network thread
 * @param logger the logger from the navigator network thread
 * @param config the config from the navigator network thread
 */
JoystickControl::JoystickControl(MotorInterface *motor_interface, KickerInterface *kicker_interface,Logger *logger, Configuration *config)
{
  this->motor_interface = motor_interface;
  this->kicker_interface = kicker_interface;
  this->logger = logger;
  this->config = config;
  actual_speed = 0;
  actual_joystick_axis_scale = 0;
  logger_modulo_counter = 0;
  logger_modulo_counter2 = 0;
}

/** Destructor. */
JoystickControl::~JoystickControl()
{/*
   try 
   {
   interface_manager->close(motor_interface);
   }
   catch (Exception& e)
   {
   logger->log_error("JoystickControl", "Closing interface failed!");
   logger->log_error("JoystickControl", e);
   }*/
}

/**  With this method the navigator network thread enqueues the kick command
 *   from the joystick tool to send it out to the kicker plugin.
 * @param left the left kicker command
 * @param center the center kicker command
 * @param right the right kicker command
 */
void 
JoystickControl::enqueueKick(bool left, bool center, bool right) 
{
  KickerInterface::KickMessage* msg = new  KickerInterface::KickMessage(right, center, left, false, false, false);
  
  if(kicker_interface->hasWriter())
    kicker_interface->msgq_enqueue(msg);
  
  if((++logger_modulo_counter %= 100) == 0)
    {
      logger->log_info("JoystickControl", "kick left: %f, center: %f, right: %f", left, center, right);
    }
        
}

/** With this method the navigator network thread enqueues the drive command
 *   from the joystick tool to send it out to the motor.
 *  @param forward forward command
 *  @param sideward sideward command
 *  @param rotation rotation command
 *  @param max_speed maximum speed command
 */
void 
JoystickControl::enqueueCommand(double forward, double sideward, double rotation, double max_speed)
{ 
  /*if((++logger_modulo_counter2 %= 100) == 0)
    {
      logger->log_info("JoystickControl", "enqueueCommand");
    }
  */
  double axis_scale = sqrt(pow(forward, 2) + pow(sideward, 2));
  double rotation_scale = fabs(rotation);
  
  if(max_speed * axis_scale <= actual_speed)// || actual_joystick_axis_scale > sqrt(fabs(forward) + fabs(sideward))/sqrt(2))
    {
      actual_speed = max_speed * axis_scale;
    
      //  if((++logger_modulo_counter2 %= 100) == 0)
      {
    //    logger->log_info("JoystickControl", "if1 actual speed: %f max_speed: %f", actual_speed, max_speed);
      }
    }
  else if(forward == 0. && sideward == 0. && rotation != 0.  && actual_rotation_scale <= rotation_scale && max_speed * rotation_scale >= actual_speed)
    {
      actual_speed += (max_speed * rotation_scale) / config->get_float("navigator", "/joystick_c/max_acceleration");
    //  logger->log_info("JoystickControl", "if4 actual speed: %f speed: %f", actual_speed, max_speed);
    }
  else if((forward != 0. || sideward != 0. || rotation != 0.) && actual_joystick_axis_scale <= axis_scale && max_speed * axis_scale >= actual_speed)
    {
      try
        {
          actual_speed += (max_speed * axis_scale) / config->get_float("navigator", "/joystick_c/max_acceleration");
          // logger->log_info("JoystickControl", "/joystick_c/max_acceleration: %f", config->get_float("navigator", "/joystick_c/max_acceleration"));
                                                 
    //      logger->log_info("JoystickControl", "if2 actual speed: %f speed: %f", actual_speed,max_speed);
    //      logger->log_info("JoystickControl", "if2.1 axis_scale: %f", axis_scale);
    
        } 
      catch (Exception &e) 
        {
          logger->log_error("JoystickControl", "enqueueCommand()");
          logger->log_error("JoystickControl", e);
          throw;
        }
    } 
  else if(forward == 0. && sideward == 0. && rotation == 0.)
    {
      actual_speed = 0;
        
   //   logger->log_info("JoystickControl", "if3 actual speed: %f speed: %f", actual_speed, max_speed);
    } 
  
  actual_joystick_axis_scale = axis_scale;
  actual_rotation_scale = rotation_scale;
  /* 
     logger->log_info("JoystickControl", "actual_rotation_scale <= rotation_scale: %i", (actual_rotation_scale <= rotation_scale));
     logger->log_info("JoystickControl", "max_speed * rotation_scale >= actual_speed: %i", (max_speed * rotation_scale >= actual_speed));
     logger->log_info("JoystickControl", "forward == 0: %i", (forward == 0));
     logger->log_info("JoystickControl", "sideward == 0: %i", (sideward == 0));
     logger->log_info("JoystickControl", "rotation != 0: %i", (rotation != 0));
     logger->log_info("JoystickControl", "actual_joystick_axis_scale <= axis_scale: %i", (actual_joystick_axis_scale <= axis_scale));
     logger->log_info("JoystickControl", "max_speed * rotation_scale >= actual_speed: %i", (max_speed * rotation_scale >= actual_speed));
     logger->log_info("JoystickControl", "forward != 0: %i", (forward != 0));
     logger->log_info("JoystickControl", "sideward != 0: %i", (sideward != 0));
     logger->log_info("JoystickControl", "rotation != 0: %i", (rotation != 0));
     logger->log_info("JoystickControl", "rotation: %f", rotation);
     logger->log_info("JoystickControl", "max_speed * axis_scale > actual_speed: %i", (max_speed * axis_scale > actual_speed));
     logger->log_info("JoystickControl", "actual speed: %f max_speed: %f", actual_speed, max_speed);
     logger->log_info("JoystickControl", "actual_joystick_axis_scale: %f sqrt(fabs(forward) + fabs(sideward))/sqrt(2): %f", actual_joystick_axis_scale, sqrt(fabs(forward) + fabs(sideward))/sqrt(2));
  */
  MotorInterface::JoystickMessage* msg = new  MotorInterface::JoystickMessage(forward, sideward, rotation, actual_speed);
  
  motor_interface->msgq_enqueue(msg); 
  
  // msg->unref();
}
