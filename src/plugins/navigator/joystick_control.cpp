
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
#include <utils/time/clock.h>
#include <interfaces/motor.h>
#include <interfaces/kicker.h>
#include <config/config.h>

#include <cmath>


/** @class JoystickControl <plugins/navigator/joystick_control.h>
 *      The joystick control for the motors.
 *      It outreaches the incoming driving commands from the joystick tool
 *      to the motor thread and checks whether there is an obstale
 *      and stops the motors respectively.
 * 	 The NavigatorNetworkThread owns it.
 */
/** @var JoystickControl::motor_interface
 *      The interface to communicate with the motor thread.
 *      The joystick control gets it from the navigator network thread.
 */
/** @var JoystickControl::logger
 *      The logger to log something.
 *      The joystick control gets it from the navigator network thread.
 */


/** Constructor.
 * @param motor_interface the motor interface from the navigator network thread
 * @param kicker_interface the kicker interface from the navigator network thread
 * @param logger the logger from the navigator network thread
 * @param config the config from the navigator network thread
 * @param clock Clock instance
 */
JoystickControl::JoystickControl(MotorInterface * motor_interface,
                                 KickerInterface * kicker_interface,
                                 Logger * logger, Configuration * config,
                                 Clock * clock)
{
  this->motor_interface = motor_interface;
  this->kicker_interface = kicker_interface;
  this->logger = logger;
  this->config = config;
  this->clock = clock;
  actual_velocity = 0;
  last_joystick_axis_scale = 0;
  logger_modulo_counter = 0;
  logger_modulo_counter2 = 0;
  last_kick_time = clock->now();

  try
    {
      joystick_max_rotation     = config->get_float("navigator",
                                  "/joystick_control/max_rotation_velocity");
      joystick_max_velocity     = config->get_float("navigator",
                                  "/joystick_control/max_velocity");
    }
  catch (Exception &e)
    {
      e.append("Joystick control could not read all desired config values.");
      throw;
    }
}

/** Destructor. */
JoystickControl::~JoystickControl()
{}


/**  With this method the navigator network thread enqueues the kick command
 *   from the joystick tool to send it out to the kicker plugin.
 * @param left the left kicker command
 * @param center the center kicker command
 * @param right the right kicker command
 */
void
JoystickControl::enqueueKick(bool left, bool center, bool right)
{
  if ((clock->now() - last_kick_time).in_sec() > 1)
    {
      KickerInterface::KickMessage * msg =
        new KickerInterface::KickMessage (right, center, left, /* intensity */ 150);

      if (kicker_interface->has_writer())
        {
          kicker_interface->msgq_enqueue(msg);
        }

      logger->log_info("JoystickControl",
                       "kick left: %i, center: %i, right: %i", left, center,
                       right);

      last_kick_time = clock->now();
    }

}

/** With this method the navigator network thread enqueues the drive command
 *   from the joystick tool to send it out to the motor.
 *  @param forward the percentage of the maximum forward velocity
 *  @param sideward the percentage of the maximum velocity sideward
 *  @param rotation the maximum angular velocity with which the robot can rotate in rad/s
 *  @param max_velocity the maximum velocity with which the robot can translate in m/s
 */
void
JoystickControl::enqueueCommand(double forward, double sideward,
                                double rotation, double max_velocity)
{
  double axis_scale = sqrt(pow(forward, 2) + pow(sideward, 2));
  max_velocity *= joystick_max_velocity * axis_scale;

  MotorInterface::TransRotMessage * msg =
    new MotorInterface::TransRotMessage(max_velocity * forward, max_velocity * -sideward,
                                        -rotation * joystick_max_rotation);

  motor_interface->msgq_enqueue(msg);
}
