
/***************************************************************************
 *  motor_thread.cpp - Motor Thread
 *
 *  Generated: Son Jun 03 00:07:33 2007
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


#include <plugins/navigator/motor_thread.h>
#include <interfaces/motor.h>
#include <utils/math/angle.h>
#include <utils/time/clock.h>
#include <utils/time/watch.h>
#include <geometry/vector.h>

#include <vmc/LayerClasses/CvmcAPI.h>
#include <vmc/SupportClasses/Enums.h>

#include <cmath>
#include <unistd.h>

/* @class MotorThread plugins/navigator/motor_thread.h
 * The thread controlling the motors.
 * It gets some driving commands and calculates the rpms for
 * the wheels. It gets the real rpms from the motor controller as well.
 */
/* @var MotorThread::forward
 * The forward command.
 */
/* @var MotorThread::sideward
 * The sideward command.
 */
/* @var MotorThread::rotation
 * The rotation command.
 */
/* @var MotorThread::orbit_velocity
 * The orbit_velocity command.
 */

/** Constructor. */
MotorThread::MotorThread()
    : Thread("MotorThread", Thread::OPMODE_CONTINUOUS)
{
  apiObject = 0;
  motor_interface = 0;
  forward = 0;
  sideward = 0;
  rotation = 0;
  orbit_velocity = 0;
  orbit_radius = 0;
  orbit_sign = 1;
  orbit_angular_velocity = 0;
  alpha = 0;
  beta = 0;
  gamma = 0;
  alpha_ = 0;
  beta_ = 0;
  gamma_ = 0;
  last_alpha_rotations = 0;
  last_beta_rotations = 0;
  last_gamma_rotations = 0;
  last_alpha = 0;
  last_beta = 0;
  last_gamma = 0;
  odometry_distance = 0;
  logger_modulo_counter = 0;
  old_alpha = old_beta = old_gamma = 0.f;
  orbit_direction_x = 0;
  orbit_direction_y = 0;
  last_velocity = 0;
  current_max_velocity = 0;
  current_velocity = 0;
  time_difference = 0;
  // orbit_rotation_velocity = 0;
  clock = Clock::instance();
  stopped = true;
  start_time = false;
}

/** Destructor. */
MotorThread::~MotorThread()
{}

/** Initialize thread.
 * Here, the motor interface is opened.
 */
void
MotorThread::init()
{
  correction_x = config->get_float("/navigator/motor/correction_x");
  correction_y = config->get_float("/navigator/motor/correction_y");
  correction_rotation = config->get_float("/navigator/motor/correction_rotation");
  correction_translation = config->get_float("/navigator/motor/correction_translation");
  gear_reduction = config->get_float("/navigator/motor/gear_reduction");
  wheel_radius = config->get_float("/navigator/motor/wheel_radius");
  radius = config->get_float("/navigator/motor/radius");
  differential_part = config->get_float("/navigator/motor/vmc/differential_part");
  integral_part = config->get_float("/navigator/motor/vmc/integral_part");
  linear_part = config->get_float("/navigator/motor/vmc/linear_part");
  ticks = config->get_int("/navigator/motor/vmc/ticks");

  //calculate a factor for the rpms to obtain m/s
  translation_rpm_factor = /*correction_translation * */ gear_reduction * (1 / (wheel_radius * 2 * M_PI)  ) * 60;
  rotation_rpm_factor = radius * gear_reduction * (1 / (wheel_radius * 2 * M_PI)  ) * 60;

  if(config->exists("/navigator/motor/no_vmc"))
    {
      no_vmc = config->get_bool("/navigator/motor/no_vmc");
    }
  else
    {
      no_vmc = false;
    }

  if(!no_vmc)
    {
      apiObject = new VMC::CvmcAPI();

      apiObject->selectHardwareAdapter(VMC::RS232);
      if ( ! apiObject->selectDevice("/dev/ttyS0") )
        {
          throw Exception("MotorThread failed to open serial device VMC");
        }

      usleep(15000);
      apiObject->useVMC().Motor[0].EncoderTicks.Set(ticks);
      usleep(15000);
      apiObject->useVMC().Motor[1].EncoderTicks.Set(ticks);
      usleep(15000);
      apiObject->useVMC().Motor[2].EncoderTicks.Set(ticks);
      usleep(15000);
      apiObject->useVMC().Motor[0].VelocityControllerIntegralPart.Set(integral_part);
      usleep(15000);
      apiObject->useVMC().Motor[1].VelocityControllerIntegralPart.Set(integral_part);
      usleep(15000);
      apiObject->useVMC().Motor[2].VelocityControllerIntegralPart.Set(integral_part);
      usleep(15000);
      apiObject->useVMC().Motor[0].VelocityControllerDifferentialPart.Set(differential_part);
      usleep(15000);
      apiObject->useVMC().Motor[1].VelocityControllerDifferentialPart.Set(differential_part);
      usleep(15000);
      apiObject->useVMC().Motor[2].VelocityControllerDifferentialPart.Set(differential_part);
      usleep(15000);
      apiObject->useVMC().Motor[0].VelocityControllerLinearPart.Set(linear_part);
      usleep(15000);
      apiObject->useVMC().Motor[1].VelocityControllerLinearPart.Set(linear_part);
      usleep(15000);
      apiObject->useVMC().Motor[2].VelocityControllerLinearPart.Set(linear_part);
      usleep(15000);

      //to guarantee that the rotations are set
      logger->log_debug("MotorThread", "Wait for reply from the VMC.");
      while(0. == apiObject->useVMC().Motor[0].AbsolutRotations.getValue()
            && 0. == apiObject->useVMC().Motor[1].AbsolutRotations.getValue()
            && 0. == apiObject->useVMC().Motor[2].AbsolutRotations.getValue())
        {
          apiObject->useVMC().MotorRPMs.Set(10, 10, 10);
          usleep(15000);
        }
      logger->log_debug("MotorThread", "Got the reply from the VMC.");

      apiObject->useVMC().MotorRPMs.Set(0, 0, 0);
      usleep(15000);

      last_alpha_rotations = apiObject->useVMC().Motor[0].AbsolutRotations.getValue();
      last_beta_rotations = apiObject->useVMC().Motor[1].AbsolutRotations.getValue();
      last_gamma_rotations = apiObject->useVMC().Motor[2].AbsolutRotations.getValue();
    }
  try
    {
      motor_interface = interface_manager->open_for_writing<MotorInterface>("Motor");
    }
  catch (Exception& e)
    {
      logger->log_error("MotorThread", "Opening interface failed!");
      logger->log_error("MotorThread", e);
      throw;
    }
  motor_interface->set_motor_state(MotorInterface::MOTOR_ENABLED);
  motor_interface->set_drive_mode(0);
  motor_interface->set_odometry_path_length(0);
  motor_interface->set_odometry_position_x(0);
  motor_interface->set_odometry_position_y(0);
  motor_interface->set_odometry_orientation(0);
  motor_interface->write();
}

void
MotorThread::finalize()
{
  logger->log_info("MotorThread", "Finalizing thread %s", name());

  try
    {
      interface_manager->close(motor_interface);
    }
  catch (Exception& e)
    {
      logger->log_error("MotorThread", "Closing interface failed!");
      logger->log_error("MotorThread", e);
    }

  if(!no_vmc)
    {
      apiObject->closeDevice();
      delete apiObject;
    }
  logger->log_info("MotorThread", "End of Finalizing thread %s", name());
}


/** Here the driving commands are transformed to the RPMs
 *   for the three motors.
 */
void
MotorThread::loop()
{
  motor_interface->read();
  /*
    if ( motor_interface->msgq_size() > 0 ) {
    logger->log_error(name(), "We have %u messages", motor_interface->msgq_size());
    }*/
  if ( ! motor_interface->msgq_empty() )
    {
      if (motor_interface->msgq_first_is<MotorInterface::AcquireControlMessage>() )
        {
          MotorInterface::AcquireControlMessage* msg = motor_interface->msgq_first<MotorInterface::AcquireControlMessage>();

          if ( msg->thread_id() == 0 )
            {
              motor_interface->set_controller_thread_id(msg->sender_id());
              motor_interface->set_controller_thread_name(msg->sender());
            }
          else
            {
              motor_interface->set_controller_thread_id(msg->thread_id());
              motor_interface->set_controller_thread_name(msg->thread_name());
            }
          motor_interface->write();

          logger->log_debug(name(), "Thread %s (%lu) acquired motor control",
                            motor_interface->controller_thread_name(),
                            motor_interface->controller_thread_id());
        }
      else if ( motor_interface->msgq_first_is<MotorInterface::DriveRPMMessage>() )
        {
          MotorInterface::DriveRPMMessage* msg = motor_interface->msgq_first<MotorInterface::DriveRPMMessage>();

          if ( msg->sender_id() == motor_interface->controller_thread_id() )
            {
              alpha = msg->front_right();
              beta = msg->rear();
              gamma =msg->front_left();

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_RPM);
              motor_interface->write();
              start_time = true;
            }
          else
            {
              logger->log_warn(name(), "Warning, received DriveRPMMessage of thread %s (%lu), "
                               "but the motor is currently controlled by thread %s (%lu)",
                               msg->sender(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller_thread_id());
            }
        }
      else if ( motor_interface->msgq_first_is<MotorInterface::TransRotMessage>() )
        {
          MotorInterface::TransRotMessage* msg = motor_interface->msgq_first<MotorInterface::TransRotMessage>();

          if ( msg->sender_id() == motor_interface->controller_thread_id() )
            {
              //correction_factor * RPM for m/s * gear_factor
              forward = msg->vx() * translation_rpm_factor;
              sideward = msg->vy() * translation_rpm_factor;
              rotation = msg->omega() * rotation_rpm_factor;

              if(!no_vmc)
                {
                  forward *= correction_x;
                  forward *= correction_y;
                  forward *= correction_rotation;
                  //                  if(stopped)
                  //                    {
                  //                      last_alpha_rotations += apiObject->useVMC().Motor[0].AbsolutRotations.getValue();
                  //                      last_beta_rotations += apiObject->useVMC().Motor[1].AbsolutRotations.getValue();
                  //                      last_gamma_rotations += apiObject->useVMC().Motor[2].AbsolutRotations.getValue();
                  //                      stopped = false;
                  //                    }

                }
              current_max_velocity = sqrt(pow(forward, 2.) + pow(sideward, 2.));
              last_acceleration_time = clock->now();

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_TRANS_ROT);
              motor_interface->write();
              start_time = true;
              /*
                logger->log_debug(name(), "Processing TransRotMessage, VX: %f, "
                "VY: %f, Omega: %f",
                forward, sideward, rotation);
              */
            }
          else
            {
              logger->log_warn(name(), "Warning, received TransRotMessage of thread %s (%lu), "
                               "but the motor is currently controlled by thread %s (%lu)",
                               msg->sender(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller_thread_id());
            }
        }
      else if ( motor_interface->msgq_first_is<MotorInterface::TransMessage>() )
        {
          MotorInterface::TransMessage* msg = motor_interface->msgq_first<MotorInterface::TransMessage>();

          if ( msg->sender_id() == motor_interface->controller_thread_id() )
            {
              //correction_factor * RPM for m/s * gear_factor
              forward = msg->vx() * translation_rpm_factor;
              sideward = msg->vy() * translation_rpm_factor;
              if(!no_vmc)
                {
                  forward *= correction_x;
                  forward *= correction_y;
                  //                  if(stopped)
                  //                    {
                  //                      last_alpha_rotations += apiObject->useVMC().Motor[0].AbsolutRotations.getValue();
                  //                      last_beta_rotations += apiObject->useVMC().Motor[1].AbsolutRotations.getValue();
                  //                      last_gamma_rotations += apiObject->useVMC().Motor[2].AbsolutRotations.getValue();
                  //                      stopped = false;
                  //                    }
                }
              current_max_velocity = sqrt(pow(forward, 2.) + pow(sideward, 2.));
              last_acceleration_time = clock->now();
              rotation = 0;
              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_TRANS);
              motor_interface->write();
              start_time = true;
              logger->log_info("MotorThread", " received trans message x = %f, y = %f ", msg->vx(), msg->vy());
            }
          else
            {
              logger->log_warn(name(), "Warning, received TransMessage of thread %s (%lu), "
                               "but the motor is currently controlled by thread %s (%lu)",
                               msg->sender(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller_thread_id());
            }
        }
      else if ( motor_interface->msgq_first_is<MotorInterface::RotMessage>() )
        {
          MotorInterface::RotMessage* msg = motor_interface->msgq_first<MotorInterface::RotMessage>();

          if ( msg->sender_id() == motor_interface->controller_thread_id() )
            {
              //correction_factor * RPM for m/s * gear_factor
              rotation = msg->omega() * rotation_rpm_factor;
              if(!no_vmc)
                {
                  forward *= correction_rotation;
                  //                  if(stopped)
                  //                    {
                  //                      last_alpha_rotations += apiObject->useVMC().Motor[0].AbsolutRotations.getValue();
                  //                      last_beta_rotations += apiObject->useVMC().Motor[1].AbsolutRotations.getValue();
                  //                      last_gamma_rotations += apiObject->useVMC().Motor[2].AbsolutRotations.getValue();
                  //                      stopped = false;
                  //                    }
                }
              sideward = 0;
              forward = 0;
              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_ROT);
              motor_interface->write();
              start_time = true;
            }
          else
            {
              logger->log_warn(name(), "Warning, received RotMessage of thread %s (%lu), "
                               "but the motor is currently controlled by thread %s (%lu)",
                               msg->sender(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller_thread_id());
            }
        }
      else if ( motor_interface->msgq_first_is<MotorInterface::OrbitMessage>() )
        {
          MotorInterface::OrbitMessage* msg = motor_interface->msgq_first<MotorInterface::OrbitMessage>();

          if ( msg->sender_id() == motor_interface->controller_thread_id() )
            {
              orbit_center.x(msg->px());
              orbit_center.y(msg->py());
              orbit_position.x(0.);
              orbit_position.y(0.);
              orbit_angular_velocity = msg->omega(); //neg clockwise, pos counterclockwise
              orbit_radius = orbit_center.length(); //m
              orbit_direction = orbit_center;
              if(orbit_angular_velocity < 0)
                {
                  orbit_sign = 1;
                  orbit_angular_velocity *= -1;
                }
              else
                {
                  orbit_sign = -1;
                }
              orbit_velocity = orbit_radius * orbit_angular_velocity; //m/s

              //     logger->log_info("MotorThread", "-----> orbit_center.x() : %f", orbit_center.x());
              //     logger->log_info("MotorThread", "-----> orbit_center.y() : %f", orbit_center.y());

              rotation = 0;

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_ORBIT);
              motor_interface->write();
              start_time = true;
              //  last_time = clock->now();
              //              if(!no_vmc)
              //                {
              //                  if(stopped)
              //                    {
              //                      last_alpha_rotations += apiObject->useVMC().Motor[0].AbsolutRotations.getValue();
              //                      last_beta_rotations += apiObject->useVMC().Motor[1].AbsolutRotations.getValue();
              //                      last_gamma_rotations += apiObject->useVMC().Motor[2].AbsolutRotations.getValue();
              //                      stopped = false;
              //                    }
              //                }
              /*
                logger->log_debug(name(), "Processing OrbitMessage, PX: %f, "
                "PY: %f, Omega: %f",
                orbit_center.x(), orbit_center.y(), orbit_angular_velocity);
              */
            }
        }
      else if (motor_interface->msgq_first_is<MotorInterface::ResetOdometryMessage>() )
        {
          odometry_distance = 0.;
          motor_interface->set_odometry_path_length(0.);
          motor_interface->set_odometry_position_x(0.);
          motor_interface->set_odometry_position_y(0.);
          motor_interface->set_odometry_orientation(0.);
          motor_interface->write();
        }
      else if (motor_interface->msgq_first_is<MotorInterface::SetMotorStateMessage>() )
        {
          MotorInterface::SetMotorStateMessage* msg = motor_interface->msgq_first<MotorInterface::SetMotorStateMessage>();
          // we really want to make sure that we got a correct message with useful values
          // thus we check every single value
          if ( msg->motor_state() == MotorInterface::MOTOR_ENABLED )
            {
              motor_interface->set_motor_state(MotorInterface::MOTOR_ENABLED);
              logger->log_info(name(), "Enabling motor control");
            }
          else if ( msg->motor_state() == MotorInterface::MOTOR_DISABLED )
            {
              motor_interface->set_motor_state(MotorInterface::MOTOR_DISABLED);
              logger->log_info(name(), "Disabling motor control");
            }
          else
            {
              logger->log_error(name(), "SetMotorStateMessage received with illegal value: %u",
                                msg->motor_state());
            }
        }
      else
        {
          logger->log_error("MotorThread", "Message of invalid type received from %s", motor_interface->msgq_first()->sender());
        }

      motor_interface->msgq_pop();
    }

  motor_interface->read();

  //if a new command is received and the robot is not moving than last_time has to be 0
  if(start_time && motor_interface->vx() == 0 && motor_interface->vy() == 0)
    {
      last_time = clock->now();
      time_difference = (last_time - last_time).in_sec();
    }
  else
    {
      time_difference = (clock->now() - last_time).in_sec();
      last_time = clock->now();
    }

  start_time = false;

  if(motor_interface->drive_mode() != MotorInterface::DRIVE_MODE_RPM)
    {
      if(motor_interface->drive_mode() == MotorInterface::DRIVE_MODE_ORBIT)
        {
          /*
            logger->log_info("MotorThread", " orbit_center.x() %f ",  orbit_center.x());
            logger->log_info("MotorThread", " orbit_center.y() %f ",  orbit_center.y());
            logger->log_info("MotorThread", " orbit_position.x() %f ",  orbit_center.x());
            logger->log_info("MotorThread", " orbit_position.y() %f ",  orbit_center.y());
            logger->log_info("MotorThread", " orbit_direction.length() %f ", orbit_direction.length());
          */
          //calculate the tangent to the orbit passing through the actual position
          if(orbit_direction.length() != 0)
            {
              Vector old_direction = orbit_direction;
              orbit_direction = orbit_center - orbit_position;
              double alpha = 0;
              // logger->log_info("MotorThread", " orbit_direction.length() %f ", orbit_direction.length());
              //  logger->log_info("MotorThread", " orbit_radius %f ", orbit_radius);
              if(orbit_radius <= orbit_direction.length())
                {
                  alpha = asin(orbit_radius / orbit_direction.length()) * orbit_sign;
                  //   logger->log_info("MotorThread", " alpha %f ", alpha);

                  orbit_direction.rotate_z(alpha);
                }
              else
                {
                  orbit_direction = old_direction;
                }

              orbit_direction.unit();
              forward = orbit_direction.x() * orbit_velocity * translation_rpm_factor;
              sideward = orbit_direction.y() * orbit_velocity * translation_rpm_factor;
            }
          else if(orbit_direction.length() != 0 && orbit_position.length() == 0)
            {
              forward = orbit_direction.x() * orbit_velocity * translation_rpm_factor;
              sideward = orbit_direction.y() * orbit_velocity * translation_rpm_factor;
            }
          else
            {
              sideward = 0.;
              forward = 0.;
            }
          //the positions has to be calculated after the velocities are calculated
          //because if not, orbit_radius == orbit_direction.length() will not hold
          //and then the robot will first intersect the orbit until it will start drive on the orbit
          orbit_position.x(orbit_position.x() + motor_interface->vx() * time_difference);
          orbit_position.y(orbit_position.y() + motor_interface->vy() * time_difference);

        }
      else //acceleration
        {
          double velocity = sqrt(pow(forward, 2.) + pow(sideward, 2.));
          if(velocity > 0.)
            {
              forward /= velocity;
              sideward /= velocity;
              if(last_velocity > 0.)
                {
                  velocity = last_velocity;
                }
              else
                {
                  velocity = 0;
                }
            }

          if(velocity < current_max_velocity)
            {
              velocity += clock->elapsed(&last_acceleration_time) * 0.2 * translation_rpm_factor; //m/s^2
              last_acceleration_time = clock->now();
            }
          else if(velocity >= current_max_velocity)
            {
              velocity = current_max_velocity;
              last_acceleration_time = clock->now();
            }
          last_velocity = velocity;
          forward *= velocity;
          sideward *= velocity;
        }

      alpha  = 	((cos(M_PI/3.)				* -sideward) - (sin(M_PI/3.)				* forward)) - rotation;
      beta   = 	((cos(M_PI) 					* -sideward) - (sin(M_PI) 					* forward)) - rotation;
      gamma  = 	((cos((5./3.) * M_PI) 	* -sideward) - (sin((5./3.) * M_PI) 	* forward)) - rotation;


      //    logger->log_info("MotorThread", " sideward : %f, forward: %f, rotation: %f",
      //                     sideward, forward, rotation);
    }

  //needful for accurate acceleration
  if(alpha == 0. && beta == 0. && gamma == 0.)
    {
      forward = 0;
      sideward = 0;
      rotation = 0;
      last_velocity = 0;
    }

  if(!no_vmc)
    {
      if ( motor_interface->motor_state() == MotorInterface::MOTOR_ENABLED )
        {
          if(alpha != 0 || beta != 0 || gamma != 0)
            {
              apiObject->useVMC().MotorRPMs.Set(alpha, beta, gamma);
            }
          usleep(10000);
        }

      else if( (alpha == 0.) && (beta == 0.) && (gamma == 0.) && ( (round(alpha_ )!= 0.) || (round(beta_) != 0.) || (round(gamma_) != 0.)))
        {
          logger->log_info("MotorThread",  "braking");

          apiObject->useVMC().MotorRPMs.Set(0, 0, 0);
        }
      else if ((alpha_ == 0.) && (beta_ == 0.) && (gamma_ == 0.))
        {
          stopped = true;
        }



      //  logger->log_info("MotorThread", " alpha : %f, beta: %f, gamma: %f, rotation: %f"
      //                   " old_alpha : %f, old_beta: %f, old_gamma: %f",
      //                   alpha, beta, gamma, rotation,
      //                   old_alpha, old_beta, old_gamma );
      /*
        //for saving too much logging
        if ( (alpha != old_alpha) || (beta != old_beta) || (gamma != old_gamma) )
          {
            logger->log_info("MotorThread", " alpha : %f, beta: %f, gamma: %f, rotation: %f",
                             alpha, beta, gamma, rotation);
       
            logger->log_info("MotorThread", " sideward : %f, forward: %f, rotation: %f",
                             sideward, forward, rotation);
            old_alpha = alpha;
            old_beta  = beta;
            old_gamma = gamma;
          }
      */
      /*
        logger->log_info("MotorThread", "RPM1: %f  RPM2: %f  RPM3: %f ", 
        apiObject->useVMC().Motor[0].ActualRPM.getValue(),
        apiObject->useVMC().Motor[1].ActualRPM.getValue(),
        apiObject->useVMC().Motor[2].ActualRPM.getValue());
      */

      //  	logger->log_info("MotorThread",  "odometry");
      //AbsolutRotations provides encoder ticks
      alpha_ = apiObject->useVMC().Motor[0].AbsolutRotations.getValue() / ticks;
      beta_ = apiObject->useVMC().Motor[1].AbsolutRotations.getValue() / ticks;
      gamma_ = apiObject->useVMC().Motor[2].AbsolutRotations.getValue() / ticks;

      //   logger->log_info("MotorThread", " time_diff : %f",
      //                   time_diff);

      alpha_ = alpha_ - last_alpha_rotations;
      beta_ = beta_ - last_beta_rotations;
      gamma_ = gamma_ - last_gamma_rotations;

      /*
       * At 2147483647.0 and -2147483647.0 there can happen an overflow.
       * But this will only happen if the robot drives about 1,000 km.
       * So, we don't care, because the AbsolutRotations will be reset to 0 at every switching off.
       */

      last_alpha_rotations += alpha_;
      last_beta_rotations += beta_;
      last_gamma_rotations += gamma_;

      //get RPMs
      // double time_diff = (clock->now() - last_time).in_sec();
      alpha_ = (alpha_ / time_difference) * 60;
      beta_ = (beta_ / time_difference) * 60;
      gamma_ = (gamma_ / time_difference) * 60;
    }
  else
    {
      alpha_ =  last_alpha;
      beta_ =  last_beta;
      gamma_ =  last_gamma;
      last_alpha = alpha;
      last_beta  = beta;
      last_gamma = gamma;
    }
  double rotation_ = -(alpha_ + beta_ + gamma_) / 3.;

  //  logger->log_info("MotorThread", " alpha_ : %f, beta_: %f, gamma_: %f, rotation_: %f",
  //                   alpha_, beta_, gamma_, rotation_);
  alpha_ += rotation_;  //right
  beta_ += rotation_;
  gamma_ += rotation_;

  double  sideward_ = -(alpha_ * cos(M_PI/3.) + beta_ * cos(M_PI) + gamma_ * cos((5./3.) * M_PI)) * (2./3.);
  double  forward_ = -(alpha_ * sin(M_PI/3.) + beta_ * sin(M_PI) + gamma_ * sin((5./3.) * M_PI)) * (2./3.);

  sideward_ /= translation_rpm_factor;
  forward_ /= translation_rpm_factor;
  rotation_ /= rotation_rpm_factor;
  //     logger->log_info("MotorThread", " sideward_ : %f, forward_: %f, rotation_: %f",
  //                      sideward_, forward_, rotation_);
  double velocity_ = sqrt(pow(sideward_, 2.) + pow(forward_, 2.));
  //double time_difference_odometry = (clock->now() - last_time_odometry).in_sec();
  //last_time_odometry = clock->now();
  double odometry_difference = velocity_ * time_difference;//_odometry;

  odometry_distance += odometry_difference;

  Vector new_position;
  Vector old_position;
  Vector bend_vector;

  old_position.x(motor_interface->odometry_position_x());
  old_position.y(motor_interface->odometry_position_y());
  old_position.z(0.f);

  //calculation of the odometry

  //rotation and translation
  if((rotation_ > 0.000000005 || rotation_ < -0.0000000005) && odometry_difference != 0)
    {
      logger->log_info("MotorThread", " rotation and translation ");
      //recalculation of the arc
      double turned_angle = rotation_ * time_difference;//_odometry;

      double bend_radius = odometry_difference / fabs(turned_angle);

      if(turned_angle < 0)
        {
          bend_vector.y(bend_radius * sin(atan2(sideward_, forward_) + motor_interface->odometry_orientation() + deg2rad(90)));
          bend_vector.x(bend_radius * cos(atan2(sideward_, forward_) + motor_interface->odometry_orientation() + deg2rad(90)));
        }
      else
        {
          bend_vector.y(bend_radius * sin(atan2(sideward_, forward_) + motor_interface->odometry_orientation() - deg2rad(90)));
          bend_vector.x(bend_radius * cos(atan2(sideward_, forward_) + motor_interface->odometry_orientation() - deg2rad(90)));
        }
      bend_vector.z(0.);

      new_position = old_position - bend_vector;
      bend_vector.rotate_z(turned_angle);

      new_position += bend_vector;
      /*
      logger->log_info("MotorThread", " odometry with rotation ");
      logger->log_info("MotorThread", " turned_angle %f", turned_angle);
      logger->log_info("MotorThread", " rotation_ %f", rotation_);
      logger->log_info("MotorThread", " bend_radius %f", bend_radius);
      logger->log_info("MotorThread", " bend_vector.length() %f", bend_vector.length());
      logger->log_info("MotorThread", " odometry_difference %f", odometry_difference);
      logger->log_info("MotorThread", " odometry_distance %f", odometry_distance);
      logger->log_info("MotorThread", " motor_interface->getOdometryOrientation() %f ", motor_interface->getOdometryOrientation());
      logger->log_info("MotorThread", " old_position.x() %f ", old_position.x());
      logger->log_info("MotorThread", " old_position.y() %f ", old_position.y());
      logger->log_info("MotorThread", " time_difference_odometry %f ", time_difference_odometry);
      */
    } //rotation without translation
  else if((rotation_ > 0.000000005 || rotation_ < -0.0000000005) && odometry_difference == 0)
    {
      logger->log_info("MotorThread", " rotation without translation ");
      new_position = old_position;
    }
  else //translation without rotation
    {
      /*
            logger->log_info("MotorThread", " odometry without rotation ");
            logger->log_info("MotorThread", " odometry_difference %f", odometry_difference);
            logger->log_info("MotorThread", " old_position.x() %f ", old_position.x());
            logger->log_info("MotorThread", " old_position.y() %f ", old_position.y());
            logger->log_info("MotorThread", " forward_ %f ", forward_ * translation_rpm_factor);
            logger->log_info("MotorThread", " forward %f ", forward);
            logger->log_info("MotorThread", " sideward_ %f ", sideward_ * translation_rpm_factor);
            logger->log_info("MotorThread", " sideward %f ", sideward);
            logger->log_info("MotorThread", " time_difference %f ", time_difference);
      */
      new_position.x(old_position.x() + odometry_difference * cos(atan2(sideward_, forward_) + motor_interface->odometry_orientation()));
      new_position.y(old_position.y() + odometry_difference * sin(atan2(sideward_, forward_) + motor_interface->odometry_orientation()));
      /*
            logger->log_info("MotorThread", " cos(atan2(sideward_, forward_) + motor_interface->odometry_orientation()) %f ", cos(atan2(sideward_, forward_) + motor_interface->odometry_orientation()));
            logger->log_info("MotorThread", " sin(atan2(sideward_, forward_) + motor_interface->odometry_orientation()) %f ", sin(atan2(sideward_, forward_) + motor_interface->odometry_orientation()));
            logger->log_info("MotorThread", " new_position.x() %f ", new_position.x());
            logger->log_info("MotorThread", " new_position.y() %f ", new_position.y());*/
    }

  motor_interface->set_odometry_path_length(odometry_distance);
  motor_interface->set_odometry_position_x(new_position.x());
  motor_interface->set_odometry_position_y(new_position.y());
  motor_interface->set_odometry_orientation(motor_interface->odometry_orientation() + rotation_ * time_difference);//_odometry);
  /*
    logger->log_info("MotorThread", " time_difference %f ", time_difference);
    logger->log_info("MotorThread", " rotation_ %f ", rotation_);
    logger->log_info("MotorThread", " motor_interface->odometry_orientation() %f ", motor_interface->odometry_orientation());
    logger->log_info("MotorThread", " --------------------------------------------------------------------------");
  */
  motor_interface->set_right_rpm((int)(alpha_ - rotation_));
  motor_interface->set_rear_rpm((int)(beta_ - rotation_));
  motor_interface->set_left_rpm((int)(gamma_ - rotation_));

  motor_interface->set_vx(forward_);
  motor_interface->set_vy(sideward_ );
  motor_interface->set_omega(rotation_ );
  // usleep(1000000);
  motor_interface->write();
}//loop
