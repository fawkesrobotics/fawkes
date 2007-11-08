
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
  motor_interface = NULL;
  forward = 0;
  sideward = 0;
  rotation = 0;
  orbit_velocity = 0;
  point_x = 0;
  point_y = 0;
  orbit_angular_velocity = 0;
  alpha = 0;
  beta = 0;
  gamma = 0;
  alpha_ = 0;
  beta_ = 0;
  gamma_ = 0;
  odometry_distance = 0;
  logger_modulo_counter = 0;
  old_alpha = old_beta = old_gamma = 0.f;
  orbit_direction_x = 0;
  orbit_direction_y = 0;
  last_velocity = 0;
  current_max_velocity = 0;
  current_velocity = 0;
  // orbit_rotation_velocity = 0;
  clock = Clock::instance();
  stop_watch = new Watch(clock);
  stop_time = new Time(clock);
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
  correction_x = config->get_float("navigator", "/motor/correction_x");
  correction_y = config->get_float("navigator", "/motor/correction_y");
  correction_rotation = config->get_float("navigator", "/motor/correction_rotation");

  if(config->exists("navigator", "/motor/no_vmc"))
    {
      no_vmc = config->get_bool("navigator", "/motor/no_vmc");
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
      else if ( motor_interface->msgq_first_is<MotorInterface::TransRotMessage>() )
        {
          MotorInterface::TransRotMessage* msg = motor_interface->msgq_first<MotorInterface::TransRotMessage>();

          if ( msg->sender_id() == motor_interface->controller_thread_id() )
            {
              //correction_factor * RPM for m/s * gear_factor
              forward = msg->vx() * (1.1 * 187.978289782 * 8.656666666666666667);
              sideward = msg->vy() * (1.1 * 187.978289782 * 8.656666666666666667);
              rotation = msg->omega() * (0.188 * 187.978289782 * 8.656666666666666667);

              current_max_velocity = sqrt(pow(forward, 2.) + pow(sideward, 2.));
              last_acceleration_time = clock->now();

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_TRANS_ROT);
              motor_interface->write();
              last_time = clock->now();
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
              last_time = clock->now();
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
      else if ( motor_interface->msgq_first_is<MotorInterface::TransMessage>() )
        {
          MotorInterface::TransMessage* msg = motor_interface->msgq_first<MotorInterface::TransMessage>();

          if ( msg->sender_id() == motor_interface->controller_thread_id() )
            {
              //correction_factor * RPM for m/s * gear_factor
              forward = msg->vx() * (1.1 * 187.978289782 * 8.656666666666666667);
              sideward = msg->vy() * (1.1 * 187.978289782 * 8.656666666666666667);
              current_max_velocity = sqrt(pow(forward, 2.) + pow(sideward, 2.));
              last_acceleration_time = clock->now();
              rotation = 0;
              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_TRANS);
              motor_interface->write();
              last_time = clock->now();
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
              rotation = msg->omega() * (0.188 * 187.978289782 * 8.656666666666666667);
              sideward = 0;
              forward = 0;
              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_ROT);
              motor_interface->write();
              last_time = clock->now();
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
              point_x = msg->px();
              point_y = msg->py();
              orbit_angular_velocity = -msg->omega(); //neg clockwise, pos counterclockwise

              double distance = sqrt(pow(point_x, 2.) + pow(point_y, 2.)); //m

              orbit_velocity = distance * orbit_angular_velocity; //m/s

              orbit_direction_x = point_x;
              orbit_direction_y = -point_y;

              double direction_length = sqrt(pow(orbit_direction_x, 2.) + pow(orbit_direction_y, 2.));

              if(direction_length != 0.)
                {
                  orbit_direction_y /= direction_length;
                  orbit_direction_x /= direction_length;
                }
              else
                {
                  orbit_direction_y = 0.;
                  orbit_direction_x = 0.;
                }

              rotation = 0;

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_ORBIT);
              motor_interface->write();
              last_time = clock->now();

              /*
                logger->log_debug(name(), "Processing OrbitMessage, PX: %f, "
                "PY: %f, Omega: %f",
                point_x, point_y, orbit_angular_velocity);
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

  //  motor_interface->read();

  if(motor_interface->drive_mode() == MotorInterface::DRIVE_MODE_ORBIT)
    {
      double time_difference = (clock->now() - last_time).in_sec();
      last_time = clock->now();

      double direction_change = time_difference * orbit_angular_velocity;

      if(direction_change == 0.)
        {
          sideward = 0.;
          forward = 0.;
        }
      else
        {
          orbit_direction_x = orbit_direction_x * cos(direction_change) - orbit_direction_y * sin(direction_change);
          orbit_direction_y = orbit_direction_x * sin (direction_change) + orbit_direction_y * cos(direction_change);

          forward = orbit_direction_y * orbit_velocity * (1.1 * 187.978289782 * 8.656666666666666667);
          sideward = orbit_direction_x * orbit_velocity * (1.1 * 187.978289782 * 8.656666666666666667);
        }
    }

  if(motor_interface->drive_mode() != MotorInterface::DRIVE_MODE_RPM)
    {
      forward *= correction_x;  //must not to be negative, since then it will oscillate
      sideward *= correction_y;
      rotation *= correction_rotation;

      //acceleration
      if(motor_interface->drive_mode() != MotorInterface::DRIVE_MODE_ORBIT)
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
              velocity += clock->elapsed(&last_acceleration_time) * 0.2 * 1.1 * 187.978289782 * 8.656666666666666667; //m/s^2
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

      //  double dir = 60;
      alpha  = ((cos(M_PI/3./*deg2rad(dir)*/)       * -sideward) - (sin(M_PI/3./*deg2rad(dir)*/)       * forward))  - rotation;
      beta   = ((cos(M_PI/*deg2rad(dir + 120)*/) * -sideward) - (sin(M_PI/*deg2rad(dir + 120)*/) * forward))  - rotation;
      gamma  = ((cos((5./3.) * M_PI /*deg2rad(dir + 240)*/) * -sideward) - (sin((5./3.) * M_PI /*deg2rad(dir + 240)*/) * forward)) - rotation;


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
              logger->log_info("MotorThread", "drive");
              apiObject->useVMC().MotorRPMs.Set(alpha, beta, gamma);
              if(stop_watch->watch_time().in_sec() > 0.)
                {
                  stop_watch->stop(stop_time);
                  stop_watch->reset();
                }
            }
          else if(stop_watch->watch_time().in_sec() <= 0.)
            {
              stop_watch->start(stop_time);
            }
          else if(stop_watch->watch_time().in_sec() <= 0.5) //send stop for 0.5 seconds
            {
              logger->log_info("MotorThread", "stop");
              apiObject->useVMC().MotorRPMs.Set(0, 0, 0);
            }
          usleep(15000);
        }
    }

  //  logger->log_info("MotorThread", " alpha : %f, beta: %f, gamma: %f, rotation: %f"
  //                   " old_alpha : %f, old_beta: %f, old_gamma: %f",
  //                   alpha, beta, gamma, rotation,
  //                   old_alpha, old_beta, old_gamma );

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

  /*
    logger->log_info("MotorThread", "RPM1: %f  RPM2: %f  RPM3: %f ", 
    apiObject->useVMC().Motor[0].ActualRPM.getValue(),
    apiObject->useVMC().Motor[1].ActualRPM.getValue(),
    apiObject->useVMC().Motor[2].ActualRPM.getValue());
  */

  if(!no_vmc)
    {
      //   logger->log_info("MotorThread",  "(alpha_ != 0.) %i", (alpha_ != 0.));
      //   logger->log_info("MotorThread",  "(beta_ != 0.) %i", (beta_ != 0.));
      //   logger->log_info("MotorThread",  "(gamma_ != 0.) %i", (gamma_ != 0.));
      if( (alpha == 0.) && (beta == 0.) && (gamma == 0.) && ( (round(alpha_ )!= 0.) || (round(beta_) != 0.) || (round(gamma_) != 0.)))
        {
          logger->log_info("MotorThread",  "update");
          logger->log_info("MotorThread",  "update %i", apiObject->useVMC().Motor[0].ActualRPM.Update());
          usleep(25000);
          logger->log_info("MotorThread",  "update %i", apiObject->useVMC().Motor[1].ActualRPM.Update());
          usleep(25000);
          logger->log_info("MotorThread",  "update %i", apiObject->useVMC().Motor[2].ActualRPM.Update());
          usleep(25000);
        }

      //  if(( (alpha != 0.) || (beta != 0.) || (gamma != 0.)) || ((alpha_ != 0.) || (beta_ != 0.) || (gamma_ != 0.)))
      {

        //  	logger->log_info("MotorThread",  "odometry");
        alpha_ = apiObject->useVMC().Motor[0].ActualRPM.getValue();
        usleep(50000);
        beta_ = apiObject->useVMC().Motor[1].ActualRPM.getValue();
        usleep(50000);
        gamma_ = apiObject->useVMC().Motor[2].ActualRPM.getValue();
        usleep(50000);
      }
    }
  else
    {
      alpha_ = alpha;
      beta_ = beta;
      gamma_ = gamma;
    }
  double rotation_ = -(alpha_ + beta_ + gamma_) / 3.;

  //  logger->log_info("MotorThread", " alpha_ : %f, beta_: %f, gamma_: %f, rotation_: %f",
  //                   alpha_, beta_, gamma_, rotation_);
  alpha_ += rotation_;  //right
  beta_ += rotation_;
  gamma_ += rotation_;

  // double dir = 60;
  double  sideward_ = -(alpha_ * cos(M_PI/3./*deg2rad(dir)*/) + beta_ * cos(M_PI/*deg2rad(dir + 120)*/) + gamma_ * cos((5./3.) * M_PI /*deg2rad(dir + 240)*/)) * (2./3.);
  double  forward_ = -(alpha_ * sin(M_PI/3./*deg2rad(dir)*/) + beta_ * sin(M_PI/*deg2rad(dir + 120)*/) + gamma_ * sin((5./3.) * M_PI /*deg2rad(dir + 240)*/)) * (2./3.);

  sideward_ /= 1.1 * 187.978289782 * 8.656666666666666667;
  forward_ /= 1.1 * 187.978289782 * 8.656666666666666667;
  rotation_ /= 0.188 * 187.978289782 * 8.656666666666666667;
  //     logger->log_info("MotorThread", " sideward_ : %f, forward_: %f, rotation_: %f",
  //                      sideward_, forward_, rotation_);
  double velocity_ = sqrt(pow(sideward_, 2.) + pow(forward_, 2.));
  double time_difference_odometry = (clock->now() - last_time_odometry).in_sec();
  last_time_odometry = clock->now();
  double odometry_difference = velocity_ * time_difference_odometry;
  odometry_distance += odometry_difference;

  Vector new_position;
  Vector old_position;
  Vector bend_vector;

  old_position.x(motor_interface->odometry_position_x());
  old_position.y(motor_interface->odometry_position_y());
  old_position.z(0.f);

  //calculation of the odometry
  if(rotation_ > 0.000000005 || rotation_ < -0.0000000005)
    {
      //recalculation of the arc
      double turned_angle = rotation_ * time_difference_odometry;

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
    }
  else
    {
      /*
       logger->log_info("MotorThread", " odometry without rotation ");
       logger->log_info("MotorThread", " odometry_difference %f", odometry_difference);
       logger->log_info("MotorThread", " old_position.x() %f ", old_position.x());
       logger->log_info("MotorThread", " old_position.y() %f ", old_position.y());
       logger->log_info("MotorThread", " forward_ %f ", forward_ * 1.1 * 187.978289782 * 8.656666666666666667);
       logger->log_info("MotorThread", " forward %f ", forward);
       logger->log_info("MotorThread", " sideward_ %f ", sideward_ * 1.1 * 187.978289782 * 8.656666666666666667);
       logger->log_info("MotorThread", " sideward %f ", sideward);
       logger->log_info("MotorThread", " time_difference_odometry %f ", time_difference_odometry);
       */
      new_position.x(old_position.x() + odometry_difference * cos(atan2(sideward_, forward_) + motor_interface->odometry_orientation()));
      new_position.y(old_position.y() + odometry_difference * sin(atan2(sideward_, forward_) + motor_interface->odometry_orientation()));
    }

  motor_interface->set_odometry_path_length(odometry_distance);
  motor_interface->set_odometry_position_x(new_position.x());
  motor_interface->set_odometry_position_y(new_position.y());
  motor_interface->set_odometry_orientation(motor_interface->odometry_orientation() + rotation_ * time_difference_odometry);
  if(!no_vmc)
    {
      motor_interface->set_right_rpm((int)apiObject->useVMC().Motor[0].ActualRPM.getValue());
      motor_interface->set_rear_rpm((int)apiObject->useVMC().Motor[1].ActualRPM.getValue());
      motor_interface->set_left_rpm((int)apiObject->useVMC().Motor[2].ActualRPM.getValue());
    }
  else
    {
      motor_interface->set_right_rpm((int)alpha);
      motor_interface->set_rear_rpm((int)beta);
      motor_interface->set_left_rpm((int)gamma);
    }
  motor_interface->set_vx((int)(forward_));
  motor_interface->set_vy((int)(sideward_ ));
  motor_interface->set_omega((int)(rotation_ / (0.188 * 187.978289782 * 8.656666666666666667)));

  motor_interface->write();
}//loop
