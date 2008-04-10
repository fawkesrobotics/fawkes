
/***************************************************************************
 *  motor_thread.cpp - Motor Thread
 *
 *  Generated: Son Jun 03 00:07:33 2007
 *  Copyright  2007  Martin Liebenberg
 *             2008  Daniel Beck
 *
 *  $Id$
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

#include <plugins/navigator/motor_thread.h>
#include <plugins/navigator/robot_motion/omni_motion_model.h>
#include <plugins/navigator/robot_motion/linear_velocity_controller.h>
#include <interfaces/motor.h>
#include <geometry/hom_vector.h>

#include <vmc/LayerClasses/CvmcAPI.h>
#include <vmc/SupportClasses/Enums.h>

#include <cmath>
#include <unistd.h>

/** @class MotorThread plugins/navigator/motor_thread.h
 * The thread controlling the motors.
 * It gets some driving commands and calculates the rpms for
 * the wheels. It gets the real rpms from the motor controller as well.
 * @author Martin Liebenberg
 * @author Daniel Beck
 */

/** Constructor. */
MotorThread::MotorThread()
  : Thread("MotorThread", Thread::OPMODE_CONTINUOUS)
{
  vmc_api = 0;
  motor_interface = 0;

  no_vmc = true;
  first_loop = true;

  // desired velocities
  vx_des    = 0.0;
  vy_des    = 0.0;
  omega_des = 0.0;

  // actual velocities
  vx_act    = 0.0;
  vy_act    = 0.0;
  omega_act = 0.0;

  // odometric movement in last step
  odo_delta_x   = 0.0;
  odo_delta_y   = 0.0;
  odo_delta_phi = 0.0;

  loop_time = 22000;
}

/** Destructor. */
MotorThread::~MotorThread()
{
}

/** Initialize thread.
 * Here, the motor interface is opened.
 */
void
MotorThread::init()
{
  // get config values
  try
    {
      acceleration_factor = config->get_float("/navigator/motor/acceleration_factor");
      correction_x = config->get_float("/navigator/motor/correction_x");
      correction_y = config->get_float("/navigator/motor/correction_y");
      correction_rotation = config->get_float("/navigator/motor/correction_rotation");
      correction_translation = config->get_float("/navigator/motor/correction_translation");
      gear_reduction = 1 / config->get_float("/navigator/motor/gear_reduction");
      wheel_radius = 1000.0 * config->get_float("/navigator/motor/wheel_radius");
      radius = 1000.0 * config->get_float("/navigator/motor/radius");
      differential_part = config->get_float("/navigator/motor/vmc/differential_part");
      integral_part = config->get_float("/navigator/motor/vmc/integral_part");
      linear_part = config->get_float("/navigator/motor/vmc/linear_part");
      ticks = config->get_int("/navigator/motor/vmc/ticks");
    }
  catch (Exception& e)
    {
      e.append("MotorThread::init() failed since config parameters are missing");
      throw;
    }

  try
    {
      vmc_port = strdup( config->get_string("/navigator/motor/vmc/port").c_str() );
    }
  catch (Exception& e)
    {
      logger->log_info(name(), "No VMC port specified. Assuming '/dev/ttyS0'");
      vmc_port = strdup("/dev/ttyS0");
    }

  if( config->exists("/navigator/motor/no_vmc") )
    {
      no_vmc = config->get_bool("/navigator/motor/no_vmc");
      logger->log_info(name(), "Running in simulation mode");
    }
  else
    {
      no_vmc = false;
    }

  // setup VMC
  if( !no_vmc )
    {
      vmc_api = new VMC::CvmcAPI();
      vmc_api->selectHardwareAdapter(VMC::RS232);

      if ( !vmc_api->selectDevice(vmc_port) )
        {
          throw Exception("MotorThread failed to open serial device VMC");
        }

      usleep(15000);
      vmc_api->useVMC().Motor[0].EncoderTicks.Set(ticks);
      usleep(15000);
      vmc_api->useVMC().Motor[1].EncoderTicks.Set(ticks);
      usleep(15000);
      vmc_api->useVMC().Motor[2].EncoderTicks.Set(ticks);
      usleep(15000);
      vmc_api->useVMC().Motor[0].VelocityControllerIntegralPart.Set(integral_part);
      usleep(15000);
      vmc_api->useVMC().Motor[1].VelocityControllerIntegralPart.Set(integral_part);
      usleep(15000);
      vmc_api->useVMC().Motor[2].VelocityControllerIntegralPart.Set(integral_part);
      usleep(15000);
      vmc_api->useVMC().Motor[0].VelocityControllerDifferentialPart.Set(differential_part);
      usleep(15000);
      vmc_api->useVMC().Motor[1].VelocityControllerDifferentialPart.Set(differential_part);
      usleep(15000);
      vmc_api->useVMC().Motor[2].VelocityControllerDifferentialPart.Set(differential_part);
      usleep(15000);
      vmc_api->useVMC().Motor[0].VelocityControllerLinearPart.Set(linear_part);
      usleep(15000);
      vmc_api->useVMC().Motor[1].VelocityControllerLinearPart.Set(linear_part);
      usleep(15000);
      vmc_api->useVMC().Motor[2].VelocityControllerLinearPart.Set(linear_part);
      usleep(15000);

      //to guarantee that the rotations are set
      logger->log_debug(name(), "Wait for reply from the VMC.");
      while( 0.0 == vmc_api->useVMC().Motor[0].AbsolutRotations.getValue() &&
	     0.0 == vmc_api->useVMC().Motor[1].AbsolutRotations.getValue() &&
	     0.0 == vmc_api->useVMC().Motor[2].AbsolutRotations.getValue() )
        {
          vmc_api->useVMC().MotorRPMs.Set(10, 10, 10);
          usleep(15000);
        }
      logger->log_debug(name(), "Got the reply from the VMC.");

      vmc_api->useVMC().MotorRPMs.Set(0, 0, 0);
      usleep(15000);

      // automatically update the actual RPMs after every command
      vmc_api->configRequestMessage(3, false);
      vmc_api->configRequestMessage(0, true);
    }

  // open blackboard interface
  try
    {
      motor_interface = blackboard->open_for_writing<MotorInterface>("Motor");
    }
  catch (Exception& e)
    {
      logger->log_error(name(), "Opening interface failed!");
      logger->log_error(name(), e);
      throw;
    }

  motor_interface->set_motor_state(MotorInterface::MOTOR_ENABLED);
  motor_interface->set_drive_mode(0);
  motor_interface->set_odometry_path_length(0.0);
  motor_interface->set_odometry_position_x(0.0);
  motor_interface->set_odometry_position_y(0.0);
  motor_interface->set_odometry_orientation(0.0);
  motor_interface->write();

  // instantiate motion model
  motion_model = new OmniMotionModel(radius, wheel_radius, gear_reduction, true);

  // instantiate velocity controller
  velocity_controller = new LinearVelocityController(clock, loop_time, 1.5);
}

void
MotorThread::finalize()
{
  logger->log_info(name(), "Finalizing thread %s", name());

  // close blackboard interface
  try
    {
      blackboard->close(motor_interface);
    }
  catch (Exception& e)
    {
      logger->log_error(name(), "Closing interface failed!");
      logger->log_error(name(), e);
    }

  // disconnect from VMC
  if( !no_vmc )
    {
      vmc_api->closeDevice();
      delete vmc_api;
    }
  
  free(vmc_port);
  
  delete motion_model;
  delete velocity_controller;

  logger->log_info(name(), "End of Finalizing thread %s", name());
}


/** Here the driving commands are transformed to the RPMs for the three motors
 * and the odometric pose is updated.
 */
void
MotorThread::loop()
{
  // desired RPMs
  float rpm_right_des = 0.0;
  float rpm_rear_des = 0.0;
  float rpm_left_des = 0.0;

  // actual RPMs
  float rpm_right_act = 0.0;
  float rpm_rear_act = 0.0;
  float rpm_left_act = 0.0;

  // shaft rotations
  float rot_right = 0.0;
  float rot_rear = 0.0;
  float rot_left = 0.0;

  // odometric pose
  float odo_x = 0.0;
  float odo_y = 0.0;
  float odo_phi = 0.0;

  float path_length = 0.0;
  float time_diff_sec = 0.0;

  Time now(clock);

  // here we go...
  motor_interface->read();

  // get message
  if ( !motor_interface->msgq_empty() )
    {
      // acquire control message
      if ( motor_interface->msgq_first_is<MotorInterface::AcquireControlMessage>() )
        {
          MotorInterface::AcquireControlMessage* msg = motor_interface->msgq_first<MotorInterface::AcquireControlMessage>();

          if ( msg->controller() == 0 )
            {
              motor_interface->set_controller(msg->sender_id());
              motor_interface->set_controller_thread_name(msg->sender_thread_name());
            }
          else
            {
              motor_interface->set_controller(msg->controller());
              motor_interface->set_controller_thread_name(msg->controller_thread_name());
            }
          motor_interface->write();

          logger->log_debug(name(), "Thread %s (%u) acquired motor control",
                            motor_interface->controller_thread_name(),
                            motor_interface->controller());
        }
      
      // RPM message
      else if ( motor_interface->msgq_first_is<MotorInterface::DriveRPMMessage>() )
        {
          MotorInterface::DriveRPMMessage* msg = motor_interface->msgq_first<MotorInterface::DriveRPMMessage>();

          if ( msg->sender_id() == motor_interface->controller() )
            {
              rpm_right_des = msg->front_right();
              rpm_rear_des  = msg->rear();
              rpm_left_des  = msg->front_left();

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_RPM);
              motor_interface->write();
            }
          else
            {
              logger->log_warn(name(), "Warning, received DriveRPMMessage of thread %s (%u), "
                               "but the motor is currently controlled by thread %s (%u)",
                               msg->sender_thread_name(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller());
            }
        }

      // trans-rot message
      else if ( motor_interface->msgq_first_is<MotorInterface::TransRotMessage>() )
        {
          MotorInterface::TransRotMessage* msg = motor_interface->msgq_first<MotorInterface::TransRotMessage>();

          if ( msg->sender_id() == motor_interface->controller() )
            {
              vx_des    = msg->vx();
              vy_des    = msg->vy();
              omega_des = msg->omega();

	      velocity_controller->set_target_velocity( (float) sqrt( vx_des * vx_des +
								      vy_des * vy_des ) );

	      // TODO: check necessity
//               if( !no_vmc )
//                 {
//                   forward *= correction_x;
//                   forward *= correction_y;
//                   forward *= correction_rotation;
//                 }

//               current_max_velocity = sqrt(pow(forward, 2.) + pow(sideward, 2.));
//               last_acceleration_time = clock->now();

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_TRANS_ROT);
              motor_interface->write();
            }
          else
            {
              logger->log_warn(name(), "Warning, received TransRotMessage of thread %s (%u), "
                               "but the motor is currently controlled by thread %s (%u)",
                               msg->sender_thread_name(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller());
            }
        }

      // trans message
      else if ( motor_interface->msgq_first_is<MotorInterface::TransMessage>() )
        {
          MotorInterface::TransMessage* msg = motor_interface->msgq_first<MotorInterface::TransMessage>();

          if ( msg->sender_id() == motor_interface->controller() )
            {
              vx_des    = msg->vx();
              vy_des    = msg->vy();
	      omega_des = 0.0;

	      velocity_controller->set_target_velocity( (float) sqrt( vx_des * vx_des +
								      vy_des * vy_des ) );

	      // TODO: see above
//               if( !no_vmc )
//                 {
//                   forward *= correction_x;
//                   forward *= correction_y;
//                 }

//               current_max_velocity = sqrt(pow(forward, 2.) + pow(sideward, 2.));
//               last_acceleration_time = clock->now();
//               rotation = 0;

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_TRANS);
              motor_interface->write();
            }
          else
            {
              logger->log_warn(name(), "Warning, received TransMessage of thread %s (%u), "
                               "but the motor is currently controlled by thread %s (%u)",
                               msg->sender_thread_name(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller());
            }
        }

      // rotation messsage
      else if ( motor_interface->msgq_first_is<MotorInterface::RotMessage>() )
        {
          MotorInterface::RotMessage* msg = motor_interface->msgq_first<MotorInterface::RotMessage>();

          if ( msg->sender_id() == motor_interface->controller() )
            {
	      vx_des    = 0.0;
	      vy_des    = 0.0;
	      omega_des = msg->omega();

	      // TODO: see above
//               if( !no_vmc )
//                 {
//                   forward *= correction_rotation;
//                 }

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_ROT);
              motor_interface->write();
            }
          else
            {
              logger->log_warn(name(), "Warning, received RotMessage of thread %s (%u), "
                               "but the motor is currently controlled by thread %s (%u)",
                               msg->sender_thread_name(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller());
            }
        }

      // orbit message
      else if ( motor_interface->msgq_first_is<MotorInterface::OrbitMessage>() )
        {
          MotorInterface::OrbitMessage* msg = motor_interface->msgq_first<MotorInterface::OrbitMessage>();

          if ( msg->sender_id() == motor_interface->controller() )
            {
	      // TODO
//               orbit_center.x(msg->px());
//               orbit_center.y(msg->py());
//               orbit_position.x(0.);
//               orbit_position.y(0.);
//               orbit_angular_velocity = msg->omega(); //neg clockwise, pos counterclockwise
//               orbit_radius = orbit_center.length(); //m
//               orbit_direction = orbit_center;
//               if(orbit_angular_velocity < 0)
//                 {
//                   orbit_sign = 1;
//                   orbit_angular_velocity *= -1;
//                 }
//               else
//                 {
//                   orbit_sign = -1;
//                 }
//               orbit_velocity = orbit_radius * orbit_angular_velocity; //m/s

//               logger->log_info("MotorThread", "-----> orbit_center.x() : %f", orbit_center.x());
//               logger->log_info("MotorThread", "-----> orbit_center.y() : %f", orbit_center.y());

//               rotation = 0;

//               motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_ORBIT);
//               motor_interface->write();
//               start_time = true;
//               //  last_time = clock->now();
//               //              if(!no_vmc)
//               //                {
//               //                  if(stopped)
//               //                    {
//               //                      last_alpha_rotations += vmc_api->useVMC().Motor[0].AbsolutRotations.getValue();
//               //                      last_beta_rotations += vmc_api->useVMC().Motor[1].AbsolutRotations.getValue();
//               //                      last_gamma_rotations += vmc_api->useVMC().Motor[2].AbsolutRotations.getValue();
//               //                      stopped = false;
//               //                    }
//               //                }
//               /*
//                 logger->log_debug(name(), "Processing OrbitMessage, PX: %f, "
//                 "PY: %f, Omega: %f",
//                 orbit_center.x(), orbit_center.y(), orbit_angular_velocity);
//               */
            }
	  else
            {
              logger->log_warn(name(), "Warning, received OrbitMessage of thread %s (%u), "
                               "but the motor is currently controlled by thread %s (%u)",
                               msg->sender_thread_name(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller());
            }
        }

      // linear trans-rot message
      else if ( motor_interface->msgq_first_is<MotorInterface::LinTransRotMessage>() )
        {
          MotorInterface::LinTransRotMessage* msg = motor_interface->msgq_first<MotorInterface::LinTransRotMessage>();

          if ( msg->sender_id() == motor_interface->controller() )
            {
	      vx_des    = msg->vx();
	      vy_des    = msg->vy();
	      omega_des = msg->omega();

	      velocity_controller->set_target_velocity( (float) sqrt( vx_des * vx_des +
								      vy_des * vy_des ) );

              motor_interface->set_drive_mode(MotorInterface::DRIVE_MODE_LINE_TRANS_ROT);
              motor_interface->write();
            }
	  else
            {
              logger->log_warn(name(), "Warning, received LinTransRotMessage of thread %s (%u), "
                               "but the motor is currently controlled by thread %s (%u)",
                               msg->sender_thread_name(), msg->sender_id(),
                               motor_interface->controller_thread_name(),
                               motor_interface->controller());
            }
        }

      // reset odometry
      else if (motor_interface->msgq_first_is<MotorInterface::ResetOdometryMessage>() )
        {
	  motion_model->reset_odometry();

          motor_interface->set_odometry_path_length(0.0);
          motor_interface->set_odometry_position_x(0.0);
          motor_interface->set_odometry_position_y(0.0);
          motor_interface->set_odometry_orientation(0.0);
          motor_interface->write();
        }

      // change motor state
      else if (motor_interface->msgq_first_is<MotorInterface::SetMotorStateMessage>() )
        {
          MotorInterface::SetMotorStateMessage* msg = motor_interface->msgq_first<MotorInterface::SetMotorStateMessage>();
          // we really want to make sure that we got a correct message with useful values
          // thus we check every single value
          if ( msg->motor_state() == MotorInterface::MOTOR_ENABLED )
            {
              motor_interface->set_motor_state(MotorInterface::MOTOR_ENABLED);
	      motor_interface->write();
              logger->log_info(name(), "Enabling motor control");
            }
          else if ( msg->motor_state() == MotorInterface::MOTOR_DISABLED )
            {
              motor_interface->set_motor_state(MotorInterface::MOTOR_DISABLED);
	      motor_interface->write();
              logger->log_info(name(), "Disabling motor control");
            }
          else
            {
              logger->log_error(name(), "SetMotorStateMessage received with illegal value: %u",
                                msg->motor_state());
            }
        }

      // received message with unknown type
      else
        {
          logger->log_error(name(), "Message of invalid type received from %s", motor_interface->msgq_first()->sender_thread_name());
        }

      motor_interface->msgq_pop();
    }

  // velocity controller
  velocity_controller->set_actual_velocity( (float) sqrt( vx_act * vx_act +
							  vy_act * vy_act ) );

  float vel;
  vel = *velocity_controller->get_next_velocity();
  HomVector v(vx_des, vy_des);
  v.set_length(vel);
  vx_des = v.x();
  vy_des = v.y();

  // non-stateless messages need some further computations to take the robot's movement
  // since the message was received into account.
  unsigned int drive_mode = motor_interface->drive_mode();
  if ( drive_mode == MotorInterface::DRIVE_MODE_ORBIT )
    {
      // TODO
      logger->log_debug(name(), "NOT YET IMPLEMENTED");

//           //calculate the tangent to the orbit passing through the actual position
//           if(orbit_direction.length() != 0)
//             {
//               Vector old_direction = orbit_direction;
//               orbit_direction = orbit_center - orbit_position;
//               double alpha = 0;
//               // logger->log_info("MotorThread", " orbit_direction.length() %f ", orbit_direction.length());
//               //  logger->log_info("MotorThread", " orbit_radius %f ", orbit_radius);
//               if(orbit_radius <= orbit_direction.length())
//                 {
//                   alpha = asin(orbit_radius / orbit_direction.length()) * orbit_sign;
//                   //   logger->log_info("MotorThread", " alpha %f ", alpha);

//                   orbit_direction.rotate_z(alpha);
//                 }
//               else
//                 {
//                   orbit_direction = old_direction;
//                 }

//               orbit_direction.unit();
//               forward = orbit_direction.x() * orbit_velocity * translation_rpm_factor;
//               sideward = orbit_direction.y() * orbit_velocity * translation_rpm_factor;
//             }
    }
  else if ( drive_mode == MotorInterface::DRIVE_MODE_LINE_TRANS_ROT)
    {
      HomVector v(vx_des, vy_des);
      v.rotate_z( -odo_delta_phi );
      vx_des = v.x();
      vy_des = v.y();
    }

  //   logger->log_info(name(), "Desired velocities: vx=%f  vy=%f  omega=%f",
  // 		   vx_des, vy_des, omega_des);

  // if the RPMs weren't set directly they have to be computed from the velocites
  if ( motor_interface->drive_mode() != MotorInterface::DRIVE_MODE_RPM )
    {
      motion_model->velocities_to_rpm( vx_des, vy_des, omega_des, 
				       rpm_right_des, rpm_rear_des, rpm_left_des );
    }

  //   logger->log_info(name(), "Desired RPMs: right: %f  rear=%f  left=%f", 
  // 		   rpm_right_des, rpm_rear_des, rpm_left_des);

  // send RPMs to the motor controller and obtain the actual  rotations form
  // the  motor controller
  if ( !no_vmc )
    {
      if ( motor_interface->motor_state() == MotorInterface::MOTOR_ENABLED )
	{
	  vmc_api->useVMC().MotorRPMs.Set(rpm_right_des, rpm_rear_des, rpm_left_des);
	}

      // wait for the ultra-fast VMC
      usleep(loop_time);
      
      now.stamp();

      if ( !first_loop )
	{ 
	  time_diff_sec = (now - last_loop).in_sec(); 
	}
      else
	{
	  time_diff_sec = loop_time / 1000000.0;
	  first_loop = false;
	}

      last_loop = now;

      rpm_right_act = vmc_api->useVMC().Motor[0].ActualRPM.getValue();
      rpm_rear_act  = vmc_api->useVMC().Motor[1].ActualRPM.getValue();
      rpm_left_act  = vmc_api->useVMC().Motor[2].ActualRPM.getValue();

      rot_right = rpm_right_act / 60.0 * time_diff_sec;
      rot_rear  = rpm_rear_act  / 60.0 * time_diff_sec;
      rot_left  = rpm_left_act  / 60.0 * time_diff_sec;
    }
  else
    {
      now.stamp();

      if ( !first_loop )
	{ 
	  time_diff_sec = (now - last_loop).in_sec(); 
	}
      else
	{
	  time_diff_sec = loop_time / 1000000.0;
	  first_loop = false;
	}

      last_loop = now;

      rpm_right_act = rpm_right_des;
      rpm_rear_act  = rpm_rear_des;
      rpm_left_act  = rpm_left_des;

      rot_right = rpm_right_des / 60.0 * time_diff_sec;
      rot_rear  = rpm_rear_des  / 60.0 * time_diff_sec;
      rot_left  = rpm_left_des  / 60.0 * time_diff_sec;

      usleep(loop_time);
    }

  //   logger->log_info(name(), "Shaft rotations: right: %f rear: %f  left: %f",
  // 		   rot_right, rot_rear, rot_left);
  
  //   logger->log_info(name(), "Actual RPMs: right: %f  rear: %f  left: %f", 
  // 		   rpm_right_act, rpm_rear_act, rpm_left_act);
  
  //   logger->log_info(name(), "time diff %f (sec)", time_diff_sec);
  
  // update pose estimation, actual velocities, and path length
  motion_model->update_odometry(rot_right, rot_rear, rot_left, time_diff_sec);
  motion_model->get_odom_pose(odo_x, odo_y, odo_phi);
  motion_model->get_odom_diff(odo_delta_x, odo_delta_y, odo_delta_phi);
  motion_model->get_actual_velocities(vx_act, vy_act, omega_act);

  path_length = motor_interface->odometry_path_length();
  path_length += sqrt( odo_delta_x * odo_delta_x + odo_delta_y * odo_delta_y );

  //   logger->log_info(name(), "Actual velocities: vx=%f vy=%f omega=%f",
  // 		   vx_act, vy_act, omega_act);
  //   logger->log_info(name(), "Odom pose: x=%f  y=%f  phi=%f",
  // 		   odo_x, odo_y, odo_phi);
  //   logger->log_info(name(), "Odom delta: x=%f  y=%f  phi=%f",
  // 		   odo_delta_x, odo_delta_y, odo_delta_phi);

  // update interface
  motor_interface->set_odometry_position_x(odo_x);
  motor_interface->set_odometry_position_y(odo_y);
  motor_interface->set_odometry_orientation(odo_phi);

  motor_interface->set_vx(vx_act);
  motor_interface->set_vy(vy_act);
  motor_interface->set_omega(omega_act);

  motor_interface->set_right_rpm( (int) rint(rpm_right_act) );
  motor_interface->set_rear_rpm( (int) rint(rpm_rear_act) );
  motor_interface->set_left_rpm( (int) rint(rpm_left_act) );

  motor_interface->set_odometry_path_length(path_length);

  motor_interface->write();
}
