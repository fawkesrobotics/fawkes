
/***************************************************************************
 *  base_motor_instruct.h - Abstract base class for a motor instructor
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
 *             2014  Tobias Neumann
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

#ifndef __PLUGINS_COLLI_DRIVE_REALIZATION_BASE_MOTORINSTRUCT_H_
#define __PLUGINS_COLLI_DRIVE_REALIZATION_BASE_MOTORINSTRUCT_H_

#include "../common/types.h"

#include <interfaces/MotorInterface.h>
#include <logging/logger.h>
#include <config/config.h>

#include <utils/time/time.h>

#include <cmath>
#include <string>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BaseMotorInstruct <plugins/colli/drive_realization/base_motor_instruct.h>
 * The Basic of a Motorinstructor.
 */

class BaseMotorInstruct
{
 public:
  BaseMotorInstruct( MotorInterface* motor,
                     float frequency,
                     Logger* logger,
                     Configuration* config );
  virtual ~BaseMotorInstruct();

  ///\brief Try to realize the proposed values with respect to the maximum allowed values.
  void drive( float trans_x, float trans_y, float rot );

  ///\brief Executes a soft stop with respect to calculate_translation and calculate_rotation.
  void stop();

 protected:
  Logger*         logger_; /**< The fawkes logger */
  Configuration*  config_; /**< The fawkse config */

  float trans_acc_; /**< Translation acceleration */
  float trans_dec_; /**< Translation deceleration */
  float rot_acc_;   /**< Rotation acceleration */
  float rot_dec_;   /**< Rotation deceleration */

 private:
  /** calculates rotation speed
   * has to be implemented in its base classes!
   * is for the smoothness of rotation transitions.
   * calculate maximum allowed rotation change between proposed and desired values
   *
   */
  virtual float calculate_rotation( float current, float desired, float time_factor ) = 0;

  /** calculates translation speed.
   * has to be implemented in its base classes!
   * is for the smoothness of translation transitions.
   * calculate maximum allowed translation change between proposed and desired values
   */
  virtual float calculate_translation( float current, float desired, float time_factor) = 0;

  MotorInterface* motor_;

  colli_trans_rot_t current_;
  colli_trans_rot_t desired_;
  colli_trans_rot_t exec_;

  ///\brief setCommand sets the executable commands and sends them
  void set_command( );
};


/* ************************************************************************** */
/* ********************  BASE IMPLEMENTATION DETAILS  *********************** */
/* ************************************************************************** */

/** Constructor. Initializes all constants and the local pointers.
 * @param motor The MotorInterface with all the motor information
 * @param frequency The frequency of the colli (should become deprecated!)
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
inline
BaseMotorInstruct::BaseMotorInstruct( MotorInterface* motor,
                                      float frequency,
                                      Logger* logger,
                                      Configuration* config )
 : logger_( logger ),
   config_( config ),
   motor_( motor )
{
  logger_->log_debug("BaseMotorInstruct", "(Constructor): Entering");

  // init all members, zero, just to be on the save side
  current_.x = current_.y = current_.rot = 0.f;
  desired_.x = desired_.y = desired_.rot = 0.f;
  exec_.x = exec_.y = exec_.rot = 0.f;

  std::string cfg_prefix = "/plugins/colli/motor_instruct/";
  trans_acc_ = config_->get_float((cfg_prefix + "trans_acc").c_str());
  trans_dec_ = config_->get_float((cfg_prefix + "trans_dec").c_str());
  rot_acc_   = config_->get_float((cfg_prefix + "rot_acc").c_str());
  rot_dec_   = config_->get_float((cfg_prefix + "rot_dec").c_str());

  logger_->log_debug("BaseMotorInstruct", "(Constructor): Exiting");
}

/** Desctructor. */
inline
BaseMotorInstruct::~BaseMotorInstruct()
{
  logger_->log_debug("BaseMotorInstruct", "(Destructor): Entering");
  logger_->log_debug("BaseMotorInstruct", "(Destructor): Exiting");
}

/** Sends the drive command to the motor. */
inline void
BaseMotorInstruct::set_command()
{
  if( !motor_->has_writer() ) {
    logger_->log_warn("BaseMotorInstruct", "Cannot set command, no writer for MotorInterface '%s'", motor_->id());
    return;
  }

  colli_trans_rot_t cmd = {0.f, 0.f, 0.f};

  // Translation borders
  float exec_trans = std::fabs(std::sqrt( exec_.x*exec_.x + exec_.y*exec_.y ));
  if ( exec_trans >= 0.05 ) {
    //send command where the total translation is between [-3, 3]
    //Calculate factor of reduction to reach 3m/s
    float reduction = 3. / exec_trans;

    //Calculate positive maximum for vx and vy
    float vx_max  = fabs( exec_.x * reduction );
    float vy_max  = fabs( exec_.y * reduction );

    //Calculate new desired velocities
    cmd.x = std::fmin(std::fmax(exec_.x, -vx_max), vx_max);
    cmd.y = std::fmin(std::fmax(exec_.y, -vy_max), vy_max);
  }

  // Rotation borders
  if ( fabs(exec_.rot) >= 0.01 ) {
    //send command within [-2*pi, 2*pi]
    cmd.rot = std::fmin(std::fmax(exec_.rot, -2*M_PI), 2*M_PI) ;
  }

  // Send the commands to the motor. No controlling afterwards done!!!!
  motor_->msgq_enqueue(new MotorInterface::TransRotMessage(cmd.x, cmd.y, cmd.rot));
}


/** Try to realize the proposed values with respect to the physical constraints of the robot.
 * @param trans_x the proposed x translation velocity
 * @param trans_y the proposed y translation velocity
 * @param rot the proposed rotation velocity
 */
inline void
BaseMotorInstruct::drive( float trans_x,  float trans_y, float rot )
{
  // initializing driving values (to be on the sure side of life)
  exec_.x = exec_.y = exec_.rot = 0.f;

  /*
  // timediff storie to realize how often one was called
  Time currentTime;
  currentTime.stamp();
  long timediff = (currentTime - m_OldTimestamp).in_msec();
  float time_factor = (float)timediff / (1000.f / frequency_);

  if (time_factor < 0.5) {
    logger_->log_debug("BaseMotorInstruct","( Drive ): Blackboard timing(case 1) strange, time_factor is %f", time_factor);
  } else if (time_factor > 2.0) {
    logger_->log_debug("BaseMotorInstruct", "( Drive ): Blackboard timing(case 2) strange, time_factor is %f", time_factor);
  }

  m_OldTimestamp = currentTime;
  */
  float time_factor = 1.f;

  // getting currently performed values
  current_.x = motor_->des_vx();
  current_.y = motor_->des_vy();
  current_.rot = motor_->des_omega();

  // calculate maximum allowed rotation change between proposed and desired values
  desired_.rot = rot;
  exec_.rot    = calculate_rotation( current_.rot, desired_.rot, time_factor );

  // calculate maximum allowed translation change between proposed and desired values
  desired_.x = trans_x;
  desired_.y = trans_y;
  exec_.x    = calculate_translation( current_.x, desired_.x, time_factor );
  exec_.y    = calculate_translation( current_.y, desired_.y, time_factor );

  // send the command to the motor
  set_command();
}


/** Executes a soft stop with respect to calculate_translation and calculate_rotation
 * if it is called several times
 */
inline void
BaseMotorInstruct::stop()
{
  // with respect to the physical borders do a stop to 0.0, 0.0.
  drive( 0.f, 0.f, 0.f );
}

} // namespace fawkes
#endif
