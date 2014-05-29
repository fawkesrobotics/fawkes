
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

#include "../utils/rob/robo_motorcontrol.h"

#include <logging/logger.h>
#include <utils/time/time.h>

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MotorInterface;

/** @class CBaseMotorInstruct <plugins/colli/drive_realization/base_motor_instruct.h>
 * The Basic of a Motorinstructor.
 */

class CBaseMotorInstruct: public MotorControl
{
 public:

  CBaseMotorInstruct( fawkes::MotorInterface* motor,
                      float frequency,
                      fawkes::Logger* logger);

  virtual ~CBaseMotorInstruct();


  ///\brief Try to realize the proposed values with respect to the maximum allowed values.
  void Drive( float proposedTransX, float proposedTransY, float proposedRot );

  ///\brief Executes a soft stop with respect to CalculateTranslation and CalculateRotation.
  void ExecuteStop( );


 protected:
  fawkes::Logger* logger_; /**< The fawkes logger */


 private:

  float m_execTranslationX, m_execTranslationY, m_execRotation;
  float m_desiredTranslationX, m_desiredTranslationY, m_desiredRotation;
  float m_currentTranslationX, m_currentTranslationY, m_currentRotation;
  float m_Frequency;

  //fawkes::Time m_OldTimestamp;


  ///\brief setCommand sets the executable commands and sends them
  void SetCommand( );


  /** calculates rotation speed
   * has to be implemented in its base classes!
   * is for the smoothness of rotation transitions.
   * calculate maximum allowed rotation change between proposed and desired values
   */
  virtual float CalculateRotation( float currentRotation, float desiredRotation,
           float time_factor ) = 0;


  /** calculates translation speed.
   * has to be implemented in its base classes!
   * is for the smoothness of translation transitions.
   * calculate maximum allowed translation change between proposed and desired values
   */
  virtual float CalculateTranslation( float currentTranslation, float desiredTranslation,
      float time_factor) = 0;
};



/* ************************************************************************** */
/* ********************  BASE IMPLEMENTATION DETAILS  *********************** */
/* ************************************************************************** */

/** Constructor. Initializes all constants and the local pointers.
 * @param motor The MotorInterface with all the motor information
 * @param frequency The frequency of the colli (should become deprecated!)
 * @param logger The fawkes logger
 */
inline
CBaseMotorInstruct::CBaseMotorInstruct( fawkes::MotorInterface* motor,
                                        float frequency,
                                        fawkes::Logger* logger )
 : MotorControl( motor ),
   logger_(logger)
{
  logger_->log_debug("CBaseMotorInstruct", "(Constructor): Entering");
  // init all members, zero, just to be on the save side
  m_desiredTranslationX = m_desiredTranslationY = m_desiredRotation = 0.0;
  m_currentTranslationX = m_currentTranslationY = m_currentRotation = 0.0;
  m_execTranslationX    = m_execTranslationY    = m_execRotation    = 0.0;
  //m_OldTimestamp.stamp();
  m_Frequency = frequency;
  logger_->log_debug("CBaseMotorInstruct", "(Constructor): Exiting");
}


/** Desctructor. */
inline
CBaseMotorInstruct::~CBaseMotorInstruct()
{
  logger_->log_debug("CBaseMotorInstruct", "(Destructor): Entering");
  logger_->log_debug("CBaseMotorInstruct", "(Destructor): Exiting");
}


/** Sends the drive command to the motor. */
inline void
CBaseMotorInstruct::SetCommand()
{
  // Translation borders
  float execTranslation = std::fabs(std::sqrt( m_execTranslationX*m_execTranslationX + m_execTranslationY*m_execTranslationY ));
  if ( execTranslation < 0.05 ) {
    SetDesiredTranslationX( 0.0 );
    SetDesiredTranslationY( 0.0 );
  } else {
    //send command where the total translation is between [-3, 3]
    float reduction = 3. / execTranslation;                                     //Calculate factor of reduction to reach 3m/s

    float vx_max  = fabs( m_execTranslationX * reduction );                     //Calculate positive maximum for vx and vy
    float vy_max  = fabs( m_execTranslationY * reduction );
    float vx      = std::fmin(std::fmax(m_execTranslationX, -vx_max), vx_max);  //Calculate new vx
    float vy      = std::fmin(std::fmax(m_execTranslationY, -vy_max), vy_max);
    SetDesiredTranslationX( vx );
    SetDesiredTranslationY( vy );
  }

  // Rotation borders
  if ( fabs(m_execRotation) < 0.01 ) {
    SetDesiredRotation( 0.0 );
  } else {
    SetDesiredRotation( std::fmin(std::fmax(m_execRotation, -2*M_PI), 2*M_PI) );  //send command within [-2*pi, 2*pi]
  }

  // Send the commands to the motor. No controlling afterwards done!!!!
  SendCommand();
}


/** Try to realize the proposed values with respect to the physical constraints of the robot.
 * @param proposedTransX the proposed x translation velocity
 * @param proposedTransY the proposed y translation velocity
 * @param proposedRot the proposed rotation velocity
 */
inline void
CBaseMotorInstruct::Drive( float proposedTransX,  float proposedTransY, float proposedRot )
{
  // initializing driving values (to be on the sure side of life)
  m_execTranslationX = 0.0;
  m_execTranslationY = 0.0;
  m_execRotation = 0.0;

  /*
  // timediff storie to realize how often one was called
  Time currentTime;
  currentTime.stamp();
  long timediff = (currentTime - m_OldTimestamp).in_msec();
  float time_factor = (float)timediff / (1000.f / m_Frequency);

  if (time_factor < 0.5) {
    logger_->log_debug("CBaseMotorInstruct","( Drive ): Blackboard timing(case 1) strange, time_factor is %f", time_factor);
  } else if (time_factor > 2.0) {
    logger_->log_debug("CBaseMotorInstruct", "( Drive ): Blackboard timing(case 2) strange, time_factor is %f", time_factor);
  }

  m_OldTimestamp = currentTime;
  */
  float time_factor = 1.f;

  // getting current performed values
  m_currentRotation    = GetMotorDesiredRotation();
  m_currentTranslationX = GetMotorDesiredTranslationX();
  m_currentTranslationY = GetMotorDesiredTranslationY();

  // calculate maximum allowed rotation change between proposed and desired values
  m_desiredRotation = proposedRot;
  m_execRotation    = CalculateRotation( m_currentRotation, m_desiredRotation, time_factor );

  // calculate maximum allowed translation change between proposed and desired values

  m_desiredTranslationX = proposedTransX;
  m_desiredTranslationY = proposedTransY;
  m_execTranslationX    = CalculateTranslation( m_currentTranslationX, m_desiredTranslationX, time_factor );
  m_execTranslationY    = CalculateTranslation( m_currentTranslationY, m_desiredTranslationY, time_factor );

  // send the command to the motor
  SetCommand( );
}


/** Executes a soft stop with respect to CalculateTranslation and CalculateRotation
 * if it is called several times
 */
inline void
CBaseMotorInstruct::ExecuteStop()
{
  m_currentTranslationX = GetMotorDesiredTranslationX();
  m_currentTranslationY = GetMotorDesiredTranslationY();
  m_currentRotation     = GetMotorDesiredRotation();

  // with respect to the physical borders do a stop to 0.0, 0.0.
  Drive( 0.0, 0.0, 0.0 );
  SetCommand( );
}



} // namespace fawkes

#endif

