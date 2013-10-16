//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
/*
 �                                                                            �
 �                                            ####   ####           .-""-.    �
 �       # #                             #   #    # #    #         /[] _ _\   �
 �       # #                                 #    # #             _|_o_LII|_  �
 � ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ �
 � #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| �
 � #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  �
 � #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  �
 � '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  �
 �                                                               /__|    |__\ �
 �                                                                            �
 */
/* ******************************************************************** */
/*                                                                      */
/* $Id$  */
/*                                                                      */
/* Description: This is the basic abstract motor instructor for Colli-A* */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This module is the base class for validity checks of drive     */
/*         commands and sets those things with respect to the physical  */
/*         borders of the robot.                                        */
/*       For this purpose the two functions CalculateRotation and       */
/*         CalculateTranslation have to be implemented in the derived   */
/*         class.                                                       */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */

#ifndef _COLLI_DRIVE_REALIZATION_BASE_MOTORINSTRUCT_H_
#define _COLLI_DRIVE_REALIZATION_BASE_MOTORINSTRUCT_H_

#include "../utils/rob/robo_motorcontrol.h"
//#include "../utils/rob/robo_laser.h" // TODO: why was this included?

#include <logging/logger.h>
#include <utils/time/time.h>

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MotorInterface;

/** The Basic of a Motorinstructor.
 */
class CBaseMotorInstruct: public MotorControl
{
 public:

  ///
  CBaseMotorInstruct( fawkes::MotorInterface* motor,
                      fawkes::MotorInterface* motor_des,
                      float frequency,
                      fawkes::Logger* logger);

  ///
  virtual ~CBaseMotorInstruct();


  /** Try to realize the proposed values
   *    with respect to the maximum allowed values.
   */
  void Drive( float proposedTrans, float proposedRot );


  /** Executes a soft stop with respect to CalculateTranslation
   *    and CalculateRotation.
   */
  void ExecuteStop( );

protected:
  fawkes::Logger* logger_;

private:

  //////////////////////////////////////////////////////////////////////
  // VARIABLES
  //////////////////////////////////////////////////////////////////////

  float m_execTranslation, m_execRotation;
  float m_desiredTranslation, m_desiredRotation;
  float m_currentTranslation, m_currentRotation;
  float m_Frequency;

  fawkes::Time m_OldTimestamp;

  //////////////////////////////////////////////////////////////////////
  // METHODS
  //////////////////////////////////////////////////////////////////////

  // setCommand sets the executable commands and sends them
  void SetCommand( );


  // calculates rotation speed
  // has to be implemented in its base classes!
  // is for the smoothness of rotation transitions.
  // calculate maximum allowed rotation change between proposed and desired values
  virtual float CalculateRotation( float currentRotation, float desiredRotation,
           float time_factor ) = 0;


  // calculates translation speed.
  // has to be implemented in its base classes!
  // is for the smoothness of translation transitions.
  // calculate maximum allowed translation change between proposed and desired values
  virtual float CalculateTranslation( float currentTranslation, float desiredTranslation,
              float time_factor ) = 0;
};



/* ************************************************************************** */
/* ********************  BASE IMPLEMENTATION DETAILS  *********************** */
/* ************************************************************************** */

// Constructor. Initializes all constants and the local pointers.
inline
CBaseMotorInstruct::CBaseMotorInstruct( fawkes::MotorInterface* motor,
                                        fawkes::MotorInterface* motor_des,
                                        float frequency,
                                        fawkes::Logger* logger )
 : MotorControl( motor, motor_des ),
   logger_(logger)
{
  logger_->log_info("CBaseMotorInstruct", "(Constructor): Entering");
  // init all members, zero, just to be on the save side
  m_desiredTranslation = m_desiredRotation = 0.0;
  m_currentTranslation = m_currentRotation = 0.0;
  m_execTranslation    = m_execRotation    = 0.0;
  m_OldTimestamp.stamp();
  m_Frequency = frequency;
  logger_->log_info("CBaseMotorInstruct", "(Constructor): Exiting");
}



// Destructor
inline
CBaseMotorInstruct::~CBaseMotorInstruct()
{
  logger_->log_info("CBaseMotorInstruct", "(Destructor): Entering");
  logger_->log_info("CBaseMotorInstruct", "(Destructor): Exiting");
}



// setcommand. Puts the command to the motor.
inline void
CBaseMotorInstruct::SetCommand()
{
  // This case here should be removed after Alex's RPC does work.
  // SJ TODO!!!
  if ( !(GetMovingAllowed()) )
    SetRecoverEmergencyStop();
  // SJ TODO!!!


  // Translation borders
  if ( fabs(m_execTranslation) < 0.05 )
    SetDesiredTranslation( 0.0 );
  else
    if ( fabs(m_execTranslation) > 3.0 )
      if ( m_execTranslation > 0.0 )
  SetDesiredTranslation( 3.0 );
      else
  SetDesiredTranslation( -3.0 );
    else
      SetDesiredTranslation( m_execTranslation );

  // Rotation borders
  if ( fabs(m_execRotation) < 0.01 )
    SetDesiredRotation( 0.0 );
  else
    if ( fabs(m_execRotation) > 2*M_PI )
      if ( m_execRotation > 0.0 )
  SetDesiredRotation( 2*M_PI );
      else
  SetDesiredRotation( -2*M_PI );
    else
      SetDesiredRotation( m_execRotation );

  // Send the commands to the motor. No controlling afterwards done!!!!
  SendCommand();
}



// Set a drive command with respect to the physical constraints of the robot.
inline void
CBaseMotorInstruct::Drive( float proposedTrans, float proposedRot )
{
  // initializing driving values (to be on the sure side of life)
  m_execTranslation = 0.0;
  m_execRotation = 0.0;

  // timediff storie to realize how often one was called
  Time currentTime;
  currentTime.stamp();
  float timediff = (currentTime - m_OldTimestamp).in_sec();
  float time_factor = ( (timediff*1000.0) / m_Frequency);

  if (time_factor < 0.5) {
    logger_->log_debug("CBaseMotorInstruct","( Drive ): Blackboard timing(case 1) strange, time_factor is %f", time_factor);
  } else if (time_factor > 2.0) {
    logger_->log_debug("CBaseMotorInstruct", "( Drive ): Blackboard timing(case 2) strange, time_factor is %f", time_factor);
  }

  m_OldTimestamp = currentTime;

  // getting current performed values
  m_currentRotation    = GetMotorDesiredRotation();
  m_currentTranslation = GetMotorDesiredTranslation();

  // calculate maximum allowed rotation change between proposed and desired values
  m_desiredRotation = proposedRot;
  m_execRotation    = CalculateRotation( m_currentRotation, m_desiredRotation, time_factor );

  // calculate maximum allowed translation change between proposed and desired values
  m_desiredTranslation = proposedTrans;
  m_execTranslation    = CalculateTranslation( m_currentTranslation, m_desiredTranslation, time_factor );

  // send the command to the motor
  SetCommand( );
}



// Does execute a stop command if it is called several times
inline void
CBaseMotorInstruct::ExecuteStop()
{
  m_currentTranslation = GetMotorDesiredTranslation();
  m_currentRotation    = GetMotorDesiredRotation();

  // with respect to the physical borders do a stop to 0.0, 0.0.
  Drive( 0.0, 0.0 );
  SetCommand( );
}



} // namespace fawkes

#endif

