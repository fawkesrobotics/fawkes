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
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$  */
/*                                                                      */
/* Description: This is an abstract drive module interface of Colli-A*  */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is an abstract drive module.                              */
/*       Base class for all other drive modes.                          */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_ABSTRACT_DRIVE_MODE_H_
#define _COLLI_ABSTRACT_DRIVE_MODE_H_

#include <interfaces/NavigatorInterface.h>
#include <logging/logger.h>
#include <config/config.h>

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** This is the base class which calculates drive modes. Drive modes
 *    are the proposed settings for the drive-realization out of the
 *    found search things.
 */
class CAbstractDriveMode
{
public:

  /** Constructor. You have to set the drive mode name out of colli_targetobj.h
   *    to your derived drive mode.
   */
  CAbstractDriveMode(fawkes::Logger* logger, fawkes::Configuration* config);


  /** Destructor.
   */
  virtual ~CAbstractDriveMode();


  /** Sets the current target.
   *  Has to be set before Update!
   */
  void SetCurrentTarget( float targetX, float targetY, float targetOri );


  /** Sets the current robo position.
   *  Has to be set before Update!
   */
  void SetCurrentRoboPos( float roboX, float roboY, float roboOri );


  /** Set the local targetpoint found by the search.
   *  Has to be set before Update!
   */
  void SetLocalTarget( float localTargetX, float localTargetY );


  /** Set the local trajectory point found by the search.
   *  Has to be set before Update!
   */
  void SetLocalTrajec( float localTrajecX, float localTrajecY );


  /** Sets the current robo speed.
   *  Has to be set before Update!
   */
  void SetCurrentRoboSpeed( float roboTrans, float roboRot );


  /** Set the colli mode values for each drive mode.
   *  Has to be set before Update!
   */
  void SetCurrentColliMode( bool orient, bool stop );


  /** Has to be implemented and has to calculate
   *   the proposed settings which are asked for
   *   afterwards.
   */
  virtual void Update() = 0;


  /** Returns the drive modes name.
   *  Has to be set in the constructor of your drive mode!
   */
  NavigatorInterface::DriveMode GetDriveModeName();


  /** Returns the proposed translation which
   *  was calculated previously in
   *  'Update' which has to be implemented!
   */
  float GetProposedTranslation();


  /** Returns the proposed rotation which
   *  was calculated previously in
   *  'Update' which has to be implemented!
   */
  float GetProposedRotation();



protected:


  float LinInterpol( float x, float left, float right, float bot, float top );

  float GuaranteeTransStop( float distance, float current_trans, float desired_trans );


  float m_TargetX, m_TargetY, m_TargetOri;  // current target
  float m_RoboX, m_RoboY, m_RoboOri;        // current robo pos
  float m_RoboTrans, m_RoboRot;             // current robo speed

  float m_LocalTargetX, m_LocalTargetY;     // local target
  float m_LocalTrajecX, m_LocalTrajecY;     // local trajectory

  bool m_OrientAtTarget;                    // flag if orienting necessary
  bool m_StopAtTarget;                      // flag if stopping on or after target

  float m_ProposedTranslation;       // proposed translation setting for next timestep
  float m_ProposedRotation;          // proposed rotation setting for next timestep

  fawkes::NavigatorInterface::DriveMode m_DriveModeName;        // the drive modes name. your have to set this

  fawkes::Logger* logger_;
  fawkes::Configuration* config_;

private:

  float m_cMaxTransDec;
  float m_cMaxRotDec;
};



/* ************************************************************************** */
/* ***********************  IMPLEMENTATION DETAILS  ************************* */
/* ************************************************************************** */


inline
CAbstractDriveMode::CAbstractDriveMode(fawkes::Logger* logger, fawkes::Configuration* config)
 : logger_( logger ),
   config_( config )
{
  logger_->log_info("CAbstractDriveMode", "(Constructor): Entering...");
  m_ProposedTranslation = 0.0;
  m_ProposedRotation = 0.0;
  m_DriveModeName = NavigatorInterface::MovingNotAllowed;

  // read m_cMaxTransDec and m_cMaxRotDec
  m_cMaxTransDec = 0.75*config_->get_float("/plugins/colli/QuadraticMotorInstruct/BASIC_TRANS_DEC");
  m_cMaxRotDec   = 0.75*config_->get_float("/plugins/colli/QuadraticMotorInstruct/BASIC_ROT_DEC");

  logger_->log_info("CAbstractDriveMode", "(Constructor): Exiting...");
}


inline CAbstractDriveMode::~CAbstractDriveMode()
{
  logger_->log_info("CAbstractDriveMode", "(Destructor): Entering...");
  logger_->log_info("CAbstractDriveMode", "(Destructor): Exiting...");
}


inline float CAbstractDriveMode::GetProposedTranslation()
{
  return m_ProposedTranslation;
}


inline float CAbstractDriveMode::GetProposedRotation()
{
  return m_ProposedRotation;
}


inline void CAbstractDriveMode::SetCurrentTarget( float targetX, float targetY, float targetOri )
{
  m_TargetX   = targetX;
  m_TargetY   = targetY;
  m_TargetOri = targetOri;
}


inline void CAbstractDriveMode::SetCurrentRoboPos( float roboX, float roboY, float roboOri )
{
  m_RoboX   = roboX;
  m_RoboY   = roboY;
  m_RoboOri = roboOri;
}


inline void CAbstractDriveMode::SetCurrentRoboSpeed( float roboTrans, float roboRot )
{
  m_RoboTrans = roboTrans;
  m_RoboRot   = roboRot;
}


inline void CAbstractDriveMode::SetCurrentColliMode( bool orient, bool stop )
{
  m_OrientAtTarget = orient;
  m_StopAtTarget   = stop;
}


inline void CAbstractDriveMode::SetLocalTarget( float localTargetX, float localTargetY )
{
  m_LocalTargetX = localTargetX;
  m_LocalTargetY = localTargetY;
}


inline void CAbstractDriveMode::SetLocalTrajec( float localTrajecX, float localTrajecY )
{
  m_LocalTrajecX = localTrajecX;
  m_LocalTrajecY = localTrajecY;
}


inline fawkes::NavigatorInterface::DriveMode CAbstractDriveMode::GetDriveModeName()
{
  return m_DriveModeName;
}


inline float CAbstractDriveMode::LinInterpol( float x, float x1, float x2,
                float y1, float y2 )
{
  return ( ((x-x1)*(y2-y1))/(x2-x1) + y1 );
}


inline float CAbstractDriveMode::GuaranteeTransStop( float distance,
                 float current_trans,
                 float desired_trans )
{
  distance = fabs( distance );
  current_trans = fabs( current_trans );


  if ( distance < 0.05 )
    return 0.0;

  if ( current_trans < 0.05 )
    return desired_trans;

  int time_needed_to_distance = (int)( distance / (current_trans/10.0) );
  int time_needed_to_stop = 0;

  float tmp_trans = 0.0;

  while( tmp_trans < desired_trans ) {
    ++time_needed_to_stop;
    tmp_trans += m_cMaxTransDec;
  }

  if( time_needed_to_stop >= time_needed_to_distance ) {
    float value = std::max( 0.0, current_trans - (1.0 * m_cMaxTransDec) );
    return value;
  } else {
    float value = std::min( current_trans + m_cMaxTransDec, desired_trans );
    return value;
  }
}

} // end namespace fawkes

#endif
