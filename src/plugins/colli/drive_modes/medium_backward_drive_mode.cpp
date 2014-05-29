
/***************************************************************************
 *  medium_backward_drive_mode.cpp - Implementation of drive-mode "medium backward"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

#include "medium_backward_drive_mode.h"
#include <utils/math/angle.h>
#include <utils/math/common.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CMediumBackwardDriveModule <plugins/colli/drive_modes/medium_backward_drive_mode.h>
 * This is the MediumBackward drive-module, for medium backward only movements.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
CMediumBackwardDriveModule::CMediumBackwardDriveModule(Logger* logger, Configuration* config)
 : CAbstractDriveMode(logger, config)
{
  logger_->log_debug("CMediumBackwardDriveModule", "(Constructor): Entering...");
  m_DriveModeName = NavigatorInterface::ModerateBackward;

  m_MaxTranslation = config_->get_float( "/plugins/colli/drive_mode/medium/max_trans" );
  m_MaxRotation    = config_->get_float( "/plugins/colli/drive_mode/medium/max_rot" );

  logger_->log_debug("CMediumBackwardDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here.
 */
CMediumBackwardDriveModule::~CMediumBackwardDriveModule()
{
  logger_->log_debug("CMediumBackwardDriveModule", "(Destructor): Entering...");
  m_DriveModeName = NavigatorInterface::MovingNotAllowed;
  logger_->log_debug("CMediumBackwardDriveModule", "(Destructor): Exiting...");
}





/** Calculate by given variables a new rotation to give for the motor to minimize curvature.
 *
 *  DOC.: This here is the most complicated part in realizing the colli. By the given information
 *        I have to calculate a rotation I want to achieve in an optimal way.
 *        Here this is solved in an interesting way:
 *        First, check how long the curvature is, we want to drive to the target. This is done by
 *        approximating the size of the triangle around this curvature given by collision and
 *        and targetpoint and the angle between those both. Afterwards the time we have to drive
 *        with constant speed is calculated. Now we have the time we want to turn the angle. By
 *        multiplying this with a high constant (here 4), we rotate faster than we want to, but
 *        so the curvatures are not so extraordinary big. Afterwards there is an proportional
 *        part added, so we have a control factor here. (P-Regler ;-) )
 *
 *  @return A desired rotation.
 */
float
CMediumBackwardDriveModule::MediumBackward_Curvature( float dist_to_target, float dist_to_trajec, float alpha,
                                                      float trans_0, float rot_0 )
{
  return 1.4*alpha;
}




/** Calculate by given variables a new translation to give for the motor to
 *    minimize distance to the target.
 *
 *  DOC.: This here is a fairly easy routine after the previous one ;-). It calculates
 *        to a proposed rotation the translation part. This means, in relation to
 *        distances to target and trajec and the current values we calculate a new one.
 *
 *  @return A desired translation.
 */
float
CMediumBackwardDriveModule::MediumBackward_Translation ( float dist_to_target, float dist_to_front, float alpha,
                                                         float trans_0, float rot_0, float rot_1 )
{
  float trans_1 = 0.0;

  if ( fabs( rot_1 ) >= 0.0 && fabs( rot_1 ) <= 0.5 )
    trans_1 = LinInterpol( fabs( rot_1 ), 0.5, 0.0, 1.7, fabs(m_MaxTranslation+0.05) );

  else if ( fabs( rot_1 ) > 0.5 && fabs( rot_1 ) <= 1.0 )
    trans_1 = LinInterpol( fabs( rot_1 ), 1.0, 0.5, 1.2, 1.5 );

  else if ( fabs( rot_1 ) > 1.0 && fabs( rot_1 ) <= 1.8 )
    trans_1 = LinInterpol( fabs( rot_1 ), 1.8, 1.0, 0.8, 1.2 );

  else if ( fabs( rot_1 ) > 1.8 )
    trans_1 = LinInterpol( fabs( rot_1 ), M_PI, 1.8, 0.0, 0.8 );

  else {
    logger_->log_debug("CMediumBackwardDriveModule", "************ MEDIUM DRIVE MODES:::NOT DEFINED STATE!!!!! EXITING");
    trans_1 = 0;
  }



  // test the borders (no agressive behaviour!)
  if ( trans_1 > 0.0 )
    trans_1 = 0.0;
  if ( trans_1 < m_MaxTranslation )
    trans_1 = m_MaxTranslation;

  // OLD STUFF
  //   // check stopping on target and compare distances with choosen velocities
  //   if ( fabs( dist_to_target - dist_to_front ) < 0.2 )
  //     {
  //       if (m_StopAtTarget == true)
  //  trans_1 = min( trans_1, dist_to_target*1.5 );
  //       else
  //  ; // do not stop, so drive behind the target with full power
  //     }
  //   else
  //     {
  //       trans_1 = min( trans_1, dist_to_front );
  //     }


  // NEW STUFF
  float trans_target = 10000.0;
  float trans_front  = 10000.0;

  if ( m_StopAtTarget == true )
    trans_target = GuaranteeTransStop( dist_to_target, trans_0, trans_1 );

  // And the next collision point
  if ( dist_to_front < dist_to_target )
    trans_front = GuaranteeTransStop( dist_to_front, trans_0, trans_1 );
  // NEW STUFF END HERE

  trans_1 = std::min( trans_1, std::min( trans_target, trans_front ) );

  return trans_1;
}




/* ************************************************************************** */
/* ***********************        U P D A T E       ************************* */
/* ************************************************************************** */

/** Calculate here your desired settings. What you desire is checked afterwards to the current
 *    settings of the physical boundaries, but take care also.
 *
 *  How you do this is up to you, but be careful, our hardware is expensive!!!!
 *
 *  Available are:
 *
 *     m_TargetX, m_TargetY, m_TargetOri  --> current Target to drive to
 *     m_RoboX, m_RoboY, m_RoboOri        --> current Robot coordinates
 *     m_RoboTrans, m_RoboRot             --> current Motor values
 *
 *     m_LocalTargetX, m_LocalTargetY     --> our local target found by the search component we want to reach
 *     m_LocalTrajecX, m_LocalTrajecY     --> The point we would collide with, if we would drive WITHOUT Rotation
 *
 *     m_OrientAtTarget                   --> Do we have to orient ourself at the target?
 *     m_StopAtTarget                     --> Do we have to stop really ON the target?
 *
 *  Afterwards filled should be:
 *
 *     m_ProposedTranslation              --> Desired Translation speed
 *     m_ProposedRotation                 --> Desired Rotation speed
 *
 *  Those values are questioned after an Update() was called.
 */
void CMediumBackwardDriveModule::Update()
{
  m_ProposedTranslationX  = 0.;
  m_ProposedTranslationY  = 0.;
  m_ProposedRotation      = 0.;

  float dist_to_target = sqrt( sqr(m_LocalTargetX) + sqr(m_LocalTargetY) );
  float alpha          = normalize_mirror_rad(atan2( m_LocalTargetY, m_LocalTargetX ) + M_PI);
  float dist_to_trajec = sqrt( sqr(m_LocalTrajecX) + sqr(m_LocalTrajecY) );


  m_ProposedRotation = MediumBackward_Curvature( dist_to_target, dist_to_trajec, alpha,
                                                 -m_RoboTrans, -m_RoboRot );


  if ( fabs( alpha ) > M_PI_2+0.2 )
    m_ProposedTranslationX = 0.0;
  else
    m_ProposedTranslationX = MediumBackward_Translation( dist_to_target, dist_to_trajec, alpha,
                                                        m_RoboTrans, m_RoboRot, m_ProposedRotation);


  // last time border check............. IMPORTANT!!!
  // because the motorinstructor just tests robots physical borders.
  if ( dist_to_target < 0.04 ) {
    m_ProposedTranslationX = 0.0;
    m_ProposedRotation    = 0.0;

  } else {
    m_ProposedTranslationX  = std::min ( m_ProposedTranslationX, m_MaxTranslation );
    m_ProposedTranslationX  = std::max ( m_ProposedTranslationX, (float)0.0 );
    m_ProposedTranslationX *= -1;

    if (m_ProposedRotation >  m_MaxRotation)
      m_ProposedRotation =  m_MaxRotation;

    if (m_ProposedRotation < -m_MaxRotation)
      m_ProposedRotation = -m_MaxRotation;


    if ( m_StopAtTarget == false && dist_to_target < 1.5 ) {
      // Reduziere die rotationsgeschwindigkeiten, damit keine wilden lenkmanoever kommen
      if ( m_ProposedRotation > 0.6 )
        m_ProposedRotation =  0.6;
      else if ( m_ProposedRotation < -0.6 )
        m_ProposedRotation = -0.6;
    }
  }
}

} // namespace fawkes
