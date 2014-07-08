
/***************************************************************************
 *  escape_drive_mode.cpp - Implementation of drive-mode "escape"
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


#include "escape_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CEscapeDriveModule <plugins/colli/drive_modes/escape_drive_mode.h>
 * Class Escape-Drive-Module. This module is called, if an escape is neccessary.
 * It should try to maximize distance to the disturbing obstacle.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
CEscapeDriveModule::CEscapeDriveModule( Logger* logger, Configuration* config )
 : CAbstractDriveMode(logger, config)
{
  logger_->log_info("CEscapeDriveModule", "(Constructor): Entering...");
  m_DriveModeName = NavigatorInterface::ESCAPE;

  m_MaxTranslation = config_->get_float( "/plugins/colli/drive_mode/escape/max_trans" );
  m_MaxRotation    = config_->get_float( "/plugins/colli/drive_mode/escape/max_rot" );

  m_pRoboShape = new CRoboShape_Colli( "/plugins/colli/roboshape/", logger, config, 2 );

  logger_->log_info("CEscapeDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here.
 */
CEscapeDriveModule::~CEscapeDriveModule()
{
  logger_->log_info("CEscapeDriveModule", "(Destructor): Entering...");
  logger_->log_info("CEscapeDriveModule", "(Destructor): Exiting...");
}



/* ************************************************************************** */
/* ***********************        U P D A T E       ************************* */
/* ************************************************************************** */

/** Calculate here your desired settings. What you desire is checked afterwards to the current
 *    settings of the physical boundaries, but take care also.
 *
 *  How you do this is up to you, but be careful, our hardware is expensive!!!!
 *
 *  All values of the other drive modes inherited by the abstract-drive-mode are
 *    non-valid, because search did not succeed or should not have been called!
 *    So do not use them. Instead here you use the m_pLaser!
 *
 *  Afterwards filled should be:
 *
 *     m_ProposedTranslation              --> Desired Translation speed
 *     m_ProposedRotation                 --> Desired Rotation speed
 *
 *  Those values are questioned after an Update() was called.
 */
void
CEscapeDriveModule::Update()
{
  // This is only called, if we recently stopped...
  logger_->log_debug("CEscapeDriveModule", "CEscapeDriveModule( Update ): Calculating ESCAPING...");

  m_ProposedTranslationX  = 0.;
  m_ProposedTranslationY  = 0.;
  m_ProposedRotation      = 0.;

  FillNormalizedReadings();
  SortNormalizedReadings();

  bool dangerFront = CheckDanger( m_vFront );
  bool dangerBack  = CheckDanger( m_vBack  );

  bool turnLeftAllowed = TurnLeftAllowed();
  bool turnRightAllowed = TurnRightAllowed();

  if (dangerFront)
    logger_->log_debug("CEscapeDriveModule", "DANGER IN FRONT");

  if (dangerBack)
    logger_->log_debug("CEscapeDriveModule", "DANGER IN BACK");

  if (CheckDanger(m_vLeftFront))
    logger_->log_debug("CEscapeDriveModule", "DANGER IN LEFT FRONT");

  if (CheckDanger(m_vLeftBack))
    logger_->log_debug("CEscapeDriveModule", "DANGER IN LEFT BACK");

  if (CheckDanger(m_vRightFront))
    logger_->log_debug("CEscapeDriveModule", "DANGER IN RIGHT FRONT");

  if (CheckDanger(m_vRightBack))
    logger_->log_debug("CEscapeDriveModule", "DANGER IN RIGHT BACK");

  if (!turnLeftAllowed)
    logger_->log_debug("CEscapeDriveModule", "DANGER IF TURNING LEFT!!!");

  if (!turnRightAllowed)
    logger_->log_debug("CEscapeDriveModule", "DANGER IF TURNING RIGHT!!!");


  if ( dangerFront && dangerBack && turnRightAllowed ) {
    m_ProposedTranslationX = 0.0;
    m_ProposedRotation = -m_MaxRotation;

  } else if ( dangerFront && dangerBack && turnLeftAllowed ) {
    m_ProposedTranslationX = 0.0;
    m_ProposedRotation = m_MaxRotation;

  } else if (!dangerFront && dangerBack) {
    m_ProposedTranslationX = m_MaxTranslation;

    if ( (turnRightAllowed) && (m_LocalTargetY <= m_RoboY) )
      m_ProposedRotation =  -m_MaxRotation;
    else if ( (turnLeftAllowed) && (m_LocalTargetY >= m_RoboY) )
      m_ProposedRotation = m_MaxRotation;

  } else if (dangerFront && !dangerBack) {
    m_ProposedTranslationX = -m_MaxTranslation;

    if ( (turnRightAllowed) && (m_LocalTargetY <= m_RoboY) )
      m_ProposedRotation =  -m_MaxRotation;
    else if ( (turnLeftAllowed) && (m_LocalTargetY >= m_RoboY) )
      m_ProposedRotation = m_MaxRotation;

  } else if ( !dangerFront && !dangerBack ) {
    // depending on target coordinates, decide which direction to escape to
    if ( m_TargetX > m_RoboX )
      m_ProposedTranslationX = m_MaxTranslation;
    else
      m_ProposedTranslationX = -m_MaxTranslation;

    if ( (turnRightAllowed) && (m_LocalTargetY <= m_RoboY) )
      m_ProposedRotation =  -m_MaxRotation;
    else if ( (turnLeftAllowed) && (m_LocalTargetY >= m_RoboY) )
      m_ProposedRotation = m_MaxRotation;
  }
}

/**
 * This function sets the laser points for one escape round
 * @param laser_points vector of laser points
 */
void
CEscapeDriveModule::setLaserData( std::vector<CEscapeDriveModule::LaserPoint>& laser_points )
{
  m_laser_points = laser_points;
}

/* ************************************************************************** */
/* ***********************     Private Methods      ************************* */
/* ************************************************************************** */

void
CEscapeDriveModule::FillNormalizedReadings()
{
  m_vNormalizedReadings.clear();

  for ( int i = 0; i < (int)m_laser_points.size(); i++ ) {
    float rad    = normalize_rad( m_laser_points.at( i ).angle );
    float sub    = m_pRoboShape->GetRobotLengthforRad( rad );
    float length = m_laser_points.at( i ).length;
    m_vNormalizedReadings.push_back( length - sub );
  }
}


void
CEscapeDriveModule::SortNormalizedReadings()
{
  m_vFront.clear();
  m_vBack.clear();
  m_vLeftFront.clear();
  m_vLeftBack.clear();
  m_vRightFront.clear();
  m_vRightBack.clear();

  float ang_fl = normalize_rad(m_pRoboShape->GetAngleFrontLeft());
  float ang_fr = normalize_rad(m_pRoboShape->GetAngleFrontRight());
  float ang_bl = normalize_rad(m_pRoboShape->GetAngleBackLeft());
  float ang_br = normalize_rad(m_pRoboShape->GetAngleBackRight());
  float ang_ml = normalize_rad(m_pRoboShape->GetAngleLeft());
  float ang_mr = normalize_rad(m_pRoboShape->GetAngleRight());

  if(!( (ang_fl < ang_ml) && (ang_ml < ang_bl) && (ang_bl < ang_br)
     &&(ang_br < ang_mr) && (ang_mr < ang_fr) ))
    logger_->log_error("RoboShape", "Angles are bad!!!");

  int i = 0;
  float rad = 0.f;

  while ( i < (int)m_laser_points.size() ) {
    if( m_laser_points.at(i).length > 0. ) {
      rad = normalize_rad( m_laser_points.at(i).angle );

      if( rad < ang_fl || rad >= ang_fr )
        m_vFront.push_back( m_vNormalizedReadings[i] );

      else if( rad < ang_ml )
        m_vLeftFront.push_back( m_vNormalizedReadings[i] );

      else if( rad < ang_bl )
        m_vLeftBack.push_back( m_vNormalizedReadings[i] );

      else if( rad < ang_br )
        m_vBack.push_back( m_vNormalizedReadings[i] );

      else if( rad < ang_mr )
        m_vRightBack.push_back( m_vNormalizedReadings[i] );

      else if( rad < ang_fr )
        m_vRightFront.push_back( m_vNormalizedReadings[i] );
    }

    ++i;
  }
}


bool
CEscapeDriveModule::CheckDanger( std::vector< float > readings )
{
  // if something is smaller than 5 cm, you have to flee.......
  for ( unsigned int i = 0; i < readings.size(); i++ )
    if ( readings[i] < 0.06 )
      return true;

  return false;
}


bool
CEscapeDriveModule::TurnLeftAllowed()
{
  for ( unsigned int i = 0; i < m_vFront.size(); i++ )
    if ( m_vFront[i] < 0.12 )
      return false;

  for ( unsigned int i = 0; i < m_vRightFront.size(); i++ )
    if ( m_vRightFront[i] < 0.06 )
      return false;

  for ( unsigned int i = 0; i < m_vBack.size(); i++ )
    if ( m_vBack[i] < 0.07 )
      return false;

  for ( unsigned int i = 0; i < m_vLeftBack.size(); i++ )
    if ( m_vLeftBack[i] < 0.13 )
      return false;

  return true;
}



bool
CEscapeDriveModule::TurnRightAllowed()
{
  for ( unsigned int i = 0; i < m_vFront.size(); i++ )
    if ( m_vFront[i] < 0.12 )
      return false;

  for ( unsigned int i = 0; i < m_vLeftFront.size(); i++ )
    if ( m_vLeftFront[i] < 0.06 )
      return false;

  for ( unsigned int i = 0; i < m_vBack.size(); i++ )
    if ( m_vBack[i] < 0.07 )
      return false;

  for ( unsigned int i = 0; i < m_vRightBack.size(); i++ )
    if ( m_vRightBack[i] < 0.13 )
      return false;

  return true;
}

} // namespace fawkes
