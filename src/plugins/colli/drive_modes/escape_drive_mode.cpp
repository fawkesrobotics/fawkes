
/***************************************************************************
 *  escape_drive_mode.cpp - Implementation of drive-mode "escape"
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


#include "escape_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Initialize your local values here.
 */
CEscapeDriveModule::CEscapeDriveModule( Laser* laser, Logger* logger, Configuration* config )
 : CAbstractDriveMode(logger, config)
{
  logger_->log_info("CEscapeDriveModule", "(Constructor): Entering...");
  m_DriveModeName = NavigatorInterface::ESCAPE;
  m_pLaser = laser;

  m_MaxTranslation = config_->get_float( "/plugins/colli/EscapeDriveModule/MAX_TRANS" );
  m_MaxRotation    = config_->get_float( "/plugins/colli/EscapeDriveModule/MAX_ROT" );

  m_pRoboShape = new CRoboShape_Colli( "/plugins/colli/Roboshape/", logger, config, 2 );

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

  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  FillNormalizedReadings();
  SortNormalizedReadings();

  bool dangerFront = CheckDanger( m_vFront );
  bool dangerBack  = CheckDanger( m_vBack  );

  bool dangerLeft  = ( CheckDanger( m_vLeftFront ) || CheckDanger( m_vLeftBack ) );
  bool dangerRight = ( CheckDanger( m_vRightFront ) || CheckDanger( m_vRightBack ) );

  bool turnLeftAllowed = TurnLeftAllowed();
  bool turnRightAllowed = TurnRightAllowed();

  if (dangerFront)
    logger_->log_warn("CEscapeDriveModule", "DANGER IN FRONT");

  if (dangerBack)
    logger_->log_warn("CEscapeDriveModule", "DANGER IN BACK");

  if (dangerLeft)
    logger_->log_warn("CEscapeDriveModule", "DANGER IN LEFT");

  if (dangerRight)
    logger_->log_warn("CEscapeDriveModule", "DANGER IN RIGHT");

  if (!turnLeftAllowed)
    logger_->log_warn("CEscapeDriveModule", "DANGER IF TURNING LEFT!!!");

  if (!turnRightAllowed)
    logger_->log_warn("CEscapeDriveModule", "DANGER IF TURNING RIGHT!!!");


  if ( dangerFront && dangerBack && turnRightAllowed ) {
    m_ProposedTranslation = 0.0;
    m_ProposedRotation = -m_MaxRotation;
    return;
  }

  if ( dangerFront && dangerBack && turnLeftAllowed ) {
    m_ProposedTranslation = 0.0;
    m_ProposedRotation = m_MaxRotation;
    return;
  }

  if (!dangerFront && dangerBack) {
    m_ProposedTranslation = m_MaxTranslation;

    if ( (turnRightAllowed) && (m_LocalTargetY <= m_RoboY) )
      m_ProposedRotation =  -m_MaxRotation;
    else if ( (turnLeftAllowed) && (m_LocalTargetY >= m_RoboY) )
      m_ProposedRotation = m_MaxRotation;
  }

  if (dangerFront && !dangerBack) {
    m_ProposedTranslation = -m_MaxTranslation;

    if ( (turnRightAllowed) && (m_LocalTargetY <= m_RoboY) )
      m_ProposedRotation =  -m_MaxRotation;
    else if ( (turnLeftAllowed) && (m_LocalTargetY >= m_RoboY) )
      m_ProposedRotation = m_MaxRotation;
  }

  if ( !dangerFront && !dangerBack ) {
    // entscheide ueber die zielkoordinaten welche richtung einzuschlagen ist
    if ( m_TargetX > m_RoboX )
      m_ProposedTranslation = m_MaxTranslation;
    else
      m_ProposedTranslation = -m_MaxTranslation;

    if ( (turnRightAllowed) && (m_LocalTargetY <= m_RoboY) )
      m_ProposedRotation =  -m_MaxRotation;
    else if ( (turnLeftAllowed) && (m_LocalTargetY >= m_RoboY) )
      m_ProposedRotation = m_MaxRotation;
  }
}


/* ************************************************************************** */
/* ***********************     Private Methods      ************************* */
/* ************************************************************************** */

void
CEscapeDriveModule::FillNormalizedReadings()
{
  m_vNormalizedReadings.clear();

  for ( int i = 0; i < m_pLaser->GetNumberOfReadings(); i++ ) {
    float rad    = normalize_rad( m_pLaser->GetRadiansForReading( i ) );
    float sub    = m_pRoboShape->GetRobotLengthforRad( rad );
    float length = m_pLaser->GetReadingLength( i );
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

  int pipe = 0;
  int i = 0;
  float rad = normalize_rad( m_pLaser->GetRadiansForReading( i ) );

  while ( i < m_pLaser->GetNumberOfReadings() ) {

    if ( (pipe == 0) && !m_pLaser->IsPipe( rad ) )
      m_vFront.push_back( m_vNormalizedReadings[i] );

    else if ( (pipe == 1) && !m_pLaser->IsPipe( rad ) && (rad < M_PI_2) )
      m_vLeftFront.push_back( m_vNormalizedReadings[i] );

    else if ( (pipe == 1) && !m_pLaser->IsPipe( rad ) && (rad > M_PI_2) )
      m_vLeftBack.push_back( m_vNormalizedReadings[i] );

    else if ( (pipe == 2) && !m_pLaser->IsPipe( rad ) )
      m_vBack.push_back( m_vNormalizedReadings[i] );

    else if ( (pipe == 3) && !m_pLaser->IsPipe( rad ) && (rad > 3*M_PI_2) )
      m_vRightFront.push_back( m_vNormalizedReadings[i] );

    else if ( (pipe == 3) && !m_pLaser->IsPipe( rad ) && (rad < 3*M_PI_2) )
      m_vRightBack.push_back( m_vNormalizedReadings[i] );

    else if ( (pipe == 4) && !m_pLaser->IsPipe( rad ) )
      m_vFront.push_back( m_vNormalizedReadings[i] );

    rad = m_pLaser->GetRadiansForReading( ++i );

    if ( m_pLaser->IsOnlyPipe( rad ) ) {
      ++pipe;
      while (m_pLaser->IsOnlyPipe( rad )) {
        rad = m_pLaser->GetRadiansForReading( ++i );
      }
    }
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