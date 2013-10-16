/***************************************************************************
 *  colli_thread.cpp - Colli Thread
 *
 *  Created: Sat Jul 13 12:00:00 2013
 *  Copyright  2013  AllemaniACs
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

#include "colli_thread.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/NavigatorInterface.h>
#include <utils/math/common.h>

#include <string>

using namespace fawkes;
using namespace std;

ColliThread::ColliThread()
  : Thread("ColliThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

ColliThread::~ColliThread()
{
}

void
ColliThread::init()
{
  logger->log_info(name(), "(init): Constructing...");

  std::string cfg_prefix = "/plugins/colli/";

  m_ColliFrequency      = (int)(1000.0/(float)config->get_int((cfg_prefix + "Colli_FREQUENCY").c_str()));
  m_OccGridHeight       = config->get_float((cfg_prefix + "OccGrid_HEIGHT").c_str());
  m_OccGridWidth        = config->get_float((cfg_prefix + "OccGrid_WIDTH").c_str());
  m_OccGridCellHeight   = config->get_int((cfg_prefix + "OccGrid_CELL_HEIGHT").c_str());
  m_OccGridCellWidth    = config->get_int((cfg_prefix + "OccGrid_CELL_WIDTH").c_str());
  m_MaximumRoboIncrease = config->get_float((cfg_prefix + "Colli_MAX_ROBO_INCREASE").c_str());

  m_RobocupMode         = config->get_int((cfg_prefix + "Colli_ROBOCUP_MODE").c_str());

  std::string default_hostname=""; // RCSoft was: =GetBBNames()[0];
  /* As default we use a football player AllemaniACs robot */
  if (default_hostname == "carl_rc.informatik.rwth-aachen.de") {
    isRwiRobot = true;
    logger->log_info(name(), "(init): Using a RWI Robot so this effects robs position in grid");
  } else {
    isRwiRobot = false;
    logger->log_info(name(), "(init): Using Colli for an AllemaniACs IKEA Style Robot");
  }

  for ( unsigned int i = 0; i < 10; i++ )
    m_oldAnglesToTarget.push_back( 0.0 );

  srand( time( NULL ) );

  logger->log_info(name(), "(init): Entering initialization ..." );

  RegisterAtBlackboard();
  InitializeModules();

  //~ SetTime( m_ColliFrequency );
  //TODO: this defines how often the thread should be called! We should integrate that
  //into the loop() method

  m_oldTargetX   = m_pColliTargetObj->dest_x();
  m_oldTargetY   = m_pColliTargetObj->dest_y();
  m_oldTargetOri = m_pColliTargetObj->dest_ori();

  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  escape_count = 0;

  logger->log_info(name(), "(init): Initialization done.");
}


void
ColliThread::finalize()
{
  logger->log_info(name(), "(finalize): Entering destructing ...");

  // delete own modules
  //~ delete m_pSelectDriveMode;
  //~ delete m_pSearch;
  //~ delete m_pLaserOccGrid;
  //~ delete m_pLaser;
  //~ delete m_pMotorInstruct;

  // close all registered bb-interfaces
  blackboard->close( m_pColliDataObj );
  blackboard->close( m_pColliTargetObj );
  blackboard->close( m_pLaserScannerObj );
  blackboard->close( m_pMopoObj );

  logger->log_info(name(), "(finalize): Destructing done.");
}




/* **************************************************************************** */
/* **************************************************************************** */
/* ******************************  L O O P  *********************************** */
/* **************************************************************************** */
/* **************************************************************************** */

//
// ============================================================================ //
// ============================================================================ //
//                               BBCLIENT LOOP                                  //
//                               *************                                  //
//                                                                              //
//           The desired structure should be something like this                //
//           ===================================================                //
//                                                                              //
// Update the BB Things                                                         //
// Update the state machine                                                     //
//                                                                              //
// If we are in stop state                                                      //
//    Do stop                                                                   //
// Else if we are in orient state                                               //
//    Do orient                                                                 //
// else if we are in a drive state                                              //
//    Update the grid                                                           //
//    If we are to close to an obstacle                                         //
//       Escape the obstacle                                                    //
//       Get Motor settings for escaping                                        //
//       Set Motor parameters for escaping                                      //
//    else                                                                      //
//       Search for a way                                                       //
//       if we found a way,                                                     //
//          Translate the way in motor things                                   //
//          Set Motor parameters for driving                                    //
//       else                                                                   //
//          do nothing, because this is an error!                               //
//          Set Motor parameters for stopping                                   //
//                                                                              //
// Translate and Realize the motor commands                                     //
// Update the BB Things                                                         //
//                                                                              //
// ============================================================================ //
// ============================================================================ //
//
void
ColliThread::loop()
{
  // to be on the sure side of life
  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  // Update blackboard data
  UpdateBB();

  if( !m_pLaserScannerObj->has_writer()
   || !m_pMopoObj->has_writer() ) {
    logger->log_warn(name(), "***** Laser or sim_robot dead!!! --> STOPPING!!!!");
    //~ m_pMotorInstruct->Drive( 0.0, 0.0 );
    m_pColliDataObj->set_final( true );
    m_pColliDataObj->write();
    escape_count = 0;
    return;
  }

  // THIS IF FOR CHALLENGE ONLY!!!
  if( m_pColliTargetObj->drive_mode() == NavigatorInterface::OVERRIDE ) {
    logger->log_debug(name(), "BEING OVERRIDDEN!");
    m_pColliDataObj->set_final( false );
    m_pColliDataObj->write();
    escape_count = 0;
    return;

  } else if( m_pColliTargetObj->drive_mode() == NavigatorInterface::MovingNotAllowed ) {
    logger->log_debug(name(), "Moving is not allowed!");
    //~ m_pMotorInstruct->Drive( 0.0, 0.0 );
    m_pColliDataObj->set_final( true );
    m_pColliDataObj->write();
    escape_count = 0;
    return;
  }


  // Do only drive, if there is a new (first) target
  if( ( m_oldTargetX   == m_pColliTargetObj->dest_x() )
   && ( m_oldTargetY   == m_pColliTargetObj->dest_y() )
   && ( m_oldTargetOri == m_pColliTargetObj->dest_ori() ) ) {

    m_oldAnglesToTarget.clear();
    for ( unsigned int i = 0; i < 10; i++ )
      m_oldAnglesToTarget.push_back( 0.0 );

    m_ProposedTranslation = 0.0;
    m_ProposedRotation    = 0.0;
    m_pColliDataObj->set_final( true );
    //~ m_pMotorInstruct->Drive( m_ProposedTranslation, m_ProposedRotation );

    escape_count = 0;
    // Send motor and colli data away.
    m_pColliDataObj->write();

    return;

  } else {
    m_oldTargetX   = m_pColliTargetObj->dest_x()   + 1000.0;
    m_oldTargetY   = m_pColliTargetObj->dest_y()   + 1000.0;
    m_oldTargetOri = m_pColliTargetObj->dest_ori() + 1.0;
  }

  // Update state machine
  UpdateColliStateMachine();

  // nothing is to do
  if (m_ColliStatus == NothingToDo) {
    //~ m_pLaserOccGrid->ResetOld();
    m_ProposedTranslation = 0.0;
    m_ProposedRotation    = 0.0;
    m_pColliDataObj->set_final( true );

    m_oldTargetX   = m_pColliTargetObj->dest_x();
    m_oldTargetY   = m_pColliTargetObj->dest_y();
    m_oldTargetOri = m_pColliTargetObj->dest_ori();
    //~ m_pLaserOccGrid->ResetOld();

    escape_count = 0;

  } else {
    //TODO: the actual movement stuff :)
  }

  logger->log_debug(name(), "I want to realize %f , %f", m_ProposedTranslation, m_ProposedRotation);

  // Realize drive mode proposal with realization module
  //~ m_pMotorInstruct->Drive( m_ProposedTranslation, m_ProposedRotation );

  // Send motor and colli data away.
  m_pColliDataObj->write();
}


/* **************************************************************************** */
/* **************************************************************************** */
/* ****************** P R I V A T E  -   S T U F F **************************** */
/* **************************************************************************** */
/* **************************************************************************** */


/* **************************************************************************** */
/*                       Initialization                                         */
/* **************************************************************************** */



/// Register all BB-Interfaces at the Blackboard.
void
ColliThread::RegisterAtBlackboard()
{
  m_pMopoObj = blackboard->open_for_reading<MotorInterface>("Motor");

  // only use the obstacle laser scanner here!
  m_pLaserScannerObj = blackboard->open_for_reading<Laser360Interface>("Laser laser merged");

  m_pColliTargetObj = blackboard->open_for_writing<NavigatorInterface>("Colli local target");

  m_pColliDataObj = blackboard->open_for_writing<NavigatorInterface>("Navigator");

  m_pMopoObj->read();
  m_pLaserScannerObj->read();

  m_pColliDataObj->set_final(true);
  m_pColliDataObj->write();
}


/// Initialize all modules used by the Colli
void
ColliThread::InitializeModules()
{
  // FIRST(!): the laserinterface (uses the laserscanner)
  //~ m_pLaser = new Laser( m_pLaserScannerObj, m_pXMLConfigFile );
  //~ m_pLaser->UpdateLaser();

  // SECOND(!): the occupancy grid (it uses the laser)
  //~ m_pLaserOccGrid = new CLaserOccupancyGrid( m_pLaser );

  // set the cell width and heigth to 5 cm and the grid size to 7.5 m x 7.5 m.
  // this are 750/5 x 750/5 grid cells -> (750x750)/5 = 22500 grid cells
  //~ m_pLaserOccGrid->setCellWidth(  m_OccGridCellWidth );
  //~ m_pLaserOccGrid->setWidth(  (int)((m_OccGridWidth*100)/m_pLaserOccGrid->getCellWidth()) );
  //~ m_pLaserOccGrid->setCellHeight( m_OccGridCellHeight );
  //~ m_pLaserOccGrid->setHeight( (int)((m_OccGridHeight*100)/m_pLaserOccGrid->getCellHeight()) );

  // THIRD(!): the search component (it uses the occ grid (without the laser)
  //~ m_pSearch = new CSearch( m_pLaserOccGrid );


  // BEFORE DRIVE MODE: the motorinstruction set
  //~ m_pMotorInstruct = (CBaseMotorInstruct *)new CQuadraticMotorInstruct( m_pMopoObj, m_ColliFrequency );
  //~ m_pMotorInstruct->SetRecoverEmergencyStop();


  // AFTER MOTOR INSTRUCT: the motor propose values object
  //~ m_pSelectDriveMode = new CSelectDriveMode( m_pMotorInstruct, m_pLaser, m_pColliTargetObj );

  // Initialization of colli state machine:
  // Currently nothing is to accomplish
  m_ColliStatus  = NothingToDo;
  m_oldTargetX   = m_pColliTargetObj->dest_x();
  m_oldTargetY   = m_pColliTargetObj->dest_y();
  m_oldTargetOri = m_pColliTargetObj->dest_ori();

  //~ m_OldX   = m_pMotorInstruct->GetCurrentX();
  //~ m_OldY   = m_pMotorInstruct->GetCurrentY();
  //~ m_OldOri = m_pMotorInstruct->GetCurrentOri();
}




/* **************************************************************************** */
/*                          During Runtime                                      */
/* **************************************************************************** */



/// Get the newest values from the blackboard
void
ColliThread::UpdateBB()
{
  m_pLaserScannerObj->read();
  m_pMopoObj->read();
  m_pColliTargetObj->write();
  m_pColliDataObj->write();
}



void
ColliThread::UpdateColliStateMachine()
{
  // initialize
  m_ColliStatus = NothingToDo;

  float curPosX = 0;//m_pMotorInstruct->GetCurrentX(); //TODO
  float curPosY = 0;//m_pMotorInstruct->GetCurrentY(); //TODO
  float curPosO = 0;//m_pMotorInstruct->GetCurrentOri(); //TODO

  float targetX = m_pColliTargetObj->dest_x();
  float targetY = m_pColliTargetObj->dest_y();
  float targetO = m_pColliTargetObj->dest_ori();

  bool  orient = true; // m_pColliTargetObj->OrientAtTarget(); //TODO
  //  bool  stop_on_target =  m_pColliTargetObj->StopOnTarget();

//   if ( stop_on_target == false )
//     {
//       float angle_to_target = atan2( targetY - curPosY,
//             targetX - curPosX );
//       std::vector< float > new_angles;
//       for ( unsigned int i = 0; i < m_oldAnglesToTarget.size(); i++ )
//  new_angles.push_back( m_oldAnglesToTarget[i] );
//       m_oldAnglesToTarget.clear();

//       for ( unsigned int i = 0; i < 9; i++ )
//  m_oldAnglesToTarget.push_back( new_angles[i] );

//       m_oldAnglesToTarget.push_back( angle_to_target );


//       // vergleiche die angles mit dem neusten. Wenn wir nen M_PI bekommen, dann fertig
//       for ( unsigned int i = 0; i < m_oldAnglesToTarget.size()-1; i++ )
//  {
//    if ( fabs( normalize_mirror_rad( m_oldAnglesToTarget[m_oldAnglesToTarget.size()-1] -
//             m_oldAnglesToTarget[i] ) ) > 2.5 )
//      {
//        cout << "Detected final stop!" << endl;
//        if ( orient == true )
//    {
//      m_ColliStatus = OrientAtTarget;
//    }
//        else
//    {
//      m_ColliStatus = NothingToDo;
//    }
//        m_oldAnglesToTarget.clear();
//        for ( unsigned int i = 0; i < 10; i++ )
//    m_oldAnglesToTarget.push_back( 0.0 );
//        return;
//      }
//  }
//     }

  // Real driving....
  if( ( orient == true )
   && ( sqr( curPosX - targetX ) + sqr( curPosY - targetY ) >= sqr(2.1) ) ) {

   float ori = m_pColliTargetObj->dest_ori();

    float mult = 0.0;
    //~ if ( m_pMotorInstruct->GetUserDesiredTranslation() > 0 )
      //~ //  mult =  1.2;
      //~ mult = 0.8;
    //~ else if ( m_pMotorInstruct->GetUserDesiredTranslation() < 0 )
      //~ //  mult = -1.2;
      //~ mult = -0.8;

    float orientPointX = targetX - ( mult * cos(ori) );
    float orientPointY = targetY - ( mult * sin(ori) );

    m_TargetPointX = orientPointX;
    m_TargetPointY = orientPointY;
    m_ColliStatus = DriveToOrientPoint;
    return;

  } else if( sqr( curPosX - targetX ) + sqr( curPosY - targetY ) > sqr(0.15) )  { // soll im navigator wegen intercept parametrisierbar sein
    m_TargetPointX = targetX;
    m_TargetPointY = targetY;
    m_ColliStatus = DriveToTarget;
    return;

  } else if ( (orient == true) && ( fabs( normalize_mirror_rad(curPosO - targetO) ) > 0.1 ) ) {
    m_ColliStatus = OrientAtTarget;
    return;

  } else {
    m_ColliStatus = NothingToDo;
    return;
  }

  return;
}