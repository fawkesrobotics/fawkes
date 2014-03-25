
/***************************************************************************
 *  colli_thread.cpp - Fawkes Colli Thread
 *
 *  Created: Sat Jul 13 12:00:00 2013
 *  Copyright  2013  Bahram Maleki-Fard
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
#ifdef HAVE_VISUAL_DEBUGGING
 #include "visualization_thread.h"
#endif

#include "common/defines.h"
#include "drive_modes/select_drive_mode.h"
#include "drive_realization/quadratic_motor_instruct.h"
#include "utils/rob/robo_laser.h"
#include "search/og_laser.h"
#include "search/astar_search.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/NavigatorInterface.h>
#include <utils/math/common.h>

#include <string>

using namespace fawkes;
using namespace std;

/** @class ColliThread "colli_thread.h"
 * Thread that performs the navigation and collision avoidance algorithms.
 */

/** Constructor. */
ColliThread::ColliThread()
  : Thread("ColliThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    vis_thread_( 0 )
{
}

/** Destructor. */
ColliThread::~ColliThread()
{
}

void
ColliThread::init()
{
  logger->log_info(name(), "(init): Constructing...");

  std::string cfg_prefix = "/plugins/colli/";
  m_ColliFrequency      = (int)(1000.0/(float)config->get_int((cfg_prefix + "frequency").c_str()));
  m_MaximumRoboIncrease = config->get_float((cfg_prefix + "max_robo_increase").c_str());
  cfg_obstacle_inc_     = config->get_bool((cfg_prefix + "obstacle_increasement").c_str());

  cfg_visualize_idle_   = config->get_bool((cfg_prefix + "visualize_idle").c_str());

  cfg_min_rot_              = config->get_float((cfg_prefix + "min_rot").c_str());
  cfg_min_drive_dist_       = config->get_float((cfg_prefix + "min_drive_distance").c_str());
  cfg_min_long_dist_drive_  = config->get_float((cfg_prefix + "min_long_dist_drive").c_str());
  cfg_min_long_dist_prepos_ = config->get_float((cfg_prefix + "min_long_dist_prepos").c_str());
  cfg_min_rot_dist_         = config->get_float((cfg_prefix + "min_rot_distance").c_str());
  cfg_target_pre_pos_       = config->get_float((cfg_prefix + "pre_position_distance").c_str());

  cfg_frame_base_       = config->get_string((cfg_prefix + "frame/base").c_str());
  cfg_frame_laser_      = config->get_string((cfg_prefix + "frame/laser").c_str());

  cfg_iface_motor_      = config->get_string((cfg_prefix + "interface/motor").c_str());
  cfg_iface_laser_      = config->get_string((cfg_prefix + "interface/laser").c_str());

  cfg_prefix += "occ_grid/";
  m_OccGridWidth        = config->get_float((cfg_prefix + "width").c_str());
  m_OccGridHeight       = config->get_float((cfg_prefix + "height").c_str());
  m_OccGridCellWidth    = config->get_int((cfg_prefix + "cell_width").c_str());
  m_OccGridCellHeight   = config->get_int((cfg_prefix + "cell_height").c_str());

  for ( unsigned int i = 0; i < 10; i++ )
    m_oldAnglesToTarget.push_back( 0.0 );

  srand( time( NULL ) );

  logger->log_info(name(), "(init): Entering initialization ..." );

  RegisterAtBlackboard();
  InitializeModules();

#ifdef HAVE_VISUAL_DEBUGGING
  vis_thread_->setup(m_pLaserOccGrid, m_pSearch, m_pLaser);
#endif

  // adjust the frequency of how often loop() should be processed
  float fawkes_loop_time_ms = config->get_uint("/fawkes/mainapp/desired_loop_time") / 1000.f;
  loop_count_trigger_ = m_ColliFrequency / fawkes_loop_time_ms;
  logger->log_info(name(), "will process 1 loop() after %u main_loops", loop_count_trigger_);
  loop_count_ = 0;

  // get distance from laser to robot base
  laser_to_base_valid_ = false;
  tf::Stamped<tf::Point> p_laser;
  tf::Stamped<tf::Point> p_base( tf::Point(0,0,0), fawkes::Time(0,0), cfg_frame_base_);
  try {
    tf_listener->transform_point(cfg_frame_laser_, p_base, p_laser);
    laser_to_base_.x = p_laser.x();
    laser_to_base_.y = p_laser.y();
    logger->log_info(name(), "distance from laser to base: x:%f  y:%f", laser_to_base_.x, laser_to_base_.y);
    laser_to_base_valid_ = true;
  } catch(Exception &e) {
    logger->log_warn(name(), "Unable to transform %s to %s. Error: %s",
                     cfg_frame_base_.c_str(), cfg_frame_laser_.c_str(), e.what());
  }

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
  delete m_pSelectDriveMode;
  delete m_pSearch;
  delete m_pLaserOccGrid;
  delete m_pLaser;
  delete m_pMotorInstruct;

  // close all registered bb-interfaces
  blackboard->close( if_colli_data_ );
  blackboard->close( if_colli_target_ );
  blackboard->close( if_laser_ );
  blackboard->close( if_motor_ );

  logger->log_info(name(), "(finalize): Destructing done.");
}

/** Set the visualization thread.
 * By default, it is created by the plugin (colli_plugin.cpp) and passed to the colli_thread.
 * @param vis_thread Pointer to the visualization-thread
 */
void
ColliThread::set_vis_thread(ColliVisualizationThread* vis_thread)
{
  vis_thread_ = vis_thread;
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
  if( ++loop_count_ < loop_count_trigger_ )
    return;

  // reset loop_count
  loop_count_ = 0;

  // Do not continue if we don't have a valid transform from base to laser yet
  if( !laser_to_base_valid_ ) {
    try {
      tf::Stamped<tf::Point> p_laser;
      tf::Stamped<tf::Point> p_base( tf::Point(0,0,0), fawkes::Time(0,0), cfg_frame_base_);

      tf_listener->transform_point(cfg_frame_laser_, p_base, p_laser);
      laser_to_base_.x = p_laser.x();
      laser_to_base_.y = p_laser.y();
      logger->log_info(name(), "distance from laser to base: x:%f  y:%f", laser_to_base_.x, laser_to_base_.y);
      laser_to_base_valid_ = true;
    } catch(Exception &e) {
      logger->log_warn(name(), "Unable to transform %s to %s. Error: %s",
                      cfg_frame_base_.c_str(), cfg_frame_laser_.c_str(), e.what());
      return;
    }
  }

  // to be on the sure side of life
  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  // Update blackboard data
  UpdateBB();

  // check if we need to abort for some reason
  bool abort = false;
  if( !if_laser_->has_writer()
   || !if_motor_->has_writer() ) {
    logger->log_warn(name(), "***** Laser or sim_robot dead!!!");
    escape_count = 0;
    abort = true;

/*
    // THIS IF FOR CHALLENGE ONLY!!!
  } else if( if_colli_target_->drive_mode() == NavigatorInterface::OVERRIDE ) {
    logger->log_debug(name(), "BEING OVERRIDDEN!");
    if_colli_data_->set_final( false );
    if_colli_data_->write();
    escape_count = 0;
    abort = true;
*/

  } else if( if_colli_target_->drive_mode() == NavigatorInterface::MovingNotAllowed ) {
    //logger->log_debug(name(), "Moving is not allowed!");
    escape_count = 0;
    abort = true;

    // Do not drive if there is no new target
  } else if( if_colli_target_->is_final() ) {
    //logger->log_debug(name(), "No new target for colli...ABORT");
    m_oldAnglesToTarget.clear();
    for ( unsigned int i = 0; i < 10; i++ )
      m_oldAnglesToTarget.push_back( 0.0 );

    abort = true;
  }

  if( abort ) {
    // check if we need to stop the current colli movememtn
    if( !if_colli_data_->is_final() ) {
      //logger->log_debug(name(), "STOPPING");
      // colli is active, but for some reason we need to abort -> STOP colli movement
      if( abs(if_motor_->vx()) > 0.01f
       || abs(if_motor_->vy()) > 0.01f
       || abs(if_motor_->omega()) > 0.01f ) {
        // only stop movement, if we are moving
        m_pMotorInstruct->Drive( 0.0, 0.0 );
      } else {
        // movement has stopped, we are "final" now
        if_colli_data_->set_final( true );
        if_colli_data_->write();
      }
    }

#ifdef HAVE_VISUAL_DEBUGGING
    if( cfg_visualize_idle_ ) {
      UpdateOwnModules();
      vis_thread_->wakeup();
    }
#endif
    return;
  }

  // Update state machine
  UpdateColliStateMachine();

  // nothing is to do
  if (m_ColliStatus == NothingToDo) {
    m_pLaserOccGrid->ResetOld();
    m_ProposedTranslation = 0.0;
    m_ProposedRotation    = 0.0;
    if( abs(if_motor_->vx()) <= 0.01f
     && abs(if_motor_->vy()) <= 0.01f
     && abs(if_motor_->omega()) <= 0.01f ) {
      // we have stopped, can consider the colli final now
      //logger->log_debug(name(), "L, consider colli final now");
      if_colli_data_->set_final( true );
    }

    m_pLaserOccGrid->ResetOld();

    escape_count = 0;

#ifdef HAVE_VISUAL_DEBUGGING
    if( cfg_visualize_idle_ )
      UpdateOwnModules();
#endif

  } else {
    // perform the update of the grid.
    UpdateOwnModules();
    if_colli_data_->set_final( false );

    if( if_motor_->motor_state() == MotorInterface::MOTOR_DISABLED ) {
      if_motor_->msgq_enqueue(new MotorInterface::SetMotorStateMessage(MotorInterface::MOTOR_ENABLED));
      //TODO: return afterwards?! we are not controlling the motor directly as probably was the case in RCSoftX
    }

    // Check, if one of our positions (robo-, laser-gridpos is not valid) => Danger!
    if( CheckEscape() == true || escape_count > 0 ) {
      if( m_pMotorInstruct->GetMotorDesiredTranslation() == 0.0
       && m_pMotorInstruct->GetMotorDesiredRotation() == 0.0 ) {
        m_pLaserOccGrid->ResetOld();
      }

      // ueber denken und testen

      if( if_colli_target_->is_escaping_enabled() ) {
        // SJTODO: ERST wenn ich gestoppt habe, escape mode anwerfen!!!
        if (escape_count > 0)
          escape_count--;
        else {
          int rnd = (int)((rand())/(float)(RAND_MAX)) * 10; // + 5;
          escape_count = rnd;
          logger->log_warn(name(), "Escape: new round with %i", rnd);
        }

        logger->log_warn(name(), "Escape mode, escaping!");
        m_pSelectDriveMode->SetLocalTarget( m_LocalTarget.x, m_LocalTarget.y );
        m_pSelectDriveMode->Update( true );  // <-- this calls the ESCAPE mode!
        m_ProposedTranslation = m_pSelectDriveMode->GetProposedTranslation();
        m_ProposedRotation    = m_pSelectDriveMode->GetProposedRotation();

      } else {
        logger->log_warn(name(), "Escape mode, but not allowed!");
        m_ProposedTranslation = 0.0;
        m_ProposedRotation    = 0.0;
        escape_count = 0;
      }

    } else {
      // only orienting to do and moving possible

      if (m_ColliStatus == OrientAtTarget) {
        m_ProposedTranslation = 0.0;
        // turn faster if angle-diff is high
        //m_ProposedRotation    = 1.5*normalize_mirror_rad( if_colli_target_->GetTargetOri() -
        m_ProposedRotation    = 1.0*normalize_mirror_rad( if_colli_target_->dest_ori() -
                                                          m_pMotorInstruct->GetCurrentOri() );
        // need to consider minimum rotation velocity
        if ( m_ProposedRotation > 0.0 )
          m_ProposedRotation = std::min( if_colli_target_->max_rotation(), std::max( cfg_min_rot_, m_ProposedRotation));
        else
          m_ProposedRotation = std::max(-if_colli_target_->max_rotation(), std::min(-cfg_min_rot_, m_ProposedRotation));

        m_pLaserOccGrid->ResetOld();

      } else {
        // search for a path
        m_pSearch->Update( m_RoboGridPos.x, m_RoboGridPos.y,
                           (int)m_TargetGridPos.x, (int)m_TargetGridPos.y );
        if ( m_pSearch->UpdatedSuccessful() ) {
          // path exists
          m_LocalGridTarget = m_pSearch->GetLocalTarget();
          m_LocalGridTrajec = m_pSearch->GetLocalTrajec();

          // coordinate transformation from grid coordinates to relative robot coordinates
          m_LocalTarget.x = (m_LocalGridTarget.x - m_RoboGridPos.x)*m_pLaserOccGrid->getCellWidth()/100.0;
          m_LocalTarget.y = (m_LocalGridTarget.y - m_RoboGridPos.y)*m_pLaserOccGrid->getCellHeight()/100.0;

          m_LocalTrajec.x = (m_LocalGridTrajec.x - m_RoboGridPos.x)*m_pLaserOccGrid->getCellWidth()/100.0;
          m_LocalTrajec.y = (m_LocalGridTrajec.y - m_RoboGridPos.y)*m_pLaserOccGrid->getCellHeight()/100.0;

          // call appopriate drive mode
          m_pSelectDriveMode->SetLocalTarget( m_LocalTarget.x, m_LocalTarget.y );
          m_pSelectDriveMode->SetLocalTrajec( m_LocalTrajec.x, m_LocalTrajec.y );
          m_pSelectDriveMode->Update();
          m_ProposedTranslation = m_pSelectDriveMode->GetProposedTranslation();
          m_ProposedRotation    = m_pSelectDriveMode->GetProposedRotation();

        } else {
          // stop
          // logger->log_warn(name(), "Drive Mode: Update not successful ---> stopping!");
          m_LocalTarget.x = 0.f;
          m_LocalTarget.y = 0.f;
          m_LocalTrajec.x = 0.f;
          m_LocalTrajec.y = 0.f;
          m_ProposedTranslation = 0.f;
          m_ProposedRotation    = 0.f;
          m_pLaserOccGrid->ResetOld();
        }

        //TODO: we should not mis-use the NavigatorInterface for these colli-data..
        if_colli_data_->set_x( m_LocalTarget.x ); // waypoint X
        if_colli_data_->set_y( m_LocalTarget.y ); // waypoint Y
        if_colli_data_->set_dest_x( m_LocalTrajec.x ); // collision-point X
        if_colli_data_->set_dest_y( m_LocalTrajec.y ); // collision-point Y
      }
    }


  }

  logger->log_debug(name(), "I want to realize %f , %f", m_ProposedTranslation, m_ProposedRotation);

  // Realize drive mode proposal with realization module
  m_pMotorInstruct->Drive( m_ProposedTranslation, m_ProposedRotation );

  // Send motor and colli data away.
  if_colli_data_->write();

  // visualize the new information
#ifdef HAVE_VISUAL_DEBUGGING
  vis_thread_->wakeup();
#endif

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
  if_motor_     = blackboard->open_for_reading<MotorInterface>(cfg_iface_motor_.c_str());

  // only use the obstacle laser scanner here!
  if_laser_ = blackboard->open_for_reading<Laser360Interface>(cfg_iface_laser_.c_str());

  if_colli_target_ = blackboard->open_for_reading<NavigatorInterface>("Colli target");

  if_colli_data_ = blackboard->open_for_writing<NavigatorInterface>("Colli data");

  if_motor_->read();
  if_laser_->read();
  if_colli_target_->read();

  if_colli_data_->set_final(true);
  if_colli_data_->write();
}


/// Initialize all modules used by the Colli
void
ColliThread::InitializeModules()
{
  // FIRST(!): the laserinterface (uses the laserscanner)
  m_pLaser = new Laser( if_laser_, logger, config );
  m_pLaser->UpdateLaser();

  // SECOND(!): the occupancy grid (it uses the laser)
  m_pLaserOccGrid = new CLaserOccupancyGrid( m_pLaser, logger, config );

  // set the cell width and heigth to 5 cm and the grid size to 7.5 m x 7.5 m.
  // this are 750/5 x 750/5 grid cells -> (750x750)/5 = 22500 grid cells
  m_pLaserOccGrid->setCellWidth(  m_OccGridCellWidth );
  m_pLaserOccGrid->setWidth(  (int)((m_OccGridWidth*100)/m_pLaserOccGrid->getCellWidth()) );
  m_pLaserOccGrid->setCellHeight( m_OccGridCellHeight );
  m_pLaserOccGrid->setHeight( (int)((m_OccGridHeight*100)/m_pLaserOccGrid->getCellHeight()) );

  // THIRD(!): the search component (it uses the occ grid (without the laser)
  m_pSearch = new CSearch( m_pLaserOccGrid, logger, config );


  // BEFORE DRIVE MODE: the motorinstruction set
  m_pMotorInstruct = (CBaseMotorInstruct *)new CQuadraticMotorInstruct( if_motor_,
                                                                        m_ColliFrequency,
                                                                        logger,
                                                                        config );
  m_pMotorInstruct->SetRecoverEmergencyStop();


  // AFTER MOTOR INSTRUCT: the motor propose values object
  m_pSelectDriveMode = new CSelectDriveMode( m_pMotorInstruct, m_pLaser, if_colli_target_, logger, config );

  // Initialization of colli state machine:
  // Currently nothing is to accomplish
  m_ColliStatus  = NothingToDo;

  m_OldX   = m_pMotorInstruct->GetCurrentX();
  m_OldY   = m_pMotorInstruct->GetCurrentY();
  m_OldOri = m_pMotorInstruct->GetCurrentOri();
}




/* **************************************************************************** */
/*                          During Runtime                                      */
/* **************************************************************************** */



/// Get the newest values from the blackboard
void
ColliThread::UpdateBB()
{
  if_laser_->read();
  if_motor_->read();
  if_colli_target_->read();
  if_colli_data_->write();
}



void
ColliThread::UpdateColliStateMachine()
{
  // initialize
  if( if_colli_target_->changed() ) {
    // new target!
    m_ColliStatus = NothingToDo;
  }

  float curPosX = m_pMotorInstruct->GetCurrentX();
  float curPosY = m_pMotorInstruct->GetCurrentY();
  float curPosO = m_pMotorInstruct->GetCurrentOri();

  float targetX = if_colli_target_->dest_x();
  float targetY = if_colli_target_->dest_y();
  float targetO = if_colli_target_->dest_ori();

  bool  orient = if_colli_target_->is_orient_at_target();

  float targetDist = distance(targetX, targetY, curPosX, curPosY);

  bool isDriving = m_ColliStatus == DriveToTarget;
  bool isNewShortTarget = (if_colli_target_->dest_dist() < cfg_min_long_dist_drive_)
                       && (if_colli_target_->dest_dist() >= cfg_min_drive_dist_);

  //  bool  stop_on_target =  if_colli_target_->StopOnTarget();

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

  /* Decide which status we need to switch to.
   * We keep the current status, unless one of the following happens:
   * 1) The target is far away
   *    -> we drive to the target via a pre-position
   * 2) The target was initially far away, now exceeds a minimum-distance so that it can drive
   *    straight to the target without a pre-position.
   *    -> we drive to that target directly
   * 3) The robot is considered to be "at target position" and exceeds a minimum angle to the target
   *    orientation
   *    -> we rotate towards the target rotation.
   * 4) The new target is in a short distance (not as far as in case (2) yet)
   *   -> we drive to that target directly
   *
   * Special cases are also considered:
   * 1') We reached the target position and are already adjusting orientation
   *    -> ONLY rotate at this point, and finish when rotation is over (avoid driving again)
   * 2') We are driving straight to the target, but are not close enough yet to it to say "stop".
   *    -> continue drivint straight to the target, even if the distance would not trigger (2) from above
   *       anymore.
   *
   * 5) Other than that, we have nothing to do :)
   */

  if( m_ColliStatus == OrientAtTarget ) { // case (1')
    if ( !orient || ( fabs( normalize_mirror_rad(curPosO - targetO) ) < cfg_min_rot_dist_ ) )
      m_ColliStatus = NothingToDo; // we don't need to rotate anymore; case
    return;
  }

  if( orient && ( targetDist >= cfg_min_long_dist_prepos_ ) ) { // case (1)
    // We approach a point prior to the target, to adjust the orientation a little
    float pre_pos_dist = cfg_target_pre_pos_;
    if ( m_pMotorInstruct->GetUserDesiredTranslation() < 0 )
      pre_pos_dist = -pre_pos_dist;

    m_TargetPointX = targetX - ( pre_pos_dist * cos(targetO) );
    m_TargetPointY = targetY - ( pre_pos_dist * sin(targetO) );

    m_ColliStatus = DriveToOrientPoint;
    return;

  } else if( (targetDist >= cfg_min_long_dist_drive_)                  // case (2)
          || (isDriving && targetDist >= cfg_min_drive_dist_)          // case (2')
          || (isNewShortTarget && targetDist >= cfg_min_drive_dist_) ) { // case (4)
    m_TargetPointX = targetX;
    m_TargetPointY = targetY;
    m_ColliStatus = DriveToTarget;
    return;

  } else if ( orient && ( fabs( normalize_mirror_rad(curPosO - targetO) ) >= cfg_min_rot_dist_ ) ) { // case (3)
    m_ColliStatus = OrientAtTarget;
    return;

  } else {  // case (5)
    m_ColliStatus = NothingToDo;
    return;
  }

  return;
}



/// Calculate all information out of the updated blackboard data
//  m_RoboGridPos, m_LaserGridPos, m_TargetGridPos have to be updated!
//  the targetPointX and targetPointY were calculated in the collis state machine!
void
ColliThread::UpdateOwnModules()
{
  if ( !cfg_obstacle_inc_ ) {
    // do not increase cell size
    m_pLaserOccGrid->setCellWidth( (int)m_OccGridCellWidth );
    m_pLaserOccGrid->setCellHeight( (int)m_OccGridCellHeight );

  } else {
    // set the cell size according to the current speed
    m_pLaserOccGrid->setCellWidth( (int)std::max( (int)m_OccGridCellWidth,
                                                  (int)(5*fabs(m_pMotorInstruct->GetMotorDesiredTranslation())+3) ) );
    m_pLaserOccGrid->setCellHeight((int)std::max( (int)m_OccGridCellHeight,
                                                  (int)(5*fabs(m_pMotorInstruct->GetMotorDesiredTranslation())+3) ) );
  }

  // Calculate discrete position of the laser
  int laserpos_x = (int)(m_pLaserOccGrid->getWidth() / 2);
  int laserpos_y = (int)(m_pLaserOccGrid->getHeight() / 2);

  laserpos_x -= (int)( m_pMotorInstruct->GetMotorDesiredTranslation()*m_pLaserOccGrid->getWidth() / (2*3.0) );
  laserpos_x  = max ( laserpos_x, 10 );
  laserpos_x  = min ( laserpos_x, (int)(m_pLaserOccGrid->getWidth()-10) );

  int robopos_x = laserpos_x + lround( laser_to_base_.x*100 / m_pLaserOccGrid->getCellWidth() );
  int robopos_y = laserpos_y + lround( laser_to_base_.y*100 / m_pLaserOccGrid->getCellHeight() );

  // coordinate transformation for target point
  float aX = m_TargetPointX - m_pMotorInstruct->GetCurrentX();
  float aY = m_TargetPointY - m_pMotorInstruct->GetCurrentY();
  float targetContX = ( aX*cos( m_pMotorInstruct->GetCurrentOri() ) + aY*sin( m_pMotorInstruct->GetCurrentOri() ) );
  float targetContY = (-aX*sin( m_pMotorInstruct->GetCurrentOri() ) + aY*cos( m_pMotorInstruct->GetCurrentOri() ) );

  // calculation, where in the grid the target is, thats relative to the motorpos, so add it ;-)
  int targetGridX = (int)( (targetContX * 100.0) / (float)m_pLaserOccGrid->getCellWidth() );
  int targetGridY = (int)( (targetContY * 100.0) / (float)m_pLaserOccGrid->getCellHeight() );

  targetGridX += robopos_x;
  targetGridY += robopos_y;


  // check the target borders. if its out of the occ grid, put it back in by border checking
  // with linear interpolation
  if (targetGridX >= m_pLaserOccGrid->getWidth()-1) {
    targetGridY = robopos_y + ((robopos_x - (m_pLaserOccGrid->getWidth()-2))/(robopos_x - targetGridX) * (targetGridY - robopos_y));
    targetGridX = m_pLaserOccGrid->getWidth()-2;
  }

  if (targetGridX < 2) {
    targetGridY = robopos_y + ((robopos_x-2)/(robopos_x - targetGridX) * (targetGridY - robopos_y));
    targetGridX = 2;
  }

  if (targetGridY >= m_pLaserOccGrid->getHeight()-1) {
    targetGridX = robopos_x + ((robopos_y - (m_pLaserOccGrid->getHeight()-2))/(robopos_y - targetGridY) * (targetGridX - robopos_x));
    targetGridY = m_pLaserOccGrid->getHeight()-2;
  }

  if (targetGridY < 2) {
    targetGridX = robopos_x + ((robopos_y-2)/(robopos_y - targetGridY) * (targetGridX - robopos_x));
    targetGridY = 2;
  }

  // update the laser
  m_pLaser->UpdateLaser();

  // Robo increasement for robots
  float m_RoboIncrease = 0.0;

  if ( if_colli_target_->security_distance() > 0.0 ) {
    m_RoboIncrease = if_colli_target_->security_distance();
    logger->log_info(name(),"(UpdateOwnModules ): Setting EXTERN Robot secure distance = %f. ATTENTION TO THE ROBOT!!!!",
                            m_RoboIncrease);
  } else {
    if ( cfg_obstacle_inc_ ) {
      //float transinc = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentTranslation()/2.0 )-0.35);
      //float rotinc   = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentRotation()/3.5 )-0.4);
      float transinc = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentTranslation()/2.0 )-0.7);
      float rotinc   = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentRotation()/3.5 )-0.7);

      m_RoboIncrease = max( transinc, rotinc );
      m_RoboIncrease = min( m_MaximumRoboIncrease, m_RoboIncrease );
    } else {
      m_RoboIncrease = 0.0;
    }
  }

  float xdiff = m_pMotorInstruct->GetCurrentX() - m_OldX;
  m_OldX = m_pMotorInstruct->GetCurrentX();
  float ydiff = m_pMotorInstruct->GetCurrentY() - m_OldY;
  m_OldY = m_pMotorInstruct->GetCurrentY();
  float oridiff = normalize_mirror_rad( m_pMotorInstruct->GetCurrentOri() - m_OldOri );
  m_OldOri = m_pMotorInstruct->GetCurrentOri();

  float relxdiff =  xdiff *  cos( m_pMotorInstruct->GetCurrentOri() ) +
                    ydiff *  sin( m_pMotorInstruct->GetCurrentOri() );
  float relydiff =  xdiff * -sin( m_pMotorInstruct->GetCurrentOri() ) +
                    ydiff *  cos( m_pMotorInstruct->GetCurrentOri() );

  // update the occgrid...
  m_pLaserOccGrid->UpdateOccGrid( laserpos_x, laserpos_y, m_RoboIncrease,
                                  m_pMotorInstruct->GetMotorDesiredTranslation(),
                                  relxdiff, relydiff, oridiff );

  // update the positions
  m_LaserGridPos.x = laserpos_x;
  m_LaserGridPos.y = laserpos_y;
  m_RoboGridPos.x = robopos_x;
  m_RoboGridPos.y = robopos_y;
  m_TargetGridPos.x = targetGridX;
  m_TargetGridPos.y = targetGridY;
}




/// Check if we want to escape an obstacle
bool
ColliThread::CheckEscape()
{
  return ((float)m_pLaserOccGrid->getProb(m_RoboGridPos.x,m_RoboGridPos.y) == _COLLI_CELL_OCCUPIED_ );
}
