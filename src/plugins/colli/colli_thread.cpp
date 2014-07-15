
/***************************************************************************
 *  colli_thread.cpp - Fawkes Colli Thread
 *
 *  Created: Sat Jul 13 12:00:00 2013
 *  Copyright  2013-2014  Bahram Maleki-Fard
 *                  2014  Tobias Neumann
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

#include "colli_thread.h"
#ifdef HAVE_VISUAL_DEBUGGING
 #include "visualization_thread.h"
#endif

#include "drive_modes/select_drive_mode.h"
#include "drive_realization/emergency_motor_instruct.h"
#include "drive_realization/linear_motor_instruct.h"
#include "drive_realization/quadratic_motor_instruct.h"
#include "search/og_laser.h"
#include "search/astar_search.h"

#include <core/threading/mutex.h>
#include <baseapp/run.h>
#include <utils/time/wait.h>
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
  : Thread("ColliThread", Thread::OPMODE_CONTINUOUS),
    vis_thread_( 0 )
{
  mutex_ = new Mutex();
}

/** Destructor. */
ColliThread::~ColliThread()
{
  delete mutex_;
}

void
ColliThread::init()
{
  logger->log_debug(name(), "(init): Constructing...");

  std::string cfg_prefix = "/plugins/colli/";
  m_ColliFrequency      = config->get_int((cfg_prefix + "frequency").c_str());
  m_MaximumRoboIncrease = config->get_float((cfg_prefix + "max_robo_increase").c_str());
  cfg_obstacle_inc_     = config->get_bool((cfg_prefix + "obstacle_increasement").c_str());

  cfg_visualize_idle_   = config->get_bool((cfg_prefix + "visualize_idle").c_str());

  cfg_min_rot_              = config->get_float((cfg_prefix + "min_rot").c_str());
  cfg_min_drive_dist_       = config->get_float((cfg_prefix + "min_drive_distance").c_str());
  cfg_min_long_dist_drive_  = config->get_float((cfg_prefix + "min_long_dist_drive").c_str());
  cfg_min_long_dist_prepos_ = config->get_float((cfg_prefix + "min_long_dist_prepos").c_str());
  cfg_min_rot_dist_         = config->get_float((cfg_prefix + "min_rot_distance").c_str());
  cfg_target_pre_pos_       = config->get_float((cfg_prefix + "pre_position_distance").c_str());

  cfg_max_velocity_         = config->get_float((cfg_prefix + "max_velocity").c_str());

  cfg_frame_base_  = config->get_string((cfg_prefix + "frame/base").c_str());
  cfg_frame_laser_ = config->get_string((cfg_prefix + "frame/laser").c_str());

  cfg_iface_motor_        = config->get_string((cfg_prefix + "interface/motor").c_str());
  cfg_iface_laser_        = config->get_string((cfg_prefix + "interface/laser").c_str());
  cfg_iface_colli_        = config->get_string((cfg_prefix + "interface/colli").c_str());
  cfg_iface_read_timeout_ = config->get_float((cfg_prefix + "interface/read_timeout").c_str());

  cfg_write_spam_debug    = config->get_bool((cfg_prefix + "write_spam_debug").c_str());

  cfg_emergency_stop_used           = config->get_bool((cfg_prefix + "emergency_stopping/enabled").c_str());
  cfg_emergency_threshold_distance  = config->get_float((cfg_prefix + "emergency_stopping/threshold_distance").c_str());
  cfg_emergency_threshold_velocity  = config->get_float((cfg_prefix + "emergency_stopping/threshold_velocity").c_str());
  cfg_emergency_velocity_max        = config->get_float((cfg_prefix + "emergency_stopping/max_vel").c_str());

  std::string escape_mode = config->get_string((cfg_prefix + "drive_mode/default_escape").c_str());
  if ( escape_mode.compare("potential_field") == 0 ) {
    cfg_escape_mode = fawkes::colli_escape_mode_t::potential_field;
  } else if ( escape_mode.compare("basic") == 0 ) {
    cfg_escape_mode = fawkes::colli_escape_mode_t::basic;
  } else {
    cfg_escape_mode = fawkes::colli_escape_mode_t::basic;
    throw fawkes::Exception("Default escape drive_mode is unknown");
  }

  std::string motor_instruct_mode = config->get_string((cfg_prefix + "motor_instruct/mode").c_str());
  if ( motor_instruct_mode.compare("linear") == 0 ) {
    cfg_motor_instruct_mode = fawkes::colli_motor_instruct_mode_t::linear;
  } else if ( motor_instruct_mode.compare("quadratic") == 0 ) {
    cfg_motor_instruct_mode = fawkes::colli_motor_instruct_mode_t::quadratic;
  } else {
    cfg_motor_instruct_mode = fawkes::colli_motor_instruct_mode_t::linear;
    throw fawkes::Exception("Motor instruct mode is unknown, use linear");
  }

  cfg_prefix += "occ_grid/";
  m_OccGridWidth        = config->get_float((cfg_prefix + "width").c_str());
  m_OccGridHeight       = config->get_float((cfg_prefix + "height").c_str());
  m_OccGridCellWidth    = config->get_int((cfg_prefix + "cell_width").c_str());
  m_OccGridCellHeight   = config->get_int((cfg_prefix + "cell_height").c_str());

  for ( unsigned int i = 0; i < 10; i++ )
    m_oldAnglesToTarget.push_back( 0.0 );

  srand( time( NULL ) );
  distance_to_next_target_ = 1000;

  logger->log_debug(name(), "(init): Entering initialization ..." );

  RegisterAtBlackboard();
  InitializeModules();

#ifdef HAVE_VISUAL_DEBUGGING
  vis_thread_->setup(m_pLaserOccGrid, m_pSearch);
#endif

  // get distance from laser to robot base
  laser_to_base_valid_ = false;
  tf::Stamped<tf::Point> p_laser;
  tf::Stamped<tf::Point> p_base( tf::Point(0,0,0), fawkes::Time(0,0), cfg_frame_base_);
  try {
    tf_listener->transform_point(cfg_frame_laser_, p_base, p_laser);
    laser_to_base_.x = p_laser.x();
    laser_to_base_.y = p_laser.y();
    logger->log_info(name(), "distance from laser to base: x:%f  y:%f",
		     laser_to_base_.x, laser_to_base_.y);
    laser_to_base_valid_ = true;
    m_pLaserOccGrid->set_base_offset(laser_to_base_.x, laser_to_base_.y);
  } catch(Exception &e) {
    if (fawkes::runtime::uptime() >= tf_listener->get_cache_time()) {
      logger->log_warn(name(), "Unable to transform %s to %s.\n%s",
		       cfg_frame_base_.c_str(), cfg_frame_laser_.c_str(), e.what() );
    }
  }

  // setup timer for colli-frequency
  timer_ = new TimeWait(clock, 1e6 / m_ColliFrequency);

  m_ProposedTranslationX  = 0.;
  m_ProposedTranslationY  = 0.;
  m_ProposedRotation      = 0.;

  target_new_ = false;
  escape_count = 0;

  logger->log_debug(name(), "(init): Initialization done.");
}


void
ColliThread::finalize()
{
  logger->log_debug(name(), "(finalize): Entering destructing ...");

  delete timer_;

  // delete own modules
  delete m_pSelectDriveMode;
  delete m_pSearch;
  delete m_pLaserOccGrid;
  delete m_pMotorInstruct;

  // close all registered bb-interfaces
  blackboard->close( if_colli_target_ );
  blackboard->close( if_laser_ );
  blackboard->close( if_motor_ );

  logger->log_debug(name(), "(finalize): Destructing done.");
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

/** Checks if the colli is final.
 * @return True if colli is final, false otherwise.
 */
bool
ColliThread::is_final() const
{
  return colli_data_.final;
}

/** Sends a goto-command, using global coordinates.
 * @param x Global x-coordinate of destination
 * @param y Global y-coordinate of destination
 * @param ori Global orientation of robot at destination
 * @param iface This interface holds the colli-parameters for the new destination
 */
void
ColliThread::colli_goto(float x, float y, float ori, NavigatorInterface* iface)
{
  mutex_->lock();
  interfaces_read();

  colli_goto_(x, y, ori, iface);
}

/** Sends a goto-command, using relative coordinates.
 * @param x Relative x-coordinate of destination
 * @param y Relative y-coordinate of destination
 * @param ori Relative orientation of robot at destination
 * @param iface This interface holds the colli-parameters for the new destination
 */
void
ColliThread::colli_relgoto(float x, float y, float ori, NavigatorInterface* iface)
{
  mutex_->lock();
  interfaces_read();

  float colliCurrentO = if_motor_->odometry_orientation();

  //TODO: use TF instead tranform from base_link to odom
  // coord transformation: relative target -> (global) motor coordinates
  float colliTargetX = if_motor_->odometry_position_x()
                       + x * cos( colliCurrentO )
                       - y * sin( colliCurrentO );
  float colliTargetY = if_motor_->odometry_position_y()
                       + x * sin( colliCurrentO )
                       + y * cos( colliCurrentO );
  float colliTargetO = colliCurrentO + ori;

  this->colli_goto_(colliTargetX, colliTargetY, colliTargetO, iface);
}

/** Sends a stop-command.
 * Colli will stop and discard the previous destinations. */
void
ColliThread::colli_stop()
{
  mutex_->lock();

  if_colli_target_->set_dest_x( if_motor_->odometry_position_x() );
  if_colli_target_->set_dest_y( if_motor_->odometry_position_y() );
  if_colli_target_->set_dest_ori( if_motor_->odometry_orientation() );
  if_colli_target_->set_dest_dist( 0.f );
  if_colli_target_->set_final( true );
  if_colli_target_->write();
  mutex_->unlock();
}

void
ColliThread::loop()
{
  timer_->mark_start();

  // Do not continue if we don't have a valid transform from base to laser yet
  if( !laser_to_base_valid_ ) {
    try {
      tf::Stamped<tf::Point> p_laser;
      tf::Stamped<tf::Point> p_base( tf::Point(0,0,0), fawkes::Time(0,0), cfg_frame_base_);

      tf_listener->transform_point(cfg_frame_laser_, p_base, p_laser);
      laser_to_base_.x = p_laser.x();
      laser_to_base_.y = p_laser.y();
      logger->log_info(name(), "distance from laser to base: x:%f  y:%f",
		       laser_to_base_.x, laser_to_base_.y);
      laser_to_base_valid_ = true;
      m_pLaserOccGrid->set_base_offset(laser_to_base_.x, laser_to_base_.y);
    } catch(Exception &e) {
      if (fawkes::runtime::uptime() >= tf_listener->get_cache_time()) {
	logger->log_warn(name(), "Unable to transform %s to %s.\n%s",
			 cfg_frame_base_.c_str(), cfg_frame_laser_.c_str(),
			 e.what_no_backtrace());
      }
      timer_->wait();
      return;
    }
  }

  mutex_->lock();

  interfaces_read();

  // check if we need to abort for some reason
  bool abort = false;
  if( !interfaces_valid() ) {
    escape_count = 0;
    abort = true;

/*
    // THIS IF FOR CHALLENGE ONLY!!!
  } else if( if_colli_target_->drive_mode() == NavigatorInterface::OVERRIDE ) {
    logger->log_debug(name(), "BEING OVERRIDDEN!");
    colli_data_.final = false;
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
    // check if we need to stop the current colli movement
    if( !colli_data_.final ) {
      //logger->log_debug(name(), "STOPPING");
      // colli is active, but for some reason we need to abort -> STOP colli movement
      if( abs(if_motor_->vx()) > 0.01f
       || abs(if_motor_->vy()) > 0.01f
       || abs(if_motor_->omega()) > 0.01f ) {
        // only stop movement, if we are moving
        m_pMotorInstruct->Drive( 0.0, 0.0, 0.0 );
      } else {
        // movement has stopped, we are "final" now
        colli_data_.final = true;
        // send one final stop, just to make sure we really stop
        m_pMotorInstruct->Drive( 0.0, 0.0, 0.0 );
      }
    }

#ifdef HAVE_VISUAL_DEBUGGING
    if( cfg_visualize_idle_ ) {
      UpdateOwnModules();
      vis_thread_->wakeup();
    }
#endif

  } else {

    // Run Colli
    colli_execute_();

    // Send motor and colli data away.
    if_colli_target_->set_final( colli_data_.final );
    if_colli_target_->write();

    // visualize the new information
#ifdef HAVE_VISUAL_DEBUGGING
    vis_thread_->wakeup();
#endif
  }

  mutex_->unlock();

  timer_->wait();
}

/* **************************************************************************** */
/* **************************************************************************** */
/* ****************** P R I V A T E  -   S T U F F **************************** */
/* **************************************************************************** */
/* **************************************************************************** */
void
ColliThread::colli_goto_(float x, float y, float ori, NavigatorInterface* iface)
{
  if_colli_target_->copy_values(iface);

  if_colli_target_->set_dest_x( x );
  if_colli_target_->set_dest_y( y );
  if_colli_target_->set_dest_ori( ori );

  // x and y are not needed anymore. use them for calculation of target distance
  x -= if_motor_->odometry_position_x();
  y -= if_motor_->odometry_position_y();
  float dist = sqrt(x*x + y*y);
  if_colli_target_->set_dest_dist(dist);

  if_colli_target_->set_final( false );
  if_colli_target_->write();

  colli_data_.final = false;
  target_new_ = true;
  mutex_->unlock();
}

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
ColliThread::colli_execute_()
{
  // to be on the sure side of life
  m_ProposedTranslationX  = 0.;
  m_ProposedTranslationY  = 0.;
  m_ProposedRotation      = 0.;

  // Update state machine
  UpdateColliStateMachine();

  // nothing is to do
  if (m_ColliStatus == NothingToDo) {
    m_pLaserOccGrid->ResetOld();
    m_ProposedTranslationX  = 0.;
    m_ProposedTranslationY  = 0.;
    m_ProposedRotation      = 0.;
    if( abs(if_motor_->vx()) <= 0.01f
     && abs(if_motor_->vy()) <= 0.01f
     && abs(if_motor_->omega()) <= 0.01f ) {
      // we have stopped, can consider the colli final now
      //logger->log_debug(name(), "L, consider colli final now");
      colli_data_.final = true;
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
    colli_data_.final = false;

    // Check, if one of our positions (robo-, laser-gridpos is not valid) => Danger!
    if( CheckEscape() == true || escape_count > 0 ) {
      if( m_pMotorInstruct->GetMotorDesiredTranslationX() == 0.0
       && m_pMotorInstruct->GetMotorDesiredTranslationY() == 0.0
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
          if (cfg_write_spam_debug) {
            logger->log_debug(name(), "Escape: new round with %i", rnd);
          }
        }

        if (cfg_write_spam_debug) {
          logger->log_debug(name(), "Escape mode, escaping!");
        }
        m_pSelectDriveMode->SetLocalTarget( m_LocalTarget.x, m_LocalTarget.y );
        if ( cfg_escape_mode == fawkes::colli_escape_mode_t::potential_field ) {
          m_pSelectDriveMode->setGridInformation(m_pLaserOccGrid, m_RoboGridPos.x, m_RoboGridPos.y);
        } else {
          if_laser_->read();

          std::vector<CEscapeDriveModule::LaserPoint> laser_points;
          laser_points.reserve(if_laser_->maxlenof_distances());

          float angle_inc = 2.0 * M_PI / if_laser_->maxlenof_distances();

          for ( unsigned int i = 0; i < if_laser_->maxlenof_distances(); ++i ) {
            CEscapeDriveModule::LaserPoint point( if_laser_->distances(i), angle_inc * i );
            laser_points.push_back(point);
          }
          m_pSelectDriveMode->setLaserData(laser_points);
        }
        m_pSelectDriveMode->Update( true );  // <-- this calls the ESCAPE mode!
        m_ProposedTranslationX = m_pSelectDriveMode->GetProposedTranslationX();
        m_ProposedTranslationY = m_pSelectDriveMode->GetProposedTranslationY();
        m_ProposedRotation    = m_pSelectDriveMode->GetProposedRotation();

      } else {
        logger->log_warn(name(), "Escape mode, but not allowed!");
        m_ProposedTranslationX  = 0.;
        m_ProposedTranslationY  = 0.;
        m_ProposedRotation      = 0.;
        escape_count = 0;
      }

    } else {
      // only orienting to do and moving possible

      if (m_ColliStatus == OrientAtTarget) {
        m_ProposedTranslationX  = 0.;
        m_ProposedTranslationY  = 0.;
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
          m_ProposedTranslationX  = m_pSelectDriveMode->GetProposedTranslationX();
          m_ProposedTranslationY  = m_pSelectDriveMode->GetProposedTranslationY();
          m_ProposedRotation      = m_pSelectDriveMode->GetProposedRotation();

        } else {
          // stop
          // logger->log_warn(name(), "Drive Mode: Update not successful ---> stopping!");
          m_LocalTarget.x = 0.f;
          m_LocalTarget.y = 0.f;
          m_LocalTrajec.x = 0.f;
          m_LocalTrajec.y = 0.f;
          m_ProposedTranslationX  = 0.;
          m_ProposedTranslationY  = 0.;
          m_ProposedRotation      = 0.;
          m_pLaserOccGrid->ResetOld();
        }

        colli_data_.local_target = m_LocalTarget; // waypoints
        colli_data_.local_trajec = m_LocalTrajec; // collision-points
      }
    }


  }

  if (cfg_write_spam_debug) {
    logger->log_debug(name(), "I want to realize %f , %f , %f", m_ProposedTranslationX, m_ProposedTranslationY, m_ProposedRotation);
  }

  // calculate if emergency stop is needed
  if (    cfg_emergency_stop_used
      &&  distance_to_next_target_ < cfg_emergency_threshold_distance
      &&  m_pMotorInstruct->GetMotorCurrentTranslation() > cfg_emergency_threshold_velocity ) {
    float max_v = cfg_emergency_velocity_max;

    float part_x = 0;
    float part_y = 0;
    if ( ! (m_ProposedTranslationX == 0 && m_ProposedTranslationY == 0) ) {
      part_x = m_ProposedTranslationX / ( ( fabs(m_ProposedTranslationX) + fabs(m_ProposedTranslationY) ) );
      part_y = m_ProposedTranslationY / ( ( fabs(m_ProposedTranslationX) + fabs(m_ProposedTranslationY) ) );
    }

    m_ProposedTranslationX = part_x * max_v;
    m_ProposedTranslationY = part_y * max_v;

    logger->log_error(name(), "Emergency slow down: %f , %f , %f", m_ProposedTranslationX, m_ProposedTranslationY, m_ProposedRotation);

    m_pEmergencyMotorInstruct->Drive( m_ProposedTranslationX, m_ProposedTranslationY, m_ProposedRotation );
  } else {  // else send normal message
    // Realize drive mode proposal with realization module
    m_pMotorInstruct->Drive( m_ProposedTranslationX, m_ProposedTranslationY, m_ProposedRotation );
  }
}


/* **************************************************************************** */
/*                       Initialization                                         */
/* **************************************************************************** */
/// Register all BB-Interfaces at the Blackboard.
void
ColliThread::RegisterAtBlackboard()
{
  if_motor_ = blackboard->open_for_reading<MotorInterface>(cfg_iface_motor_.c_str());
  if_laser_ = blackboard->open_for_reading<Laser360Interface>(cfg_iface_laser_.c_str());
  if_motor_->read();
  if_laser_->read();

  if_colli_target_ = blackboard->open_for_writing<NavigatorInterface>(cfg_iface_colli_.c_str());
  if_colli_target_->set_final( true );
  if_colli_target_->write();
}


/// Initialize all modules used by the Colli
void
ColliThread::InitializeModules()
{
  colli_data_.final = true;

  m_pLaserOccGrid = new CLaserOccupancyGrid( if_laser_, logger, config, tf_listener);

  // set the cell width and heigth to 5 cm and the grid size to 7.5 m x 7.5 m.
  // this are 750/5 x 750/5 grid cells -> (750x750)/5 = 22500 grid cells
  m_pLaserOccGrid->setCellWidth(  m_OccGridCellWidth );
  m_pLaserOccGrid->setWidth(  (int)((m_OccGridWidth*100)/m_pLaserOccGrid->getCellWidth()) );
  m_pLaserOccGrid->setCellHeight( m_OccGridCellHeight );
  m_pLaserOccGrid->setHeight( (int)((m_OccGridHeight*100)/m_pLaserOccGrid->getCellHeight()) );

  // THIRD(!): the search component (it uses the occ grid (without the laser)
  m_pSearch = new CSearch( m_pLaserOccGrid, logger, config );

  // BEFORE DRIVE MODE: the motorinstruction set
  if ( cfg_motor_instruct_mode == fawkes::colli_motor_instruct_mode_t::linear ) {
    m_pMotorInstruct = (CBaseMotorInstruct *)new CLinearMotorInstruct( if_motor_,
                                                                       m_ColliFrequency,
                                                                       logger,
                                                                       config );
  } else if ( cfg_motor_instruct_mode == fawkes::colli_motor_instruct_mode_t::quadratic ) {
    m_pMotorInstruct = (CBaseMotorInstruct *)new CQuadraticMotorInstruct( if_motor_,
                                                                          m_ColliFrequency,
                                                                          logger,
                                                                          config );
  } else {
    logger->log_error(name(), "Motor instruct not implemented, use linear");
    m_pMotorInstruct = (CBaseMotorInstruct *)new CLinearMotorInstruct( if_motor_,
                                                                       m_ColliFrequency,
                                                                       logger,
                                                                       config );
  }

  m_pEmergencyMotorInstruct = (CBaseMotorInstruct *)new CEmergencyMotorInstruct( if_motor_,
                                                                                 m_ColliFrequency,
                                                                                 logger,
                                                                                 config );

  // AFTER MOTOR INSTRUCT: the motor propose values object
  m_pSelectDriveMode = new CSelectDriveMode( m_pMotorInstruct, if_colli_target_, logger, config, cfg_escape_mode );

  // Initialization of colli state machine:
  // Currently nothing is to accomplish
  m_ColliStatus  = NothingToDo;
}




/* **************************************************************************** */
/*                          During Runtime                                      */
/* **************************************************************************** */
/// read interface data from blackboard
void
ColliThread::interfaces_read()
{
  if_laser_->read();
  if_motor_->read();
}


/// Check if the interface data is valid, i.e. not outdated
bool
ColliThread::interfaces_valid()
{
  Time now(clock);

  /* check if we have fresh data to fetch. An error has occured if
   * a) laser or motor interface have no writer
   * b) there is no new laser data for a while, or
   * c) there is no motor data for a while and colli is currently moving
   */
  if( !if_laser_->has_writer() || !if_motor_->has_writer() ) {
    logger->log_warn(name(), "Laser or Motor dead, no writing instance for interfaces!!!");
    return false;

  } else if( (now - if_laser_->timestamp()) > (double)cfg_iface_read_timeout_ ) {
    logger->log_warn(name(), "LaserInterface writer has been inactive for too long (%f > %f)",
                             (now - if_laser_->timestamp()), cfg_iface_read_timeout_);
    return false;

  } else if( !colli_data_.final && (now - if_motor_->timestamp()) > (double)cfg_iface_read_timeout_ ) {
    logger->log_warn(name(), "MotorInterface writer has been inactive for too long (%f > %f)",
                             (now - if_motor_->timestamp()), cfg_iface_read_timeout_);
    return false;

  } else {

    return true;
  }
}



void
ColliThread::UpdateColliStateMachine()
{
  // initialize
  if( target_new_ ) {
    // new target!
    m_ColliStatus = NothingToDo;
    target_new_ = false;
  }

  float curPosX = m_pMotorInstruct->GetCurrentX();
  float curPosY = m_pMotorInstruct->GetCurrentY();
  float curPosO = m_pMotorInstruct->GetCurrentOri();

  float targetX = if_colli_target_->dest_x();
  float targetY = if_colli_target_->dest_y();
  float targetO = if_colli_target_->dest_ori();

  bool  orient = ( if_colli_target_->orientation_mode() == fawkes::NavigatorInterface::OrientationMode::OrientAtTarget
                && std::isfinite(if_colli_target_->dest_ori()) );

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
    if ( m_pMotorInstruct->GetUserDesiredTranslationX() < 0 )
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
  float vx, vy, v;
  vx = m_pMotorInstruct->GetMotorDesiredTranslationX();
  vy = m_pMotorInstruct->GetMotorDesiredTranslationY();

  if ( !cfg_obstacle_inc_ ) {
    // do not increase cell size
    m_pLaserOccGrid->setCellWidth( (int)m_OccGridCellWidth );
    m_pLaserOccGrid->setCellHeight( (int)m_OccGridCellHeight );

  } else {
    // set the cell size according to the current speed
    m_pLaserOccGrid->setCellWidth( (int)std::max( (int)m_OccGridCellWidth,
                                                  (int)(5*fabs( v )+3) ) );
    m_pLaserOccGrid->setCellHeight((int)std::max( (int)m_OccGridCellHeight,
                                                  (int)(5*fabs( v )+3) ) );
  }

  // Calculate discrete position of the laser
  int laserpos_x = (int)(m_pLaserOccGrid->getWidth() / 2);
  int laserpos_y = (int)(m_pLaserOccGrid->getHeight() / 2);

  laserpos_x -= (int)( vx * m_pLaserOccGrid->getWidth() / (2*3.0) );
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

  // Robo increasement for robots
  float m_RoboIncrease = 0.0;

  if ( if_colli_target_->security_distance() > 0.0 )
    m_RoboIncrease = if_colli_target_->security_distance();

  if ( cfg_obstacle_inc_ ) {
    // calculate increasement due to speed
    //float transinc = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentTranslation()/2.0 )-0.35);
    //float rotinc   = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentRotation()/3.5 )-0.4);
    float transinc = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentTranslation()/2.0 )-0.7);
    float rotinc   = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentRotation()/3.5 )-0.7);
    float speedinc = max( transinc, rotinc );

    // increase at least as much as "security distance"!
    m_RoboIncrease = max( m_RoboIncrease, speedinc);

    // check against increasement limits
    m_RoboIncrease = min( m_MaximumRoboIncrease, m_RoboIncrease );
  }

  // update the occgrid...
  distance_to_next_target_ = 1000;
  distance_to_next_target_ = m_pLaserOccGrid->UpdateOccGrid( laserpos_x, laserpos_y, m_RoboIncrease, vx, vy );

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
  static unsigned int cell_cost_occ = m_pLaserOccGrid->get_cell_costs().occ;
  return ((float)m_pLaserOccGrid->getProb(m_RoboGridPos.x,m_RoboGridPos.y) == cell_cost_occ );
}
