
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
#include <tf/time_cache.h>

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
  frequency_      = config->get_int((cfg_prefix + "frequency").c_str());
  max_robo_inc_ = config->get_float((cfg_prefix + "max_robo_increase").c_str());
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

  cfg_write_spam_debug_    = config->get_bool((cfg_prefix + "write_spam_debug").c_str());

  cfg_emergency_stop_enabled_        = config->get_bool((cfg_prefix + "emergency_stopping/enabled").c_str());
  cfg_emergency_threshold_distance_  = config->get_float((cfg_prefix + "emergency_stopping/threshold_distance").c_str());
  cfg_emergency_threshold_velocity_  = config->get_float((cfg_prefix + "emergency_stopping/threshold_velocity").c_str());
  cfg_emergency_velocity_max_        = config->get_float((cfg_prefix + "emergency_stopping/max_vel").c_str());

  std::string escape_mode = config->get_string((cfg_prefix + "drive_mode/default_escape").c_str());
  if ( escape_mode.compare("potential_field") == 0 ) {
    cfg_escape_mode_ = fawkes::colli_escape_mode_t::potential_field;
  } else if ( escape_mode.compare("basic") == 0 ) {
    cfg_escape_mode_ = fawkes::colli_escape_mode_t::basic;
  } else {
    cfg_escape_mode_ = fawkes::colli_escape_mode_t::basic;
    throw fawkes::Exception("Default escape drive_mode is unknown");
  }

  std::string motor_instruct_mode = config->get_string((cfg_prefix + "motor_instruct/mode").c_str());
  if ( motor_instruct_mode.compare("linear") == 0 ) {
    cfg_motor_instruct_mode_ = fawkes::colli_motor_instruct_mode_t::linear;
  } else if ( motor_instruct_mode.compare("quadratic") == 0 ) {
    cfg_motor_instruct_mode_ = fawkes::colli_motor_instruct_mode_t::quadratic;
  } else {
    cfg_motor_instruct_mode_ = fawkes::colli_motor_instruct_mode_t::linear;
    throw fawkes::Exception("Motor instruct mode is unknown, use linear");
  }

  cfg_prefix += "occ_grid/";
  occ_grid_width_        = config->get_float((cfg_prefix + "width").c_str());
  occ_grid_height_       = config->get_float((cfg_prefix + "height").c_str());
  occ_grid_cell_width_    = config->get_int((cfg_prefix + "cell_width").c_str());
  occ_grid_cell_height_   = config->get_int((cfg_prefix + "cell_height").c_str());

  srand( time( NULL ) );
  distance_to_next_target_ = 1000.f;

  logger->log_debug(name(), "(init): Entering initialization ..." );

  open_interfaces();
  try {
    initialize_modules();
  } catch(Exception &e) {
    blackboard->close( if_colli_target_ );
    blackboard->close( if_laser_ );
    blackboard->close( if_motor_ );
    throw;
  }

#ifdef HAVE_VISUAL_DEBUGGING
  vis_thread_->setup(occ_grid_, search_);
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
    occ_grid_->set_base_offset(laser_to_base_.x, laser_to_base_.y);
  } catch(Exception &e) {
    if (fawkes::runtime::uptime() >= tf_listener->get_cache_time()) {
      logger->log_warn(name(), "Unable to transform %s to %s.\n%s",
           cfg_frame_base_.c_str(), cfg_frame_laser_.c_str(), e.what() );
    }
  }

  // setup timer for colli-frequency
  timer_ = new TimeWait(clock, 1e6 / frequency_);

  proposed_.x = proposed_.y = proposed_.rot = 0.f;

  target_new_ = false;
  escape_count_ = 0;

  logger->log_debug(name(), "(init): Initialization done.");
}


void
ColliThread::finalize()
{
  logger->log_debug(name(), "(finalize): Entering destructing ...");

  delete timer_;

  // delete own modules
  delete select_drive_mode_;
  delete search_;
  delete occ_grid_;
  delete motor_instruct_;

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
      occ_grid_->set_base_offset(laser_to_base_.x, laser_to_base_.y);
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
  if( !interface_data_valid() ) {
    escape_count_ = 0;
    abort = true;

/*
    // THIS IF FOR CHALLENGE ONLY!!!
  } else if( if_colli_target_->drive_mode() == NavigatorInterface::OVERRIDE ) {
    logger->log_debug(name(), "BEING OVERRIDDEN!");
    colli_data_.final = false;
    escape_count_ = 0;
    abort = true;
*/

  } else if( if_colli_target_->drive_mode() == NavigatorInterface::MovingNotAllowed ) {
    //logger->log_debug(name(), "Moving is not allowed!");
    escape_count_ = 0;
    abort = true;

    // Do not drive if there is no new target
  } else if( if_colli_target_->is_final() ) {
    //logger->log_debug(name(), "No new target for colli...ABORT");
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
        motor_instruct_->stop();
      } else {
        // movement has stopped, we are "final" now
        colli_data_.final = true;
        // send one final stop, just to make sure we really stop
        motor_instruct_->stop();
      }
    }

#ifdef HAVE_VISUAL_DEBUGGING
    if( cfg_visualize_idle_ ) {
      update_modules();
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
// update the state machine                                                     //
//                                                                              //
// If we are in stop state                                                      //
//    Do stop                                                                   //
// Else if we are in orient state                                               //
//    Do orient                                                                 //
// else if we are in a drive state                                              //
//    update the grid                                                           //
//    If we are to close to an obstacle                                         //
//       Escape the obstacle                                                    //
//       Get Motor settings for escaping                                        //
//       Set Motor parameters for escaping                                      //
//    else                                                                      //
//       search for a way                                                       //
//       if we found a way,                                                     //
//          Translate the way in motor things                                   //
//          Set Motor parameters for driving                                    //
//       else                                                                   //
//          do nothing, because this is an error!                               //
//          Set Motor parameters for stopping                                   //
//                                                                              //
// Translate and Realize the motor commands                                     //
// update the BB Things                                                         //
//                                                                              //
// ============================================================================ //
// ============================================================================ //
//
void
ColliThread::colli_execute_()
{
  // to be on the sure side of life
  proposed_.x = proposed_.y = proposed_.rot = 0.f;

  // update state machine
  update_colli_state();

  // nothing is to do
  if (colli_state_ == NothingToDo) {
    occ_grid_->reset_old();
    if( abs(if_motor_->vx()) <= 0.01f
     && abs(if_motor_->vy()) <= 0.01f
     && abs(if_motor_->omega()) <= 0.01f ) {
      // we have stopped, can consider the colli final now
      //logger->log_debug(name(), "L, consider colli final now");
      colli_data_.final = true;
    }

    occ_grid_->reset_old();
    escape_count_ = 0;

#ifdef HAVE_VISUAL_DEBUGGING
    if( cfg_visualize_idle_ )
      update_modules();
#endif

  } else {
    // perform the update of the grid.
    update_modules();
    colli_data_.final = false;

    // Check, if one of our positions (robo-, laser-gridpos is not valid) => Danger!
    if( check_escape() == true || escape_count_ > 0 ) {
      if( if_motor_->des_vx() == 0.f
       && if_motor_->des_vy() == 0.f
       && if_motor_->des_omega() == 0.f ) {
        occ_grid_->reset_old();
      }

      // ueber denken und testen

      if( if_colli_target_->is_escaping_enabled() ) {
        // SJTODO: ERST wenn ich gestoppt habe, escape mode anwerfen!!!
        if (escape_count_ > 0)
          escape_count_--;
        else {
          int rnd = (int)((rand())/(float)(RAND_MAX)) * 10; // + 5;
          escape_count_ = rnd;
          if (cfg_write_spam_debug_) {
            logger->log_debug(name(), "Escape: new round with %i", rnd);
          }
        }

        if (cfg_write_spam_debug_) {
          logger->log_debug(name(), "Escape mode, escaping!");
        }
        select_drive_mode_->set_local_target( local_target_.x, local_target_.y );
        if ( cfg_escape_mode_ == fawkes::colli_escape_mode_t::potential_field ) {
          select_drive_mode_->set_grid_information(occ_grid_, robo_grid_pos_.x, robo_grid_pos_.y);
        } else {
          if_laser_->read();

          std::vector<polar_coord_2d_t> laser_points;
          laser_points.reserve(if_laser_->maxlenof_distances());

          float angle_inc = 2.f * M_PI / if_laser_->maxlenof_distances();

          polar_coord_2d_t laser_point;
          for ( unsigned int i = 0; i < if_laser_->maxlenof_distances(); ++i ) {
            laser_point.r = if_laser_->distances(i);
            laser_point.phi = angle_inc * i;
            laser_points.push_back(laser_point);
          }
          select_drive_mode_->set_laser_data(laser_points);
        }
        select_drive_mode_->update( true );  // <-- this calls the ESCAPE mode!
        proposed_.x = select_drive_mode_->get_proposed_trans_x();
        proposed_.y = select_drive_mode_->get_proposed_trans_y();
        proposed_.rot    = select_drive_mode_->get_proposed_rot();

      } else {
        logger->log_warn(name(), "Escape mode, but not allowed!");
        proposed_.x = proposed_.y = proposed_.rot = 0.f;
        escape_count_ = 0;
      }

    } else {
      // only orienting to do and moving possible

      if (colli_state_ == OrientAtTarget) {
        proposed_.x  = 0.f;
        proposed_.y  = 0.f;
        // turn faster if angle-diff is high
        //proposed_.rot    = 1.5*normalize_mirror_rad( if_colli_target_->GetTargetOri() -
        proposed_.rot    = 1.f*normalize_mirror_rad( if_colli_target_->dest_ori() -
                                                     if_motor_->odometry_orientation() );
        // need to consider minimum rotation velocity
        if ( proposed_.rot > 0.f )
          proposed_.rot = std::min( if_colli_target_->max_rotation(), std::max( cfg_min_rot_, proposed_.rot));
        else
          proposed_.rot = std::max(-if_colli_target_->max_rotation(), std::min(-cfg_min_rot_, proposed_.rot));

        occ_grid_->reset_old();

      } else {
        // search for a path
        search_->update( robo_grid_pos_.x, robo_grid_pos_.y,
                        (int)target_grid_pos_.x, (int)target_grid_pos_.y );
        if ( search_->updated_successful() ) {
          // path exists
          local_grid_target_ = search_->get_local_target();
          local_grid_trajec_ = search_->get_local_trajec();

          // coordinate transformation from grid coordinates to relative robot coordinates
          local_target_.x = (local_grid_target_.x - robo_grid_pos_.x)*occ_grid_->get_cell_width()/100.f;
          local_target_.y = (local_grid_target_.y - robo_grid_pos_.y)*occ_grid_->get_cell_height()/100.f;

          local_trajec_.x = (local_grid_trajec_.x - robo_grid_pos_.x)*occ_grid_->get_cell_width()/100.f;
          local_trajec_.y = (local_grid_trajec_.y - robo_grid_pos_.y)*occ_grid_->get_cell_height()/100.f;

          // call appopriate drive mode
          select_drive_mode_->set_local_target( local_target_.x, local_target_.y );
          select_drive_mode_->set_local_trajec( local_trajec_.x, local_trajec_.y );
          select_drive_mode_->update();
          proposed_.x   = select_drive_mode_->get_proposed_trans_x();
          proposed_.y   = select_drive_mode_->get_proposed_trans_y();
          proposed_.rot = select_drive_mode_->get_proposed_rot();

        } else {
          // stop
          // logger->log_warn(name(), "Drive Mode: update not successful ---> stopping!");
          local_target_.x = local_target_.y = 0.f;
          local_trajec_.x = local_trajec_.y = 0.f;
          proposed_.x = proposed_.y = proposed_.rot = 0.f;
          occ_grid_->reset_old();
        }

        colli_data_.local_target = local_target_; // waypoints
        colli_data_.local_trajec = local_trajec_; // collision-points
      }
    }


  }

  if (cfg_write_spam_debug_) {
    logger->log_debug(name(), "I want to realize %f , %f , %f", proposed_.x, proposed_.y, proposed_.rot);
  }

  // check if occ-grid has been updated successfully
  if( distance_to_next_target_ == 0.f ) {
    logger->log_error(name(), "Cccupancy-grid update failed! Stop immediately");
    proposed_.x = proposed_.y = proposed_.rot = 0.f;
    motor_instruct_->stop();

  } else if( // check if emergency stop is needed
      cfg_emergency_stop_enabled_
      &&  distance_to_next_target_ < cfg_emergency_threshold_distance_
      &&  if_motor_->vx() > cfg_emergency_threshold_velocity_ ) {
    float max_v = cfg_emergency_velocity_max_;

    float part_x = 0.f;
    float part_y = 0.f;
    if ( ! (proposed_.x == 0.f && proposed_.y == 0.f) ) {
      part_x = proposed_.x / ( ( fabs(proposed_.x) + fabs(proposed_.y) ) );
      part_y = proposed_.y / ( ( fabs(proposed_.x) + fabs(proposed_.y) ) );
    }

    proposed_.x = part_x * max_v;
    proposed_.y = part_y * max_v;

    logger->log_error(name(), "Emergency slow down: %f , %f , %f", proposed_.x, proposed_.y, proposed_.rot);

    emergency_motor_instruct_->drive( proposed_.x, proposed_.y, proposed_.rot );

  } else {
    // Realize trans-rot proposal with realization module
    motor_instruct_->drive( proposed_.x, proposed_.y, proposed_.rot );
  }
}


/* **************************************************************************** */
/*                       Initialization                                         */
/* **************************************************************************** */
/// Register all BB-Interfaces at the Blackboard.
void
ColliThread::open_interfaces()
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
ColliThread::initialize_modules()
{
  colli_data_.final = true;

  occ_grid_ = new LaserOccupancyGrid( if_laser_, logger, config, tf_listener);

  // set the cell width and heigth to 5 cm and the grid size to 7.5 m x 7.5 m.
  // this are 750/5 x 750/5 grid cells -> (750x750)/5 = 22500 grid cells
  occ_grid_->set_cell_width(  occ_grid_cell_width_ );
  occ_grid_->set_width(  (int)((occ_grid_width_*100)/occ_grid_->get_cell_width()) );
  occ_grid_->set_cell_height( occ_grid_cell_height_ );
  occ_grid_->set_height( (int)((occ_grid_height_*100)/occ_grid_->get_cell_height()) );

  try {
    // THIRD(!): the search component (it uses the occ grid (without the laser)
    search_ = new Search( occ_grid_, logger, config );
  } catch(Exception &e) {
    logger->log_error(name(), "Could not created new search (%s)", e.what_no_backtrace());
    delete occ_grid_;
    throw;
  }

  try {
    // BEFORE DRIVE MODE: the motorinstruction set
    if ( cfg_motor_instruct_mode_ == fawkes::colli_motor_instruct_mode_t::linear ) {
      motor_instruct_ = (BaseMotorInstruct *)new LinearMotorInstruct( if_motor_,
                                                                      frequency_,
                                                                      logger,
                                                                      config );
    } else if ( cfg_motor_instruct_mode_ == fawkes::colli_motor_instruct_mode_t::quadratic ) {
      motor_instruct_ = (BaseMotorInstruct *)new QuadraticMotorInstruct( if_motor_,
                                                                         frequency_,
                                                                         logger,
                                                                         config );
    } else {
      logger->log_error(name(), "Motor instruct not implemented, use linear");
      motor_instruct_ = (BaseMotorInstruct *)new LinearMotorInstruct( if_motor_,
                                                                      frequency_,
                                                                      logger,
                                                                      config );
    }
  } catch(Exception &e) {
    logger->log_error(name(), "Could not create MotorInstruct (%s", e.what_no_backtrace());
    delete occ_grid_;
    delete search_;
    throw;
  }

  try {
    emergency_motor_instruct_ = (BaseMotorInstruct *)new EmergencyMotorInstruct( if_motor_,
                                                                                 frequency_,
                                                                                 logger,
                                                                                 config );
  } catch(Exception &e) {
    logger->log_error(name(), "Could not create EmergencyMotorInstruct (%s", e.what_no_backtrace());
    delete occ_grid_;
    delete search_;
    delete motor_instruct_;
    throw;
  }

  try {
    // AFTER MOTOR INSTRUCT: the motor propose values object
    select_drive_mode_ = new SelectDriveMode( if_motor_, if_colli_target_, logger, config, cfg_escape_mode_ );
  } catch(Exception &e) {
    logger->log_error(name(), "Could not create SelectDriveMode (%s", e.what_no_backtrace());
    delete occ_grid_;
    delete search_;
    delete motor_instruct_;
    delete emergency_motor_instruct_;
    throw;
  }

  // Initialization of colli state machine:
  // Currently nothing is to accomplish
  colli_state_  = NothingToDo;
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
ColliThread::interface_data_valid()
{
  Time now(clock);

  /* check if we have fresh data to fetch. An error has occured if
   * a) laser or motor interface have no writer
   * b) there is no new laser data for a while, or
   * c) there is no motor data for a while and colli is currently moving
   * d) transforms have not been updated in a while
   */
  if( !if_laser_->has_writer() || !if_motor_->has_writer() ) {
    logger->log_warn(name(), "Laser or Motor dead, no writing instance for interfaces!!!");
    return false;
  } else if (if_laser_->timestamp()->is_zero()) {
	  logger->log_debug(name(), "No laser data");
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
    // check if transforms are up to date
    tf::TimeCacheInterfacePtr cache = tf_listener->get_frame_cache(cfg_frame_laser_);
    if( !cache ) {
	    logger->log_warn(name(), "No TimeCache for transform to laser_frame '%s'", cfg_frame_laser_.c_str());
	    return false;
    }

    tf::TransformStorage temp;
    if( !cache->get_data(Time(0,0), temp)) {
      logger->log_warn(name(), "No data in TimeCache for transform to laser frame '%s'", cfg_frame_laser_.c_str());
      return false;
    }

    fawkes::Time laser_frame_latest(cache->get_latest_timestamp());
    if (! laser_frame_latest.is_zero()) {
	    // not a static transform
	    float diff = (now - laser_frame_latest).in_sec();
	    if( diff > 2.f* cfg_iface_read_timeout_) {
		    logger->log_warn(name(), "Transform to laser frame '%s' is too old (%f > %f)",
		                     cfg_frame_laser_.c_str(), diff, 2.f*cfg_iface_read_timeout_);
		    return false;
	    }
    }

    // everything OK
    return true;
  }
}



void
ColliThread::update_colli_state()
{
  // initialize
  if( target_new_ ) {
    // new target!
    colli_state_ = NothingToDo;
    target_new_ = false;
  }

  float cur_x = if_motor_->odometry_position_x();
  float cur_y = if_motor_->odometry_position_y();
  float cur_ori = normalize_mirror_rad( if_motor_->odometry_orientation() );

  float target_x = if_colli_target_->dest_x();
  float target_y = if_colli_target_->dest_y();
  float target_ori = if_colli_target_->dest_ori();

  bool  orient = ( if_colli_target_->orientation_mode() == fawkes::NavigatorInterface::OrientationMode::OrientAtTarget
                && std::isfinite(if_colli_target_->dest_ori()) );

  float target_dist = distance(target_x, target_y, cur_x, cur_y);

  bool is_driving = colli_state_ == DriveToTarget;
  bool is_new_short_target = (if_colli_target_->dest_dist() < cfg_min_long_dist_drive_)
                          && (if_colli_target_->dest_dist() >= cfg_min_drive_dist_);

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

  if( colli_state_ == OrientAtTarget ) { // case (1')
    if ( !orient || ( fabs( normalize_mirror_rad(cur_ori - target_ori) ) < cfg_min_rot_dist_ ) )
      colli_state_ = NothingToDo; // we don't need to rotate anymore; case
    return;
  }

  if( orient && ( target_dist >= cfg_min_long_dist_prepos_ ) ) { // case (1)
    // We approach a point prior to the target, to adjust the orientation a little
    float pre_pos_dist = cfg_target_pre_pos_;
    if ( if_motor_->des_vx() < 0 )
      pre_pos_dist = -pre_pos_dist;

    target_point_.x = target_x - ( pre_pos_dist * cos(target_ori) );
    target_point_.y = target_y - ( pre_pos_dist * sin(target_ori) );

    colli_state_ = DriveToOrientPoint;
    return;

  } else if( (target_dist >= cfg_min_long_dist_drive_)                  // case (2)
          || (is_driving && target_dist >= cfg_min_drive_dist_)          // case (2')
          || (is_new_short_target && target_dist >= cfg_min_drive_dist_) ) { // case (4)
    target_point_.x = target_x;
    target_point_.y = target_y;
    colli_state_ = DriveToTarget;
    return;

  } else if ( orient && ( fabs( normalize_mirror_rad(cur_ori - target_ori) ) >= cfg_min_rot_dist_ ) ) { // case (3)
    colli_state_ = OrientAtTarget;
    return;

  } else {  // case (5)
    colli_state_ = NothingToDo;
    return;
  }

  return;
}



/// Calculate all information out of the updated blackboard data
//  robo_grid_pos_, laser_grid_pos_, target_grid_pos_ have to be updated!
//  the targetPointX and targetPointY were calculated in the collis state machine!
void
ColliThread::update_modules()
{
  float vx, vy, v;
  vx = if_motor_->des_vx();
  vy = if_motor_->des_vy();
  v  = std::sqrt( vx*vx + vy*vy );

  if ( !cfg_obstacle_inc_ ) {
    // do not increase cell size
    occ_grid_->set_cell_width( (int)occ_grid_cell_width_ );
    occ_grid_->set_cell_height( (int)occ_grid_cell_height_ );

  } else {
    // set the cell size according to the current speed
    occ_grid_->set_cell_width( (int)std::max( (int)occ_grid_cell_width_,
                                                  (int)(5*fabs( v )+3) ) );
    occ_grid_->set_cell_height((int)std::max( (int)occ_grid_cell_height_,
                                                  (int)(5*fabs( v )+3) ) );
  }

  // Calculate discrete position of the laser
  int laserpos_x = (int)(occ_grid_->get_width() / 2);
  int laserpos_y = (int)(occ_grid_->get_height() / 2);

  laserpos_x -= (int)( vx * occ_grid_->get_width() / (2*3.0) );
  laserpos_x  = max ( laserpos_x, 10 );
  laserpos_x  = min ( laserpos_x, (int)(occ_grid_->get_width()-10) );

  int robopos_x = laserpos_x + lround( laser_to_base_.x*100 / occ_grid_->get_cell_width() );
  int robopos_y = laserpos_y + lround( laser_to_base_.y*100 / occ_grid_->get_cell_height() );

  // coordinate transformation for target point
  float a_x = target_point_.x - if_motor_->odometry_position_x();
  float a_y = target_point_.y - if_motor_->odometry_position_y();
  float cur_ori = normalize_mirror_rad( if_motor_->odometry_orientation() );
  float target_cont_x = ( a_x*cos( cur_ori ) + a_y*sin( cur_ori ) );
  float target_cont_y = (-a_x*sin( cur_ori ) + a_y*cos( cur_ori ) );

  // calculation, where in the grid the target is, thats relative to the motorpos, so add it ;-)
  int target_grid_x = (int)( (target_cont_x * 100.f) / (float)occ_grid_->get_cell_width() );
  int target_grid_y = (int)( (target_cont_y * 100.f) / (float)occ_grid_->get_cell_height() );

  target_grid_x += robopos_x;
  target_grid_y += robopos_y;


  // check the target borders. if its out of the occ grid, put it back in by border checking
  // with linear interpolation
  if (target_grid_x >= occ_grid_->get_width()-1) {
    target_grid_y = robopos_y + ((robopos_x - (occ_grid_->get_width()-2))/(robopos_x - target_grid_x) * (target_grid_y - robopos_y));
    target_grid_x = occ_grid_->get_width()-2;
  }

  if (target_grid_x < 2) {
    target_grid_y = robopos_y + ((robopos_x-2)/(robopos_x - target_grid_x) * (target_grid_y - robopos_y));
    target_grid_x = 2;
  }

  if (target_grid_y >= occ_grid_->get_height()-1) {
    target_grid_x = robopos_x + ((robopos_y - (occ_grid_->get_height()-2))/(robopos_y - target_grid_y) * (target_grid_x - robopos_x));
    target_grid_y = occ_grid_->get_height()-2;
  }

  if (target_grid_y < 2) {
    target_grid_x = robopos_x + ((robopos_y-2)/(robopos_y - target_grid_y) * (target_grid_x - robopos_x));
    target_grid_y = 2;
  }

  // Robo increasement for robots
  float robo_inc = 0.f;

  if ( if_colli_target_->security_distance() > 0.f )
    robo_inc = if_colli_target_->security_distance();

  if ( cfg_obstacle_inc_ ) {
    // calculate increasement due to speed
    //float transinc = max(0.0,fabs( motor_instruct_->GetMotorCurrentTranslation()/2.0 )-0.35);
    //float rotinc   = max(0.0,fabs( motor_instruct_->GetMotorCurrentRotation()/3.5 )-0.4);
    float cur_trans = sqrt(if_motor_->vx()*if_motor_->vx() + if_motor_->vy()*if_motor_->vy());
    float transinc = max(0.f,cur_trans/2.f -0.7f);
    float rotinc   = max(0.f,fabs( if_motor_->omega()/3.5f )-0.7f);
    float speedinc = max( transinc, rotinc );

    // increase at least as much as "security distance"!
    robo_inc = max( robo_inc, speedinc);

    // check against increasement limits
    robo_inc = min( max_robo_inc_, robo_inc );
  }

  // update the occgrid...
  distance_to_next_target_ = 1000.f;
  distance_to_next_target_ = occ_grid_->update_occ_grid( laserpos_x, laserpos_y, robo_inc, vx, vy );

  // update the positions
  laser_grid_pos_.x = laserpos_x;
  laser_grid_pos_.y = laserpos_y;
  robo_grid_pos_.x = robopos_x;
  robo_grid_pos_.y = robopos_y;
  target_grid_pos_.x = target_grid_x;
  target_grid_pos_.y = target_grid_y;
}

/// Check if we want to escape an obstacle
bool
ColliThread::check_escape()
{
  static unsigned int cell_cost_occ = occ_grid_->get_cell_costs().occ;
  return ((float)occ_grid_->get_prob(robo_grid_pos_.x,robo_grid_pos_.y) == cell_cost_occ );
}
