
/***************************************************************************
 *  colli_thread.h - Fawkes Colli Thread
 *
 *  Created: Wed Oct 16 18:00:00 2013
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

#ifndef __PLUGINS_COLLI_COLLI_THREAD_H_
#define __PLUGINS_COLLI_COLLI_THREAD_H_

#include "common/types.h"

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>

#include <utils/time/clock.h>
#include <utils/math/types.h>
#include <utils/math/angle.h>

namespace fawkes
{
  class Mutex;
  class TimeWait;

  class MotorInterface;
  class Laser360Interface;
  class NavigatorInterface;

  class LaserOccupancyGrid;
  class Search;

  class SelectDriveMode;
  class BaseMotorInstruct;
}

class ColliVisualizationThread;

class ColliThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::TransformAspect,
  public fawkes::BlackBoardAspect
{
 public:
  ColliThread();
  virtual ~ColliThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void set_vis_thread(ColliVisualizationThread* vis_thread);

  bool is_final() const;

  void colli_goto(float x, float y, float ori, fawkes::NavigatorInterface* iface);
  void colli_relgoto(float x, float y, float ori, fawkes::NavigatorInterface* iface);
  void colli_stop();

 private:
  fawkes::Mutex* mutex_;
  fawkes::TimeWait* timer_;

  /* ************************************************************************ */
  /* PRIVATE OBJECT VARIABLES                                                 */
  /* ************************************************************************ */
  /*
   * This is a short list of types that have been transformed from RCSoftX->fawkes:
   *
   * Mopo_Client          ->  MotorInterface      (motor data)
   * Laser_Client         ->  Laser360Interface   (laser data)
   * Colli_Target_Client  ->  NavigatorInterface  (colli target)
   * Colli_Data_Server    ->  colli_data_t        (colli data)
   *
   * Point                ->  cart_coord_2d_t     (point with 2 floats)
   */
  fawkes::MotorInterface*        if_motor_;           // MotorObject
  fawkes::Laser360Interface*     if_laser_;           // LaserScannerObject
  fawkes::NavigatorInterface*    if_colli_target_;    // TargetObject
  fawkes::colli_data_t           colli_data_;         // Colli Data Object

  fawkes::LaserOccupancyGrid*    occ_grid_;     // the grid to drive on
  fawkes::Search*                search_;           // our plan module which calculates the info

  fawkes::SelectDriveMode*       select_drive_mode_;  // the drive mode selection module
  fawkes::BaseMotorInstruct*     motor_instruct_;    // the motor instructor module
  fawkes::BaseMotorInstruct*     emergency_motor_instruct_;  // the emergency motor instructor module

  ColliVisualizationThread*      vis_thread_;         // the VisualizationThread

  /* ************************************************************************ */
  /* PRIVATE VARIABLES THAT HAVE TO BE HANDLED ALL OVER THE MODULE            */
  /* ************************************************************************ */
  fawkes::point_t  robo_grid_pos_;           // the robots position in the grid
  fawkes::point_t  laser_grid_pos_;          // the laser its position in the grid ( not equal to robopos!!! )
  fawkes::point_t  target_grid_pos_;         // the targets position in the grid

  fawkes::point_t local_grid_target_;        // the local target in grid
  fawkes::point_t local_grid_trajec_;        // the local trajec in grid

  fawkes::cart_coord_2d_t  local_target_;   // the local target (relative)
  fawkes::cart_coord_2d_t  local_trajec_;   // the local trajec (relative)

  fawkes::colli_trans_rot_t proposed_; // the proposed trans-rot that should be realized in MotorInstruct

  bool cfg_write_spam_debug_;
  bool cfg_emergency_stop_enabled_;    // true if emergency stop is used
  float cfg_emergency_threshold_distance_; // threshold distance if emergency stop triggers
  float cfg_emergency_threshold_velocity_; // threshold velocity if emergency stop triggers
  float cfg_emergency_velocity_max_;       // if emergency stop triggers, this is the max velocity

  fawkes::colli_state_t colli_state_;     // representing current colli status
  bool target_new_;

  fawkes::cart_coord_2d_t target_point_;  // for update

  int escape_count_;                // count escaping behaviour

  // Config file constants that are read at the beginning
  int frequency_;                          // frequency of the colli
  float occ_grid_height_, occ_grid_width_;         // occgrid field sizes
  int occ_grid_cell_height_, occ_grid_cell_width_;   // occgrid cell sizes
  float max_robo_inc_;                   // maximum increasement of the robots size
  bool cfg_obstacle_inc_;                        // indicator if obstacles should be increased or not

  bool  cfg_visualize_idle_;      /**< Defines if visualization should run when robot is idle without a target. */

  float cfg_min_rot_;             /**< The minimum rotation speed. */
  float cfg_min_drive_dist_;      /**< The minimum distance to drive straight to target */
  float cfg_min_long_dist_drive_; /**< The minimum distance to drive straight to target in a long distance */
  float cfg_min_long_dist_prepos_;/**< The minimum distance to drive to a pre-positino of a target in long distance */
  float cfg_min_rot_dist_;        /**< The minimum rotation distance to rotate, when at target */
  float cfg_target_pre_pos_;      /**< Distance to target pre-position (only if colli_state_t == DriveToOrientPoint) */
  fawkes::colli_escape_mode_t         cfg_escape_mode_;
  fawkes::colli_motor_instruct_mode_t cfg_motor_instruct_mode_;

  float cfg_max_velocity_; /**< The maximum allowd velocity */

  std::string cfg_frame_base_;    /**< The frame of the robot's base */
  std::string cfg_frame_laser_;   /**< The frame of the laser */

  std::string cfg_iface_motor_;   /**< The ID of the MotorInterface */
  std::string cfg_iface_laser_;   /**< The ID of the LaserInterface */
  std::string cfg_iface_colli_;   /**< The ID of the NavigatorInterface for colli target*/
  float cfg_iface_read_timeout_;  /**< Maximum age (in seconds) of required data from reading interfaces*/

  fawkes::cart_coord_2d_t laser_to_base_; /**< The distance from laser to base */
  bool laser_to_base_valid_;              /**< Do we have a valid distance from laser to base? */

  float distance_to_next_target_; /**< the distance to the next target in drive direction*/


  /* ************************************************************************ */
  /* PRIVATE METHODS                                                          */
  /* ************************************************************************ */
  /// Run the actual Colli procedure
  void colli_execute_();

  /// Handle an incoming goto command
  void colli_goto_(float x, float y, float ori, fawkes::NavigatorInterface* iface);

  /// Register all BB-Interfaces at the Blackboard.
  void open_interfaces();

  /// Initialize all modules used by the Colli
  void initialize_modules();

  /// update interface values
  void interfaces_read();

  /// Check if the interface data is valid, i.e. not outdated
  bool interface_data_valid();

  /// Check, in what state the colli is, and what to do
  void update_colli_state();

  /// Calculate all information out of the updated blackboard data
  void update_modules();

  /// Check, if we have to do escape mode, or if we have to drive the ordinary way ;-)
  bool check_escape();

};

#endif

