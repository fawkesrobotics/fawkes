
/***************************************************************************
 *  roboshape.h - Class containing shape information of robot
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013-2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_UTILS_ROB_ROBOSHAPE_H_
#define __PLUGINS_COLLI_UTILS_ROB_ROBOSHAPE_H_

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class Configuration;

class RoboShape
{
 public:
  RoboShape( const char * cfg_prefix,
             fawkes::Logger* logger,
             fawkes::Configuration* config);
  ~RoboShape();

  bool is_round_robot( );
  bool is_angular_robot( );

  ///\brief Check if the reading is 'in' the robot
  bool is_robot_reading_for_rad( float anglerad, float length );

  ///\brief Check if the reading is 'in' the robot
  bool is_robot_reading_for_deg( float angledeg, float length );

  ///\brief return the length of the robot for a specific angle
  float get_robot_length_for_rad( float anglerad );

  ///\brief return the length of the robot for a specific angle
  float get_robot_length_for_deg( float angledeg );

  ///\brief Returns the radius of the robot if its round.
  float get_radius();

  ///\brief Returns the maximum radius of the robot if its round.
  float get_complete_radius();

  ///\brief Returns the width-x of the angular robot.
  float get_width_x();

  ///\brief Returns the width-y of the angular robot.
  float get_width_y();

  ///\brief Returns the complete x width of the angular robot.
  float get_complete_width_x();

  ///\brief Returns the complete x width of the angular robot.
  float get_complete_width_y();

  ///\brief Returns the laser offset in x direction of the robot.
  float get_laser_offset_x();

  ///\brief Returns the laser offset in y direction of the robot.
  float get_laser_offset_y();

  ///\brief Get angle to the front left corner of the robot
  float get_angle_front_left() const;

  ///\brief Get angle to the front right corner of the robot
  float get_angle_front_right() const;

  ///\brief Get angle to of the rear left corner robot
  float get_angle_back_left() const;

  ///\brief Get angle to of the rear right corner robot
  float get_angle_back_right() const;

  ///\brief Get angle to middle of the left side of the robot
  float get_angle_left() const;

  ///\brief Get angle to middle of the right side of the robot
  float get_angle_right() const;

  ///\brief Get angle to middle of the front side of the robot
  float get_angle_front() const;

  ///\brief Get angle to middle of the back side of the robot
  float get_angle_back() const;


private:

  bool is_round_;    /**< flag if the robot is round */
  bool is_angular_;  /**< flag if the robot is angular */

  // several variables containing information about the robot.
  float radius_, width_x_, width_y_;
  float laser_offset_x_, laser_offset_y_;
  float width_add_front_, width_add_back_, width_add_left_, width_add_right_;
  float robot_to_front_, robot_to_right_, robot_to_back_, robot_to_left_;

  // angles to the "corners" and mid-sections of the complete roboshape
  float ang_front_left_, ang_front_right_, ang_back_left_, ang_back_right_;
  float ang_left_, ang_right_, ang_front_, ang_back_;

  fawkes::Logger* logger_;
};

} // namespace fawkes

#endif
