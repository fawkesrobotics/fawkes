
/***************************************************************************
 *  roboshape.cpp - Class containing shape information of robot
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

#include "roboshape.h"

#include <logging/logger.h>
#include <config/config.h>
#include <core/exception.h>
#include <utils/math/angle.h>

#include <cmath>
#include <limits>
#include <string>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RoboShape <plugins/colli/utils/rob/roboshape.h>
 * This is a class containing all roboshape information.
 * All methods have been implemented but round robots.
 */

/** Constructor.
 * @param cfg_prefix The prefix of the config node, where the roboshape values are found
 * @param logger Pointer to the fawkes logger
 * @param config Pointer to the fawkes configuration.
 */
RoboShape::RoboShape( const char * cfg_prefix,
                      Logger* logger,
                      Configuration* config)
{
  logger_ = logger;
  std::string cfg = cfg_prefix;

  is_round_ = is_angular_ = false;
  radius_ = width_x_ = width_y_   = std::numeric_limits<float>::infinity();
  laser_offset_x_ = laser_offset_y_  = std::numeric_limits<float>::infinity();
  width_add_front_ = width_add_back_ = std::numeric_limits<float>::infinity();
  width_add_right_ = width_add_left_ = std::numeric_limits<float>::infinity();

  width_add_front_ = config->get_float((cfg + "extension/front").c_str());
  width_add_right_ = config->get_float((cfg + "extension/right").c_str());
  width_add_back_  = config->get_float((cfg + "extension/back").c_str());
  width_add_left_  = config->get_float((cfg + "extension/left").c_str());

  int shape = config->get_int((cfg + "shape").c_str());
  if( shape == 1 ) {
    // ANGULAR
    is_angular_ = true;
    is_round_ = false;
    width_x_        = config->get_float((cfg + "angular/width_x").c_str());
    width_y_        = config->get_float((cfg + "angular/width_y").c_str());
    laser_offset_x_  = config->get_float((cfg + "angular/laser_offset_x_from_back").c_str());
    laser_offset_y_  = config->get_float((cfg + "angular/laser_offset_y_from_left").c_str());

    float laser_to_back  = laser_offset_x_;
    float laser_to_left  = laser_offset_y_;
    float laser_to_right = width_y_ - laser_offset_y_;
    float laser_to_front = width_x_ - laser_offset_x_;

    robot_to_back_  =  laser_to_back  + width_add_back_;
    robot_to_left_  =  laser_to_left  + width_add_left_;
    robot_to_right_ =  laser_to_right + width_add_right_;
    robot_to_front_ =  laser_to_front + width_add_front_;

  // angles from laser to the edges of real robot dimension
  //  (might be more precise than the calculation below. TODO: check this)
  //ang_front_left_  = normalize_mirror_rad( atan2(  laser_to_left,   laser_to_front ) );
  //ang_front_right_ = normalize_mirror_rad( atan2( -laser_to_right,  laser_to_front ) );
  //ang_back_left_   = normalize_mirror_rad( atan2(  laser_to_left,  -laser_to_back ) );
  //ang_back_right_  = normalize_mirror_rad( atan2( -laser_to_right, -laser_to_back ) );
  //ang_left_  = normalize_mirror_rad( atan2(  laser_to_left,  laser_to_front - width_x_/2.f ) );
  //ang_right_ = normalize_mirror_rad( atan2( -laser_to_right, laser_to_front - width_x_/2.f ) );
  //ang_front_ = normalize_mirror_rad( atan2(  laser_to_left - width_y_/2.f,  laser_to_front ) );
  //ang_back_  = normalize_mirror_rad( atan2(  laser_to_left - width_y_/2.f, -laser_to_back ) );

    logger_->log_info("RoboShape", "Shape is angular!");

  } else if ( shape == 2 ) {
    // ROUND
    is_angular_ = false;
    is_round_ = true;
    radius_        = config->get_float((cfg + "round/radius").c_str());
    laser_offset_x_  = config->get_float((cfg + "round/laser_offset_x_from_middle").c_str());
    laser_offset_y_  = config->get_float((cfg + "round/laser_offset_y_from_middle").c_str());

    robot_to_back_  = radius_ + laser_offset_x_ + width_add_back_;
    robot_to_front_ = radius_ - laser_offset_x_ + width_add_front_;
    robot_to_left_  = radius_ - laser_offset_y_ + width_add_left_;
    robot_to_right_ = radius_ + laser_offset_y_ + width_add_right_;

    logger_->log_info("RoboShape", "Shape is round!");

  } else {
    // WRONG FORMAT!!!
    throw fawkes::Exception("RoboShape: Loading RoboShape from ConfigFile failed! Invalid config value for roboshape");
  }

  logger_->log_info("RoboShape", "|#-->  (m)  is to front: %f", robot_to_front_);
  logger_->log_info("RoboShape", "|#-->  (m)  is to right: %f", robot_to_right_);
  logger_->log_info("RoboShape", "|#-->  (m)  is to left : %f", robot_to_left_);
  logger_->log_info("RoboShape", "+#-->  (m)  is to back : %f", robot_to_back_);

  // angles from laser to edges of the robot extension
  ang_front_left_  = normalize_mirror_rad( atan2(  robot_to_left_,   robot_to_front_ ) );
  ang_front_right_ = normalize_mirror_rad( atan2( -robot_to_right_,  robot_to_front_ ) );
  ang_back_left_   = normalize_mirror_rad( atan2(  robot_to_left_,  -robot_to_back_ ) );
  ang_back_right_  = normalize_mirror_rad( atan2( -robot_to_right_, -robot_to_back_ ) );
  ang_left_  = normalize_mirror_rad( atan2(  robot_to_left_,  robot_to_front_ - width_x_/2.f ) );
  ang_right_ = normalize_mirror_rad( atan2( -robot_to_right_, robot_to_front_ - width_x_/2.f ) );
  ang_front_ = normalize_mirror_rad( atan2(  robot_to_left_ - width_y_/2.f,  robot_to_front_ ) );
  ang_back_  = normalize_mirror_rad( atan2(  robot_to_left_ - width_y_/2.f, -robot_to_back_ ) );
}


/** Desctructor. */
RoboShape::~RoboShape()
{
}

/** Returns if the robot is round.
 * @return bool indicating if the robot is round.
 */
bool
RoboShape::is_round_robot()
{
  return is_round_;
}

/** Returns if the robot is angular.
 * @return bool indicating if the robot is angular.
 */
bool
RoboShape::is_angular_robot()
{
  return is_angular_;
}

/** Returns, if a reading length is _in_ the robot.
 * @param anglerad is float containing the angle of the reading in radians.
 * @param length containing the length of the reading.
 * @return if the reading is in the robot.
 */
bool
RoboShape::is_robot_reading_for_rad( float anglerad, float length )
{
  return (length < get_robot_length_for_rad( anglerad ));
}

/** Returns, if a reading length is _in_ the robot.
 * @param angledeg is float containing the angle of the reading in degree.
 * @param length containing the length of the reading.
 * @return if the reading is in the robot.
 */
bool
RoboShape::is_robot_reading_for_deg( float angledeg, float length )
{
  return is_robot_reading_for_rad( deg2rad( angledeg ), length );
}

/** Get angle to the front left corner of the robot
 * @return angle in radians
 */
float
RoboShape::get_angle_front_left() const
{
  return ang_front_left_;
}

/** Get angle to the front right corner of the robot
 * @return angle in radians
 */
float
RoboShape::get_angle_front_right() const
{
  return ang_front_right_;
}

/** Get angle to the rear left corner of the robot
 * @return angle in radians
 */
float
RoboShape::get_angle_back_left() const
{
  return ang_back_left_;
}

/** Get angle to the rear right corner of the robot
 * @return angle in radians
 */
float
RoboShape::get_angle_back_right() const
{
  return ang_back_right_;
}

/** Get angle to middle of the left side of the robot
 * @return angle in radians
 */
float
RoboShape::get_angle_left() const
{
  return ang_left_;
}

/** Get angle to middle of the right side of the robot
 * @return angle in radians
 */
float
RoboShape::get_angle_right() const
{
  return ang_right_;
}

/** Get angle to middle of the front side of the robot
 * @return angle in radians
 */
float
RoboShape::get_angle_front() const
{
  return ang_front_;
}

/** Get angle to middle of the rear side of the robot
 * @return angle in radians
 */
float
RoboShape::get_angle_back() const
{
  return ang_back_;
}

/** Returns the robots length for a specific angle.
 * @param anglerad is the angle in radians.
 * @return the length in this direction.
 */
float
RoboShape::get_robot_length_for_rad( float anglerad )
{
  anglerad = normalize_mirror_rad( anglerad );

  if( is_round_robot() ) {
    /* use quadratic equation to get intersection point of ray to circle.
     * The ray origins at the laser with angle "anglerad" and is a unit_vector.
     * Consider robot-center as (0,0), we have an equation of:
     *    length(v_laser + k*ray) = radius + expansion
     * with v_laser = vector(laser_offset_x_, laser_offset_y_).
     * "k" is the length from the laser to the robot edge at angle "anglerad".
     *
     * Transform that equation, i.e. resolve "length(..)" and you get a
     * quadratic equation of the kind "ax^2 + 2bx + c = 0".
     */
    float ray_x = cos(anglerad); // unit vector!
    float ray_y = sin(anglerad);

    float a = ray_x*ray_x + ray_y*ray_y;
    float b = ray_x*laser_offset_x_ + ray_y*laser_offset_y_;
    static float c = laser_offset_x_*laser_offset_x_ + laser_offset_y_*laser_offset_y_ - get_complete_radius()*get_complete_radius();

    return ( -b + sqrt(b*b - a*c) ) / a;

  } else if( is_angular_robot() ) {
    /* check all the quadrants in which the target angles lies. The quadrants are spanned
     * by the angles from the center of the robot to its 4 corners. Use "cos(a) = adjacent / hypothenuse",
     * we are looking for the length of the hypothenuse here.
     */
    if( anglerad >= ang_back_left_ || anglerad < ang_back_right_ ) {
      // bottom quadrant; fabs(anglerad) > M_PI_2
      return robot_to_back_  / cos( M_PI - fabs(anglerad) );

    } else if( anglerad < ang_front_right_ ) {
      // right quadrant; -M_PI < anglerad < 0
      return robot_to_right_ / cos( M_PI_2 + anglerad );

    } else if( anglerad < ang_front_left_ ) {
      // top quadrant; -M_PI_2 < anglerad < M_PI_2
      return robot_to_front_ / cos( anglerad );

    } else if( anglerad < ang_back_left_ ) {
      // left quadrant; 0 < anglerad < M_PI
      return robot_to_left_  / cos( M_PI_2 - anglerad);

    } else {
      throw fawkes::Exception("RoboShape: Angles to corners of robot-shape do not cover the whole robot!");
    }

  } else {
    throw fawkes::Exception("RoboShape: Cannot return the robolength for unspecific robot!");
  }
}

/** Returns the robots length for a specific angle.
 * @param angledeg is the angle in degree.
 * @return the length in this direction.
 */
float
RoboShape::get_robot_length_for_deg( float angledeg )
{
  return get_robot_length_for_rad( deg2rad( angledeg ) );
}

/** Returns the radius of the robot if its round.
 * @return radius of the round robot
 */
float
RoboShape::get_radius()
{
  if ( is_round_robot() )
    return radius_;
  else
    logger_->log_error("RoboShape", "The Robot is not round!");

  return 0.f;
}

/** Returns the maximum radius of the robot if its round.
 * @return maximum radius of the round robot
 */
float
RoboShape::get_complete_radius()
{
  if ( is_round_robot() )
    return ( radius_ + std::max( std::max(width_add_front_, width_add_back_),
                                  std::max(width_add_right_, width_add_left_) ) );
  else
    logger_->log_error("RoboShape", "Error: The Robot is not round!");

  return 0.f;
}
/** Returns the width-x of the angular robot.
 * @return only the robot x width.
 */
float
RoboShape::get_width_x()
{
  if ( is_angular_robot() )
    return width_x_;
  else
    logger_->log_error("RoboShape", "The Robot is not angular!");

  return 0.f;
}

/** Returns the width-y of the angular robot.
 * @return only the robot y width.
 */
float
RoboShape::get_width_y()
{
  if ( is_angular_robot() )
    return width_y_;
  else
    logger_->log_error("RoboShape", "The Robot is not angular!");

  return 0.f;
}

/** Returns the complete x width of the angular robot.
 * @return the complete x width.
 */
float
RoboShape::get_complete_width_x()
{
  if ( is_angular_robot() )
    return ( width_x_ + width_add_front_ + width_add_back_ );
  else
    return 2.f*get_complete_radius();

  return 0.f;
}


/** Returns the complete y width of the angular robot.
 * @return the complete y width.
 */
float
RoboShape::get_complete_width_y()
{
  if ( is_angular_robot() )
    return ( width_y_ + width_add_right_ + width_add_left_ );
  else
    return 2.f*get_complete_radius();

  return 0.f;
}

/** Returns the laser offset in x direction of the robot.
 * @return the laser offset in x direction.
 */
float
RoboShape::get_laser_offset_x()
{
  return laser_offset_x_;
}

/** Returns the laser offset in y direction of the robot.
 * @return the laser offset in y direction.
 */
float
RoboShape::get_laser_offset_y()
{
  return laser_offset_y_;
}

} // namespace fawkes
