
/***************************************************************************
 *  roboshape.cpp - Class containing shape information of robot
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

#include "roboshape.h"

#include <logging/logger.h>
#include <config/config.h>
#include <core/exception.h>
#include <utils/math/angle.h>

#include <cmath>
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
                      fawkes::Logger* logger,
                      fawkes::Configuration* config)
{
  logger_ = logger;
  std::string cfg = cfg_prefix;

  m_isRound = m_isAngular = false;
  m_radius = m_widthX = m_widthY   = HUGE_VAL;
  m_laserOffsetX = m_laserOffsetY  = HUGE_VAL;
  m_widthAddFront = m_widthAddBack = HUGE_VAL;
  m_widthAddRight = m_widthAddLeft = HUGE_VAL;

  if( (isinf(m_laserOffsetX) )
   && (isinf(m_laserOffsetY) )
   && (isinf(m_radius) )
   && (isinf(m_widthX) )
   && (isinf(m_widthY) )
   && (isinf(m_widthAddLeft) )
   && (isinf(m_widthAddRight) )
   && (isinf(m_widthAddFront) )
   && (isinf(m_widthAddBack) ) ) {
    // go on, everything is fine, cause all are infinity

  } else {
    throw fawkes::Exception("RoboShape: Initializing Infinity-Values failed!");
  }

  m_widthAddFront = config->get_float((cfg + "extension/front").c_str());
  m_widthAddRight = config->get_float((cfg + "extension/right").c_str());
  m_widthAddBack  = config->get_float((cfg + "extension/back").c_str());
  m_widthAddLeft  = config->get_float((cfg + "extension/left").c_str());

  int shape = config->get_int((cfg + "shape").c_str());
  if( shape == 1 ) {
    // ANGULAR
    m_isAngular = true;
    m_isRound = false;
    m_widthX        = config->get_float((cfg + "angular/width_x").c_str());
    m_widthY        = config->get_float((cfg + "angular/width_y").c_str());
    m_laserOffsetX  = config->get_float((cfg + "angular/laser_offset_x_from_back").c_str());
    m_laserOffsetY  = config->get_float((cfg + "angular/laser_offset_y_from_left").c_str());

    float laserToBack  = m_laserOffsetX;
    float laserToLeft  = m_laserOffsetY;
    float laserToRight = m_widthY - m_laserOffsetY;
    float laserToFront = m_widthX - m_laserOffsetX;

    m_robotToBack  =  laserToBack  + m_widthAddBack;
    m_robotToLeft  =  laserToLeft  + m_widthAddLeft;
    m_robotToRight =  laserToRight + m_widthAddRight;
    m_robotToFront =  laserToFront + m_widthAddFront;

  // angles from laser to the edges of real robot dimension
  //  (might be more precise than the calculation below. TODO: check this)
  //m_angFrontLeft  = normalize_mirror_rad( atan2(  laserToLeft,   laserToFront ) );
  //m_angFrontRight = normalize_mirror_rad( atan2( -laserToRight,  laserToFront ) );
  //m_angBackLeft   = normalize_mirror_rad( atan2(  laserToLeft,  -laserToBack ) );
  //m_angBackRight  = normalize_mirror_rad( atan2( -laserToRight, -laserToBack ) );
  //m_angLeft  = normalize_mirror_rad( atan2(  laserToLeft,  laserToFront - m_widthX/2.f ) );
  //m_angRight = normalize_mirror_rad( atan2( -laserToRight, laserToFront - m_widthX/2.f ) );
  //m_angFront = normalize_mirror_rad( atan2(  laserToLeft - m_widthY/2.f,  laserToFront ) );
  //m_angBack  = normalize_mirror_rad( atan2(  laserToLeft - m_widthY/2.f, -laserToBack ) );

    logger_->log_info("RoboShape", "Shape is angular!");

  } else if ( shape == 2 ) {
    // ROUND
    m_isAngular = false;
    m_isRound = true;
    m_radius        = config->get_float((cfg + "round/radius").c_str());
    m_laserOffsetX  = config->get_float((cfg + "round/laser_offset_x_from_middle").c_str());
    m_laserOffsetY  = config->get_float((cfg + "round/laser_offset_y_from_middle").c_str());

    m_robotToBack  = m_radius + m_laserOffsetX + m_widthAddBack;
    m_robotToFront = m_radius - m_laserOffsetX + m_widthAddFront;
    m_robotToLeft  = m_radius - m_laserOffsetY + m_widthAddLeft;
    m_robotToRight = m_radius + m_laserOffsetY + m_widthAddRight;

    logger_->log_info("RoboShape", "Shape is round!");

  } else {
    // WRONG FORMAT!!!
    throw fawkes::Exception("RoboShape: Loading RoboShape from ConfigFile failed! Invalid config value for roboshape");
  }

  logger_->log_info("RoboShape", "|#-->  (m)  is to front: %f", m_robotToFront);
  logger_->log_info("RoboShape", "|#-->  (m)  is to right: %f", m_robotToRight);
  logger_->log_info("RoboShape", "|#-->  (m)  is to left : %f", m_robotToLeft);
  logger_->log_info("RoboShape", "+#-->  (m)  is to back : %f", m_robotToBack);

  // angles from laser to edges of the robot extension
  m_angFrontLeft  = normalize_mirror_rad( atan2(  m_robotToLeft,   m_robotToFront ) );
  m_angFrontRight = normalize_mirror_rad( atan2( -m_robotToRight,  m_robotToFront ) );
  m_angBackLeft   = normalize_mirror_rad( atan2(  m_robotToLeft,  -m_robotToBack ) );
  m_angBackRight  = normalize_mirror_rad( atan2( -m_robotToRight, -m_robotToBack ) );
  m_angLeft  = normalize_mirror_rad( atan2(  m_robotToLeft,  m_robotToFront - m_widthX/2.f ) );
  m_angRight = normalize_mirror_rad( atan2( -m_robotToRight, m_robotToFront - m_widthX/2.f ) );
  m_angFront = normalize_mirror_rad( atan2(  m_robotToLeft - m_widthY/2.f,  m_robotToFront ) );
  m_angBack  = normalize_mirror_rad( atan2(  m_robotToLeft - m_widthY/2.f, -m_robotToBack ) );
}


/** Desctructor. */
RoboShape::~RoboShape()
{
}

/** Returns if the robot is round.
 * @return bool indicating if the robot is round.
 */
bool
RoboShape::IsRoundRobot()
{
  return m_isRound;
}

/** Returns if the robot is angular.
 * @return bool indicating if the robot is angular.
 */
bool
RoboShape::IsAngularRobot()
{
  return m_isAngular;
}

/** Returns, if a reading length is _in_ the robot.
 * @param anglerad is float containing the angle of the reading in radians.
 * @param length containing the length of the reading.
 * @return if the reading is in the robot.
 */
bool
RoboShape::IsRobotReadingforRad( float anglerad, float length )
{
  return (length < GetRobotLengthforRad( anglerad ));
}

/** Returns, if a reading length is _in_ the robot.
 * @param angledeg is float containing the angle of the reading in degree.
 * @param length containing the length of the reading.
 * @return if the reading is in the robot.
 */
bool
RoboShape::IsRobotReadingforDegree( float angledeg, float length )
{
  return IsRobotReadingforRad( deg2rad( angledeg ), length );
}

/** Get angle to the front left corner of the robot
 * @return angle in radians
 */
float
RoboShape::GetAngleFrontLeft() const
{
  return m_angFrontLeft;
}

/** Get angle to the front right corner of the robot
 * @return angle in radians
 */
float
RoboShape::GetAngleFrontRight() const
{
  return m_angFrontRight;
}

/** Get angle to the rear left corner of the robot
 * @return angle in radians
 */
float
RoboShape::GetAngleBackLeft() const
{
  return m_angBackLeft;
}

/** Get angle to the rear right corner of the robot
 * @return angle in radians
 */
float
RoboShape::GetAngleBackRight() const
{
  return m_angBackRight;
}

/** Get angle to middle of the left side of the robot
 * @return angle in radians
 */
float
RoboShape::GetAngleLeft() const
{
  return m_angLeft;
}

/** Get angle to middle of the right side of the robot
 * @return angle in radians
 */
float
RoboShape::GetAngleRight() const
{
  return m_angRight;
}

/** Get angle to middle of the front side of the robot
 * @return angle in radians
 */
float
RoboShape::GetAngleFront() const
{
  return m_angFront;
}

/** Get angle to middle of the rear side of the robot
 * @return angle in radians
 */
float
RoboShape::GetAngleBack() const
{
  return m_angBack;
}

/** Returns the robots length for a specific angle.
 * @param anglerad is the angle in radians.
 * @return the length in this direction.
 */
float
RoboShape::GetRobotLengthforRad( float anglerad )
{
  anglerad = normalize_mirror_rad( anglerad );

  if( IsRoundRobot() ) {
    /* use quadratic equation to get intersection point of ray to circle.
     * The ray origins at the laser with angle "anglerad" and is a unit_vector.
     * Consider robot-center as (0,0), we have an equation of:
     *    length(v_laser + k*ray) = radius + expansion
     * with v_laser = vector(m_laserOffsetX, m_laserOffsetY).
     * "k" is the length from the laser to the robot edge at angle "anglerad".
     *
     * Transform that equation, i.e. resolve "length(..)" and you get a
     * quadratic equation of the kind "ax^2 + 2bx + c = 0".
     */
    float ray_x = cos(anglerad); // unit vector!
    float ray_y = sin(anglerad);

    float a = ray_x*ray_x + ray_y*ray_y;
    float b = ray_x*m_laserOffsetX + ray_y*m_laserOffsetY;
    static float c = m_laserOffsetX*m_laserOffsetX + m_laserOffsetY*m_laserOffsetY - GetCompleteRadius()*GetCompleteRadius();

    return ( -b + sqrt(b*b - a*c) ) / a;

  } else if( IsAngularRobot() ) {
    /* check all the quadrants in which the target angles lies. The quadrants are spanned
     * by the angles from the center of the robot to its 4 corners. Use "cos(a) = adjacent / hypothenuse",
     * we are looking for the length of the hypothenuse here.
     */
    if( anglerad >= m_angBackLeft || anglerad < m_angBackRight ) {
      // bottom quadrant; fabs(anglerad) > M_PI_2
      return m_robotToBack  / cos( M_PI - fabs(anglerad) );

    } else if( anglerad < m_angFrontRight ) {
      // right quadrant; -M_PI < anglerad < 0
      return m_robotToRight / cos( M_PI_2 + anglerad );

    } else if( anglerad < m_angFrontLeft ) {
      // top quadrant; -M_PI_2 < anglerad < M_PI_2
      return m_robotToFront / cos( anglerad );

    } else if( anglerad < m_angBackLeft ) {
      // left quadrant; 0 < anglerad < M_PI
      return m_robotToLeft  / cos( M_PI_2 - anglerad);

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
RoboShape::GetRobotLengthforDegree( float angledeg )
{
  return GetRobotLengthforRad( deg2rad( angledeg ) );
}

/** Returns the radius of the robot if its round.
 * @return radius of the round robot
 */
float
RoboShape::GetRadius()
{
  if ( IsRoundRobot() )
    return m_radius;
  else
    logger_->log_error("RoboShape", "The Robot is not round!");

  return 0.0;
}

/** Returns the maximum radius of the robot if its round.
 * @return maximum radius of the round robot
 */
float
RoboShape::GetCompleteRadius()
{
  if ( IsRoundRobot() )
    return ( m_radius + std::max( std::max(m_widthAddFront, m_widthAddBack),
                                  std::max(m_widthAddRight, m_widthAddLeft) ) );
  else
    logger_->log_error("RoboShape", "Error: The Robot is not round!");

  return 0.0;
}
/** Returns the width-x of the angular robot.
 * @return only the robot x width.
 */
float
RoboShape::GetWidthX()
{
  if ( IsAngularRobot() )
    return m_widthX;
  else
    logger_->log_error("RoboShape", "The Robot is not angular!");

  return 0.0;
}

/** Returns the width-y of the angular robot.
 * @return only the robot y width.
 */
float
RoboShape::GetWidthY()
{
  if ( IsAngularRobot() )
    return m_widthY;
  else
    logger_->log_error("RoboShape", "The Robot is not angular!");

  return 0.0;
}

/** Returns the complete x width of the angular robot.
 * @return the complete x width.
 */
float
RoboShape::GetCompleteWidthX()
{
  if ( IsAngularRobot() )
    return ( m_widthX + m_widthAddFront + m_widthAddBack );
  else
    return 2.f*GetCompleteRadius();

  return 0.0;
}


/** Returns the complete y width of the angular robot.
 * @return the complete y width.
 */
float
RoboShape::GetCompleteWidthY()
{
  if ( IsAngularRobot() )
    return ( m_widthY + m_widthAddRight + m_widthAddLeft );
  else
    return 2.f*GetCompleteRadius();

  return 0.0;
}

/** Returns the laser offset in x direction of the robot.
 * @return the laser offset in x direction.
 */
float
RoboShape::GetLaserOffsetX()
{
  return m_laserOffsetX;
}

/** Returns the laser offset in y direction of the robot.
 * @return the laser offset in y direction.
 */
float
RoboShape::GetLaserOffsetY()
{
  return m_laserOffsetY;
}

} // namespace fawkes
