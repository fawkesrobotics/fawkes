
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

  if( (isinf(m_laserOffsetX) == true)
   && (isinf(m_laserOffsetY) == true)
   && (isinf(m_radius) == true)
   && (isinf(m_widthX) == true)
   && (isinf(m_widthY) == true)
   && (isinf(m_widthAddLeft) == true)
   && (isinf(m_widthAddRight) == true)
   && (isinf(m_widthAddFront) == true)
   && (isinf(m_widthAddBack) == true) ) {
    // go on, everything is fine, cause all are infinity

  } else {
    throw fawkes::Exception("RoboShape: Initializing Infinity-Values failed!");
  }

  m_widthAddFront = config->get_float((cfg + "WIDTH_ADD_FRONT").c_str());
  m_widthAddRight = config->get_float((cfg + "WIDTH_ADD_RIGHT").c_str());
  m_widthAddBack  = config->get_float((cfg + "WIDTH_ADD_BACK").c_str());
  m_widthAddLeft  = config->get_float((cfg + "WIDTH_ADD_LEFT").c_str());

  int shape = config->get_int((cfg + "ROBOSHAPE").c_str());
  if( shape == 1 ) {
    // ANGULAR
    m_isAngular = true;
    m_isRound = false;
    m_widthX        = config->get_float((cfg + "WIDTH_X").c_str());
    m_widthY        = config->get_float((cfg + "WIDTH_Y").c_str());
    m_laserOffsetX  = config->get_float((cfg + "LASER_OFFSET_X_FROM_BACK").c_str());
    m_laserOffsetY  = config->get_float((cfg + "LASER_OFFSET_Y_FROM_LEFT").c_str());

    m_robotToBack  =             m_laserOffsetX  + m_widthAddBack;
    m_robotToLeft  = (m_widthY - m_laserOffsetY) + m_widthAddLeft;
    m_robotToRight =             m_laserOffsetY  + m_widthAddRight;
    m_robotToFront = (m_widthX - m_laserOffsetX) + m_widthAddFront;

    logger_->log_info("RoboShape", "Shape is angular!");

  } else if ( shape == 2 ) {
    // ROUND
    m_isAngular = false;
    m_isRound = true;
    m_radius        = config->get_float((cfg + "RADIUS").c_str());
    m_laserOffsetX  = config->get_float((cfg + "LASER_OFFSET_X_FROM_MIDDLE").c_str());
    m_laserOffsetY  = config->get_float((cfg + "LASER_OFFSET_Y_FROM_MIDDLE").c_str());

    m_robotToBack  = m_radius + m_laserOffsetX + m_widthAddBack;
    m_robotToFront = m_radius + m_laserOffsetX + m_widthAddFront;
    m_robotToLeft  = m_radius + m_laserOffsetY + m_widthAddLeft;
    m_robotToRight = m_radius + m_laserOffsetY + m_widthAddRight;

    logger_->log_info("RoboShape", "Shape is round!");

  } else {
    // WRONG FORMAT!!!
    throw fawkes::Exception("RoboShape: Loading RoboShape from ConfigFile failed! Invalid config value for ROBOSHAPE");
  }

  logger_->log_info("RoboShape", "|#-->  (m)  is to front: %f", m_robotToFront);
  logger_->log_info("RoboShape", "|#-->  (m)  is to right: %f", m_robotToRight);
  logger_->log_info("RoboShape", "|#-->  (m)  is to left : %f", m_robotToLeft);
  logger_->log_info("RoboShape", "+#-->  (m)  is to back : %f", m_robotToBack);
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

/** Returns the robots length for a specific angle.
 * @param anglerad is the angle in radians.
 * @return the length in this direction.
 */
float
RoboShape::GetRobotLengthforRad( float anglerad )
{
  float length_front_back;
  float length_left_right;
  float alpha = 0.0;

  anglerad = normalize_mirror_rad( anglerad );

  if ( anglerad > 0 )
    length_left_right = m_robotToRight;
  else
    length_left_right = -m_robotToLeft;

  if ( fabs( anglerad ) > M_PI/2.0 )
    length_front_back = -m_robotToBack;
  else
    length_front_back =  m_robotToFront;

  alpha = atan2( length_left_right, length_front_back );

  //logger_->log_info("RoboShape", "Question for RobotLength for Rad is hard.....");
  //logger_->log_info("RoboShape", "You want to have length for angle %f", anglerad);
  //logger_->log_info("RoboShape", "My lengths are X = %f and Y = %f", length_front_back, length_left_right);
  //logger_->log_info("RoboShape", "My distinguishing alpha is %f", alpha);


  if ( IsRoundRobot() == true ) {
    // TODO
    throw fawkes::Exception("RoboShape: GetRobotLengthforRad is NOT IMPLEMENTED YET for round robots");

  } else if ( IsAngularRobot() == true ) {

    if ( (alpha >= -M_PI) && (alpha < -M_PI_2) ) {
      if ( anglerad < alpha ) {
        //cout << "RoboShape Note: CASE 1 ||| Calculating with base LFB " << length_front_back << endl;
        return ( fabs( length_front_back / cos( M_PI + anglerad) ) );
      } else {
        //cout << "RoboShape Note: CASE 2 ||| Calculating with base LLR " << length_left_right << endl;
        return ( fabs( length_left_right / cos( M_PI_2 + anglerad) ) );
      }

    } else if ( (alpha >= -M_PI_2) && (alpha < 0.0) ) {
      if ( anglerad < alpha ) {
        //cout << "RoboShape Note: CASE 3 ||| Calculating with base LLR " << length_left_right << endl;
        return ( fabs( length_left_right / cos( M_PI_2 + anglerad) ) );
      } else {
        //cout << "RoboShape Note: CASE 4 ||| Calculating with base LFB " << length_front_back << endl;
        return ( fabs( length_front_back / cos( anglerad) ) );
      }

    } else if ( (alpha >= 0.0) && (alpha < M_PI_2) ) {
      if ( anglerad < alpha ) {
        //cout << "RoboShape Note: CASE 5 ||| Calculating with base LFB " << length_front_back << endl;
        return ( fabs( length_front_back / cos( anglerad) ) );
      } else {
        //cout << "RoboShape Note: CASE 6 ||| Calculating with base LLR " << length_left_right << endl;
        return ( fabs( length_left_right / cos( M_PI_2 - anglerad) ) );
      }

    } else if ( (alpha >= M_PI_2) && (alpha < M_PI) ) {
      if ( anglerad < alpha ) {
        //cout << "RoboShape Note: CASE 7 ||| Calculating with base LLR " << length_left_right << endl;
        return ( fabs( length_left_right / cos( M_PI_2 - anglerad) ) );
      } else {
        //cout << "RoboShape Note: CASE 8 ||| Calculating with base LFB " << length_front_back << endl;
        return ( fabs( length_front_back / cos( M_PI - anglerad) ) );
      }

    } else {
      throw fawkes::Exception("RoboShape: alpha has no valid value");
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
  if ( IsRoundRobot() == true )
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
  if ( IsRoundRobot() == true )
    return ( std::max( m_radius + m_widthAddFront + m_widthAddBack,
                       m_radius + m_widthAddRight + m_widthAddLeft ) );
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
  if ( IsAngularRobot() == true )
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
  if ( IsAngularRobot() == true )
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
  if ( IsAngularRobot() == true )
    return ( m_widthX + m_widthAddFront + m_widthAddBack );
  else
    logger_->log_error("RoboShape", "The Robot is not angular!");

  return 0.0;
}


/** Returns the complete y width of the angular robot.
 * @return the complete y width.
 */
float
RoboShape::GetCompleteWidthY()
{
  if ( IsAngularRobot() == true )
    return ( m_widthY + m_widthAddRight + m_widthAddLeft );
  else
    logger_->log_error("RoboShape", "The Robot is not angular!");

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
