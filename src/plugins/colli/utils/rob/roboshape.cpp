
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
#include <utils/math/angle.h>

#include <cmath>
#include <string>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

// initialize RoboShape
RoboShape::RoboShape( const char * cfg_prefix,
                      fawkes::Logger* logger,
                      fawkes::Configuration* config) throw (int)
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
    logger_->log_error("RoboShape", "Error: Initializing Infinity-Values failed! Aborting...");
    exit(0);
  }

  int shape = config->get_int((cfg + "ROBOSHAPE").c_str());
  if( shape == 1 ) {
    // ANGULAR
    m_isAngular = true;
    m_isRound = false;
    m_widthX        = config->get_float((cfg + "WIDTH_X").c_str());
    m_widthY        = config->get_float((cfg + "WIDTH_Y").c_str());
    m_widthAddFront = config->get_float((cfg + "WIDTH_ADD_FRONT").c_str());
    m_widthAddRight = config->get_float((cfg + "WIDTH_ADD_RIGHT").c_str());
    m_widthAddBack  = config->get_float((cfg + "WIDTH_ADD_BACK").c_str());
    m_widthAddLeft  = config->get_float((cfg + "WIDTH_ADD_LEFT").c_str());
    m_laserOffsetX  = config->get_float((cfg + "LASER_OFFSET_X_FROM_BACK").c_str());
    m_laserOffsetY  = config->get_float((cfg + "LASER_OFFSET_Y_FROM_LEFT").c_str());

    m_robotToBack  =             m_laserOffsetX  + m_widthAddBack;
    m_robotToLeft  = (m_widthY - m_laserOffsetY) + m_widthAddLeft;
    m_robotToRight =             m_laserOffsetY  + m_widthAddRight;
    m_robotToFront = (m_widthX - m_laserOffsetX) + m_widthAddFront;

    logger_->log_info("RoboShape", "Shape is angular!");
    logger_->log_info("RoboShape", "|#-->  (m)  is to front: %f", m_robotToFront);
    logger_->log_info("RoboShape", "|#-->  (m)  is to right: %f", m_robotToRight);
    logger_->log_info("RoboShape", "|#-->  (m)  is to left : %f", m_robotToLeft);
    logger_->log_info("RoboShape", "+#-->  (m)  is to back : %f", m_robotToBack);

  } else if ( shape == 2 ) {
    // ROUND
    m_isAngular = false;
    m_isRound = true;
    m_radius        = config->get_float((cfg + "RADIUS").c_str());
    m_widthAddFront = config->get_float((cfg + "WIDTH_ADD_FRONT").c_str());
    m_widthAddRight = config->get_float((cfg + "WIDTH_ADD_RIGHT").c_str());
    m_widthAddBack  = config->get_float((cfg + "WIDTH_ADD_BACK").c_str());
    m_widthAddLeft  = config->get_float((cfg + "WIDTH_ADD_LEFT").c_str());
    m_laserOffsetX  = config->get_float((cfg + "LASER_OFFSET_X_FROM_MIDDLE").c_str());
    m_laserOffsetY  = config->get_float((cfg + "LASER_OFFSET_Y_FROM_MIDDLE").c_str());

    m_robotToBack  = m_radius + m_laserOffsetX + m_widthAddBack;
    m_robotToFront = m_radius + m_laserOffsetX + m_widthAddFront;
    m_robotToLeft  = m_radius + m_laserOffsetY + m_widthAddLeft;
    m_robotToRight = m_radius + m_laserOffsetY + m_widthAddRight;

    logger_->log_info("RoboShape", "Shape is round!");
    logger_->log_info("RoboShape", "|#-->  (m)  is to front: %f", m_robotToFront);
    logger_->log_info("RoboShape", "|#-->  (m)  is to right: %f", m_robotToRight);
    logger_->log_info("RoboShape", "|#-->  (m)  is to left : %f", m_robotToLeft);
    logger_->log_info("RoboShape", "+#-->  (m)  is to back : %f", m_robotToBack);

  } else {
    // WRONG FORMAT!!!
    m_isAngular = false;
    m_isRound = false;
    logger_->log_error("RoboShape", "Error: Loading RoboShape from ConfigFile failed! Aborting...");
    exit(0);
  }
}


// destruct RoboShape
RoboShape::~RoboShape()
{
}

// return if it is a round robot
bool
RoboShape::IsRoundRobot()
{
  return m_isRound;
}

// return if it is a angular robot
bool
RoboShape::IsAngularRobot()
{
  return m_isAngular;
}

// returns if the reading is 'in' the robot
bool
RoboShape::IsRobotReadingforRad( float anglerad, float length )
{
  return (length < GetRobotLengthforRad( anglerad ));
}

// returns if the reading is 'in' the robot
bool
RoboShape::IsRobotReadingforDegree( float angledeg, float length )
{
  return IsRobotReadingforRad( deg2rad( angledeg ), length );
}

// return the length of the robot for a specific angle
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
    logger_->log_error("RoboShape", "GetRobotLengthforRad is NOT IMPLEMENTED YET for round robots");
    exit(0);

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
      logger_->log_error("RoboShape", "alpha has no valid value");
      exit(0);
    }

  } else {
    logger_->log_error("RoboShape", "Error: Cannot return the robolength for unspecific robot!");
    exit(0);
  }
}

// return the length of the robot for a specific angle
float
RoboShape::GetRobotLengthforDegree( float angledeg )
{
  return GetRobotLengthforRad( deg2rad( angledeg ) );
}

// return if it is a rod (1), if it is not (0) or unsure(3)
//  for a specific angle
int
RoboShape::IsRodforRad( float anglerad )
{
  anglerad = normalize_mirror_rad( anglerad );

  if ( (normalize_mirror_rad( deg2rad(  28 )) < anglerad ) && ( anglerad < normalize_mirror_rad( deg2rad(  35 ))) )
    return 3;
  else if ( (normalize_mirror_rad( deg2rad(  99 )) < anglerad ) && ( anglerad < normalize_mirror_rad( deg2rad( 113 ))) )
    return 3;
  else if ( (normalize_mirror_rad( deg2rad( 249 )) < anglerad ) && ( anglerad < normalize_mirror_rad( deg2rad( 260 ))) )
    return 3;
  else if ( (normalize_mirror_rad( deg2rad( 325 )) < anglerad ) && ( anglerad < normalize_mirror_rad( deg2rad( 331 ))) )
    return 3;
  else
    return 0;
}

// return if it is a rod (1), if it is not (0) or unsure(3)
//  for a specific angle
int
RoboShape::IsRodforDegree( float angledeg )
{
  return IsRodforRad( deg2rad( angledeg ) );
}

// return the radius of the round robot if it is round
float
RoboShape::GetRadius()
{
  if ( IsRoundRobot() == true )
    return m_radius;
  else {
    logger_->log_error("RoboShape", "The Robot is not round!");
    exit(0);
  }
  return 0.0;
}

float
RoboShape::GetCompleteRadius()
{
  if ( IsRoundRobot() == true )
    return ( std::max( m_radius + m_widthAddFront + m_widthAddBack,
                       m_radius + m_widthAddRight + m_widthAddLeft ) );
  else {
    logger_->log_error("RoboShape", "Error: The Robot is not round!");
    exit(0);
  }
  return 0.0;
}

// returns complete width in x direction
float
RoboShape::GetCompleteWidthX()
{
  if ( IsAngularRobot() == true )
    return ( m_widthX + m_widthAddFront + m_widthAddBack );
  else {
    logger_->log_error("RoboShape", "The Robot is not angular!");
    exit(0);
  }
  return 0.0;
}


// returns complete width in y direction
float
RoboShape::GetCompleteWidthY()
{
  if ( IsAngularRobot() == true )
    return ( m_widthY + m_widthAddRight + m_widthAddLeft );
  else {
  logger_->log_error("RoboShape", "The Robot is not angular!");
  exit(0);
  }
  return 0.0;
}

// returns robowidth in x direction
float
RoboShape::GetWidthX()
{
  if ( IsAngularRobot() == true )
    return m_widthX;
  else
    {
      logger_->log_error("RoboShape", "The Robot is not angular!");
      exit(0);
    }
  return 0.0;
}


// returns robowidth in y direction
float
RoboShape::GetWidthY()
{
  if ( IsAngularRobot() == true )
    return m_widthY;
  else {
    logger_->log_error("RoboShape", "The Robot is not angular!");
    exit(0);
  }
  return 0.0;
}


// returns the laseroffset in x direction
float
RoboShape::GetLaserOffsetX()
{
  if ( isinf(m_laserOffsetX) == false )
    return m_laserOffsetX;
  else {
    logger_->log_error("RoboShape", "LaserOffsetX is not set!");
    exit(0);
  }
  return 0.0;
}


// returns the laseroffset in y direction
float
RoboShape::GetLaserOffsetY()
{
  if ( isinf(m_laserOffsetY) == false )
    return m_laserOffsetY;
  else  {
    logger_->log_error("RoboShape", "LaserOffsetY is not set!");
    exit(0);
  }
  return 0.0;
}

} // namespace fawkes
