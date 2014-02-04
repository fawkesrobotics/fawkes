
/***************************************************************************
 *  roboshape.h - Class containing shape information of robot
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

  bool IsRoundRobot( );
  bool IsAngularRobot( );

  ///\brief Check if the reading is 'in' the robot
  bool IsRobotReadingforRad( float anglerad, float length );

  ///\brief Check if the reading is 'in' the robot
  bool IsRobotReadingforDegree( float angledeg, float length );

  ///\brief return the length of the robot for a specific angle
  float GetRobotLengthforRad( float anglerad );

  ///\brief return the length of the robot for a specific angle
  float GetRobotLengthforDegree( float angledeg );

  ///\brief Returns the radius of the robot if its round.
  float GetRadius();

  ///\brief Returns the maximum radius of the robot if its round.
  float GetCompleteRadius();

  ///\brief Returns the width-x of the angular robot.
  float GetWidthX();

  ///\brief Returns the width-y of the angular robot.
  float GetWidthY();

  ///\brief Returns the complete x width of the angular robot.
  float GetCompleteWidthX();

  ///\brief Returns the complete x width of the angular robot.
  float GetCompleteWidthY();

  ///\brief Returns the laser offset in x direction of the robot.
  float GetLaserOffsetX();

  ///\brief Returns the laser offset in y direction of the robot.
  float GetLaserOffsetY();

  ///\brief Get angle to the front left corner of the robot
  float GetAngleFrontLeft() const;

  ///\brief Get angle to the front right corner of the robot
  float GetAngleFrontRight() const;

  ///\brief Get angle to of the rear left corner robot
  float GetAngleBackLeft() const;

  ///\brief Get angle to of the rear right corner robot
  float GetAngleBackRight() const;

  ///\brief Get angle to middle of the left side of the robot
  float GetAngleLeft() const;

  ///\brief Get angle to middle of the right side of the robot
  float GetAngleRight() const;

  ///\brief Get angle to middle of the front side of the robot
  float GetAngleFront() const;

  ///\brief Get angle to middle of the back side of the robot
  float GetAngleBack() const;


private:

  bool m_isRound;    /**< flag if the robot is round */
  bool m_isAngular;  /**< flag if the robot is angular */

  // several variables containing information about the robot.
  float m_radius, m_widthX, m_widthY;
  float m_laserOffsetX, m_laserOffsetY;
  float m_widthAddFront, m_widthAddBack, m_widthAddLeft, m_widthAddRight;
  float m_robotToFront, m_robotToRight, m_robotToBack, m_robotToLeft;

  // angles to the "corners" and mid-sections of the complete roboshape
  float m_angFrontLeft, m_angFrontRight, m_angBackLeft, m_angBackRight;
  float m_angLeft, m_angRight, m_angFront, m_angBack;

  fawkes::Logger* logger_;
};

} // namespace fawkes

#endif
