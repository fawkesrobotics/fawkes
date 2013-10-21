
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

/** My RoboShape class.
 *  This is a class containing all roboshape information.
 *   All methods have been implemented but round robots.
 */
class RoboShape
{
 public:

  // =============================== //
  // CLASS METHODS                   //
  // =============================== //

  /**  This is the constructor. Has to be called with the
   *   name, of the shape description file.
   *  @param shapeFileName is the name of the shape
   *         description file
   */
  RoboShape( const char * cfg_prefix,
             fawkes::Logger* logger,
             fawkes::Configuration* config);

  /** Destructor.
   */
  ~RoboShape();

  // =============================== //
  // CLASS GETTER METHODS            //
  // =============================== //

  /** Returns if the robot is round.
   *  @return bool indicating if the robot is round.
   */
  bool IsRoundRobot( );

  /** Returns if the robot is angular.
   *  @return bool indicating if the robot is angular.
   */
  bool IsAngularRobot( );


  /** Returns, if a reading length is _in_ the robot.
   *  @param anglerad is float containing the angle of
   *      the reading in radians.
   *  @param length containing the length of the reading.
   *  @return if the reading is in the robot.
   */
  bool IsRobotReadingforRad( float anglerad, float length );

  /** Returns, if a reading length is _in_ the robot.
   *  @param angledeg is float containing the angle of
   *      the reading in degree.
   *  @param length containing the length of the reading.
   *  @return if the reading is in the robot.
   */
  bool IsRobotReadingforDegree( float angledeg, float length );


  /** Returns the robots length for a specific angle.
   *  @param anglerad is the angle in radians.
   *  @return the length in this direction.
   */
  float GetRobotLengthforRad( float anglerad );

  /** Returns the robots length for a specific angle.
   *  @param angledeg is the angle in degree.
   *  @return the length in this direction.
   */
  float GetRobotLengthforDegree( float angledeg );


  /** Returns if there is a rod waiting in this direction.
   *  @param anglerad is an angle in radians.
   *  @return 0 if there is no rod, 1 if there is,
   *      and 3 if unsure (for now all rods are unsure).
   */
  static int IsRodforRad( float anglerad );

  /** Returns if there is a rod waiting in this direction.
   *  @param angledeg is an angle in degree.
   *  @return 0 if there is no rod, 1 if there is,
   *      and 3 if unsure (for now all rods are unsure).
   */
  int IsRodforDegree( float angledeg );

  /** Returns the radius of the robot if its round.
   *  @return radius of the round robot
   */
  float GetRadius();

  /** Returns the maximum radius of the robot if its round.
   *  @return maximumradius of the round robot
   */
  float GetCompleteRadius();

  /** Returns the widthx of the angular robot.
   *  @return only the robot x width.
   */
  float GetWidthX();

  /** Returns the widthy of the angular robot.
   *  @return only the robot y width.
   */
  float GetWidthY();

  /** Returns the complete x width of the angular robot.
   *  @return the complete x width.
   */
  float GetCompleteWidthX();

  /** Returns the complete y width of the angular robot.
   *  @return the complete y width.
   */
  float GetCompleteWidthY();

  /** Returns the laser offset in x direction of the robot.
   *  @return the laser offset in x direction.
   */
  float GetLaserOffsetX();

  /** Returns the laser offset in y direction of the robot.
   *  @return the laser offset in y direction.
   */
  float GetLaserOffsetY();


  // ======================================================= //


 private:

  // VARIABLES

  // flag if the robot is round or not
  bool m_isRound, m_isAngular;

  // several variables containing information about the robot.
  float m_radius, m_widthX, m_widthY;
  float m_laserOffsetX, m_laserOffsetY;
  float m_widthAddFront, m_widthAddBack, m_widthAddLeft, m_widthAddRight;
  float m_robotToFront, m_robotToRight, m_robotToBack, m_robotToLeft;

  fawkes::Logger* logger_;
};

} // namespace fawkes

#endif
