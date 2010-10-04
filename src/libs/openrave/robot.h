
/***************************************************************************
 *  robot.h - Fawkes to OpenRAVE Robot Handler
 *
 *  Created: Mon Sep 20 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef __OPENRAVE_ROBOT_H
#define __OPENRAVE_ROBOT_H

#include <rave/rave.h>
//#include <rave/geometry.h>


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class OpenRAVEManipulator;
class OpenRAVEEnvironment;

class OpenRAVERobot
{
 public:
  OpenRAVERobot(fawkes::Logger* logger = 0);
  OpenRAVERobot(const std::string& filename, fawkes::OpenRAVEEnvironment* env, fawkes::Logger* logger = 0);
  virtual ~OpenRAVERobot();

  // build/load robot parts
  virtual bool load(const std::string& filename, fawkes::OpenRAVEEnvironment* env);
  virtual void setManipulator(OpenRAVEManipulator* manip);
  virtual void updateManipulator();

  //virtual void setTargetQuat	 (float transX, float transY, float transZ, float quatW, float quatX, float quatY, float quatZ);
  //virtual void setTargetAxisAngle(float transX, float transY, float transZ, float angle, float axisX, float axisY, float axisZ);
  //virtual void setTargetEuler	 (float transX, float transY, float transZ, float phi, float theta, float psi);

  //virtual void solveIK(); // maybe automatically with setTarget...() ?
  //virtual bool isRunning() const;

  virtual OpenRAVE::RobotBasePtr getRobotPtr() const;
 private:
  void init();
  void setTargetTransform(OpenRAVE::Vector& trans, OpenRAVE::Vector& rotQuat);

  fawkes::Logger*	__logger;

  OpenRAVE::RobotBasePtr        __robot;
  std::string                   __name;
  OpenRAVE::RobotBase::ManipulatorPtr  __arm;
  OpenRAVEManipulator*	        __manip;
  OpenRAVE::Transform*		__posCurrent;
  OpenRAVE::Transform*		__posTarget;





  //bool			        __running;
};

} // end of namespace fawkes

#endif