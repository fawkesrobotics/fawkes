
/***************************************************************************
 *  connector.h - Fawkes to OpenRAVE Connector
 *
 *  Created: Thu Sep 16 14:50:34 2010
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

#ifndef __OPENRAVE_CONNECTOR_H_
#define __OPENRAVE_CONNECTOR_H_

#include "types.h"

#include <rave/rave.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class OpenRAVEEnvironment;
class OpenRAVERobot;
class OpenRAVEManipulator;

/** This is the main class that should be used in a fawkes plugin to
 * build a connection to OpenRAVE.
 */
class OpenRAVEConnector
{
 public:
  OpenRAVEConnector(fawkes::Logger* logger = 0);
  virtual ~OpenRAVEConnector();

  virtual void setup(const std::string& filenameRobot, bool autogenerateIK=false);

  virtual void setManipulator(OpenRAVEManipulator* manip);

  virtual void setTarget(std::vector<float>& angles); //temporary. TODO: should be euler/quat/axisangle etc
  virtual bool setTargetQuat	 (float& transX, float& transY, float& transZ, float& quatW, float& quatX, float& quatY, float& quatZ);
  virtual bool setTargetAxisAngle(float& transX, float& transY, float& transZ, float& angle, float& axisX, float& axisY, float& axisZ);
  virtual bool setTargetEuler( euler_rotation_t type, float& transX, float& transY, float& transZ, float& phi, float& theta, float& psi);

  virtual void startViewer() const;
  virtual void runPlanner();

  virtual std::vector< std::vector<float> >* getTrajectory() const;
  virtual OpenRAVEEnvironment* getEnvironment() const;
  virtual OpenRAVERobot* getRobot() const;

 private:
  fawkes::Logger*	__logger;

  OpenRAVEEnvironment*  __env;
  OpenRAVERobot*        __robot;

  std::vector< std::vector<float> >*     __traj;

};

} // end of namespace fawkes

#endif