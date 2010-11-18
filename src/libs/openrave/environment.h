
/***************************************************************************
 *  environment.h - Fawkes to OpenRAVE Environment
 *
 *  Created: Sun Sep 19 14:50:34 2010
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

#ifndef __OPENRAVE_ENVIRONMEN_H_
#define __OPENRAVE_ENVIRONMEN_H_

#include <rave/rave.h>
#include <string>

namespace OpenRAVE {
  class EnvironmentBase;
  class RobotBase;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void SetViewer(OpenRAVE::EnvironmentBasePtr penv, const std::string& viewername);

class Logger;
class OpenRAVERobot;
class OpenRAVEEnvironment
{
 public:
  OpenRAVEEnvironment(fawkes::Logger* logger = 0);
  ~OpenRAVEEnvironment();

  virtual void create();
  virtual void destroy();
  virtual void lock();

  virtual void enableDebug();
  virtual void disableDebug();

  virtual void startViewer();
  virtual void runPlanner(OpenRAVERobot* robot);

  virtual void addRobot(const std::string& filename);
  virtual void addRobot(OpenRAVE::RobotBasePtr robot);
  virtual void addRobot(OpenRAVERobot* robot);


  //virtual RobotBasePtr getRobot() const;
  virtual OpenRAVE::EnvironmentBasePtr getEnvPtr() const;

 private:
  fawkes::Logger*	__logger;

  OpenRAVE::EnvironmentBasePtr	__env;
  OpenRAVE::PlannerBasePtr      __planner;
};
} // end of namespace fawkes

#endif