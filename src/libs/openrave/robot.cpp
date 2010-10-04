
/***************************************************************************
 *  robot.cpp - Fawkes to OpenRAVE Robot Handler
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

#include "robot.h"
#include "motors.h"
#include "environment.h"

#include <openrave-core.h>
#include <utils/logging/logger.h>

using namespace OpenRAVE;
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Constructor */
OpenRAVERobot::OpenRAVERobot(fawkes::Logger* logger) :
  __logger( logger ),
  __name( "" ),
  __motors( 0 )
{
  init();
}
OpenRAVERobot::OpenRAVERobot(const std::string& filename, fawkes::OpenRAVEEnvironment* env, fawkes::Logger* logger) :
  __logger( logger ),
  __name( "" ),
  __motors( 0 )
{
  init();
  this->load(filename, env);
}

/** Destructor */
OpenRAVERobot::~OpenRAVERobot()
{
}

/** Inittialize object attributes */
void
OpenRAVERobot::init()
{
  __posCurrent = new Transform();
  __posTarget = new Transform();
}


/** Load robot from xml file
 *@param filename path to robot's xml file
 */
bool
OpenRAVERobot::load(const std::string& filename, fawkes::OpenRAVEEnvironment* env)
{
  // load the robot
  // TODO: implementing without usage of 'environment'
  try {
    __robot = env->getEnvPtr()->ReadRobotXMLFile(filename);
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Robot", "Robot could not be loaded. Ex:%s", e.what());
    return 0;
  }

  if(!__robot) {
    if(__logger)
      __logger->log_warn("OpenRAVE Robot", "Robot could not be loaded.");
    return false;
  } else {
    __name = __robot->GetName();
     __robot->SetActiveManipulator(__robot->GetManipulators().at(0)->GetName());
    __arm = __robot->GetActiveManipulator();
    return true;
  }
}

/** Set pointer to OpenRAVEMotors object */
void
OpenRAVERobot::setMotors(OpenRAVEMotors* motors)
{
  __motors = motors;
}

/** Update motor values from OpenRAVE model */
void
OpenRAVERobot::updateMotors()
{
  std::vector<float> angles;
  __robot->GetDOFValues(angles);
  __motors->setAngles(angles);

  *__posCurrent = __arm->GetEndEffectorTransform();
}



void
OpenRAVERobot::setTargetTransform(OpenRAVE::Vector& trans, OpenRAVE::Vector& rotQuat)
{
  __posTarget->trans = trans;
  __posTarget->rot = rotQuat;
}


OpenRAVE::RobotBasePtr
OpenRAVERobot::getRobotPtr() const
{
  return __robot;
}

} // end of namespace fawkes