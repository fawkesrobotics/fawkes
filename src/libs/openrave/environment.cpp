
/***************************************************************************
 *  environment.cpp - Fawkes to OpenRAVE Environment
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
#include "environment.h"
#include "robot.h"

#include <openrave-core.h>
#include <utils/logging/logger.h>

using namespace OpenRAVE;
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


void
SetViewer(OpenRAVE::EnvironmentBasePtr env, const std::string& viewername)
{
  ViewerBasePtr viewer = RaveCreateViewer(env, viewername);
  BOOST_ASSERT(!!viewer);

  // attach it to the environment:
  env->AttachViewer(viewer);

  // finally you call the viewer's infinite loop (this is why you need a separate thread):
  viewer->main(/*showGUI=*/true);
}


/** @class OpenRAVEEnvironment <openrave/environment.h>
 * Class handling interaction with the OpenRAVE::EnvironmentBase class.
 * This class loads a scene and adds robots/objects etc to it. All calculations
 * in OpenRAVE (IK, planning, etc) are done based on the current scene.
 * @author Bahram Maleki-Fard
 */

/** Constructor
 * @param logger pointer to fawkes logger
 */
OpenRAVEEnvironment::OpenRAVEEnvironment(fawkes::Logger* logger) :
  __name( "OpenRAVE Environment" ),
  __logger( logger )
{
}

/** Destructor. */
OpenRAVEEnvironment::~OpenRAVEEnvironment()
{
  this->destroy();
}

/** Create and lock the environment */
void
OpenRAVEEnvironment::create()
{
  try {
    __env = RaveCreateEnvironment();
    if(__logger)
      __logger->log_debug(__name, "Environment created");
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_warn(__name, "Could not create Environment. Ex:%s", e.what());
    throw;
  }
}

/** Destroy the environment */
void
OpenRAVEEnvironment::destroy()
{
  try {
    __env->Destroy();
    if(__logger)
      __logger->log_debug(__name, "Environment destroyed");
  } catch(const openrave_exception& e) {
    __logger->log_warn(__name, "Could not destroy Environment. Ex:%s", e.what());
  }
}

/** Lock the environment to prevent changes */
void
OpenRAVEEnvironment::lock()
{
  EnvironmentMutex::scoped_lock lock(__env->GetMutex());
}

/** Enable debugging messages of OpenRAVE */
void
OpenRAVEEnvironment::enableDebug()
{
  __env->SetDebugLevel(Level_Debug);
}

/** Disable debugging messages of OpenRAVE */
void
OpenRAVEEnvironment::disableDebug()
{
  __env->SetDebugLevel(Level_Fatal);
}

/** Add a robot into the scene
 * @param robot RobotBasePtr of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
bool
OpenRAVEEnvironment::addRobot(RobotBasePtr robot)
{
    return __env->AddRobot(robot);
}

/** Add a robot into the scene
 * @param filename path to robot's xml file
 * @return 1 if succeeded, 0 if not able to load file
 */
bool
OpenRAVEEnvironment::addRobot(const std::string& filename)
{
  // load the robot
  RobotBasePtr robot;
  try {
    robot = __env->ReadRobotXMLFile(filename);
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_warn(__name, "Robot could not be loaded. Ex:%s", e.what());
    return 0;
  }

  if( !robot ) {
    // could not load robot file. Check file path, and test file itself for correct syntax and semantics
    // by loading it directly into openrave with "openrave robotfile.xml"
    if(__logger)
      __logger->log_warn(__name, "Robot could not be loaded.");
    return 0;
  } else {
    return addRobot(robot);
  }
}

bool
OpenRAVEEnvironment::addRobot(OpenRAVERobot* robot)
{
    return addRobot(robot->getRobotPtr());
}

OpenRAVE::EnvironmentBasePtr
OpenRAVEEnvironment::getEnvPtr() const
{
  return __env;
}

/** Starts the  qt viewer in a separate thread.
 *  Use this mainly for debugging purposes, as it uses
 *  a lot of CPU/GPU resources.
 */
void
OpenRAVEEnvironment::startViewer()
{
  try {
    boost::thread thviewer(boost::bind(SetViewer,__env,"qtcoin"));
  } catch( const openrave_exception &e) {
    if(__logger)
      __logger->log_warn(__name, "Could not load viewr. Ex:%s", e.what());
    throw;
  }
}

} // end of namespace fawkes
