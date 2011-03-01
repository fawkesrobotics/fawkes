
/***************************************************************************
 *  or_thread.cpp - OpenRAVE Thread
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#include "or_thread.h"

#include "environment.h"
#include "robot.h"
#include "manipulator.h"

#include <openrave-core.h>
#include <core/exceptions/software.h>

using namespace fawkes;

/** @class OpenRAVEThread "or_thread.h"
 * OpenRAVE Thread.
 * This thread maintains an active connection to OpenRAVE and provides an
 * aspect to access OpenRAVE to make it convenient for other threads to use
 * OpenRAVE.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor. */
OpenRAVEThread::OpenRAVEThread()
  : Thread("OpenRAVEThread", Thread::OPMODE_CONTINUOUS),
    AspectProviderAspect("OpenRAVEAspect", &__or_aspectIniFin),
    __or_aspectIniFin(this)
{
}


/** Destructor. */
OpenRAVEThread::~OpenRAVEThread()
{
}


void
OpenRAVEThread::init()
{
  try {
    OpenRAVE::RaveInitialize(true);
  } catch(const OpenRAVE::openrave_exception &e) {
    logger->log_error(name(), "Could not initialize OpenRAVE. Ex:%s", e.what());
  }

  __OREnv = new OpenRAVEEnvironment(logger);

  __OREnv->create();
  __OREnv->enableDebug(); // TODO: cfg

  __OREnv->lock();
}


void
OpenRAVEThread::finalize()
{
  __OREnv->destroy();
}


void
OpenRAVEThread::loop()
{
}

/* ##########################################################
 * #    methods form OpenRAVEConnector (or_connector.h)     #
 * ########################################################*/
/** Get pointer to OpenRAVEEnvironment object.
 * @return pointer
 */
OpenRAVEEnvironment*
OpenRAVEThread::getEnvironment() const
{
  return __OREnv;
}

/** Get pointer to currently used OpenRAVERobot object.
 * @return pointer
 */
OpenRAVERobot*
OpenRAVEThread::getActiveRobot() const
{
  return __ORRobot;
}

/** Set robot to be used
 * @param robot OpenRAVERobot that should be used implicitly in other methods
 */
void
OpenRAVEThread::setActiveRobot(OpenRAVERobot* robot)
{
  __ORRobot = robot;
}

/** Set OpenRAVEManipulator object for robot, and calculate
 * coordinate-system offsets or set them directly.
 * Make sure to update manip angles before calibrating!
 * @param calibrate decides whether to calculate offset (true )or set them directly (false; default)
 */
void
OpenRAVEThread::setManipulator(OpenRAVERobot* robot, OpenRAVEManipulator* manip, float transX, float transY, float transZ, bool calibrate)
{
  robot->setManipulator(manip);
  if( calibrate )
    {robot->calibrate(transX, transY, transZ);}
  else
    {robot->setOffset(transX, transY, transZ);}
}
void
OpenRAVEThread::setManipulator(OpenRAVEManipulator* manip, float transX, float transY, float transZ, bool calibrate)
{
  setManipulator(__ORRobot, manip, transX, transY, transZ, calibrate);
}


/** Start Viewer */
void
OpenRAVEThread::startViewer() const
{
  __OREnv->startViewer();
}

/** Run planner on previously set target.
 * @param robot robot to use planner on. If none is given, the currently used robot is taken
 */
void
OpenRAVEThread::runPlanner(OpenRAVERobot* robot)
{
  if(!robot)
    robot = __ORRobot;

  __OREnv->runPlanner(robot);
}

/** Add a new robot to the environment, and set it as the currently active one.
 * @param filenameRobot path to robot's xml file
 * @param autogenerateIK if true: autogenerate IKfast IK solver for robot
 * @return pointer to new OpenRAVERobot object
 */
OpenRAVERobot*
OpenRAVEThread::addRobot(const std::string& filenameRobot, bool autogenerateIK)
{
  OpenRAVERobot* robot = new OpenRAVERobot(logger);
  robot->load(filenameRobot, __OREnv);
  __OREnv->addRobot(robot);
  robot->setReady();

  if( autogenerateIK )
    __OREnv->loadIKSolver(robot);

  setActiveRobot(robot);

  return robot;
}