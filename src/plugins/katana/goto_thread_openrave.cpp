
/***************************************************************************
 *  goto_thread_openrave.cpp - Katana goto one-time thread using openrave lib
 *
 *  Created: Wed Jun 10 11:45:31 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *                  2010  Bahram Maleki-Fard
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

#include "goto_thread_openrave.h"
#include "conversion.h"

#include <cstdlib>
#include <kniBase.h>

#ifdef HAVE_OPENRAVE

#include <openrave/connector.h>
#include <openrave/manipulators/katana6M180.h>
#include <openrave/types.h>

#include <vector>

using namespace fawkes;

/** @class KatanaGotoThreadOpenRAVE "goto_thread_openrave.h"
 * Katana collision-free goto thread.
 * This thread moves the arm into a specified position,
 * using IK and path-planning from OpenRAVE.
 * @author Tim Niemueller (goto_thread.h/cpp)
 * @author Bahram Maleki-Fard (OpenRAVE extension)
 */

/** Constructor.
 * @param katana linear motion base class
 * @param logger logger
 * @param robotFile path to robot's xml-file
 * @param useViewer true, if viewer should be started (default: false)
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 */
KatanaGotoThreadOpenRAVE::KatanaGotoThreadOpenRAVE(fawkes::RefPtr<CLMBase> katana,
				   fawkes::Logger *logger,
				   unsigned int poll_interval_ms,
                                   std::string robotFile,
                                   bool autoloadIK,
                                   bool useViewer) :
  KatanaGotoThread(katana, logger, poll_interval_ms),
  __ORCon( 0 ),
  __manip( 0 ),
  __targetTraj( 0 ),
  __cfg_robotFile( robotFile ),
  __cfg_autoloadIK( autoloadIK ),
  __cfg_useViewer( useViewer )
{
}


/** Set target position.
 * @param x X coordinate relative to base
 * @param y Y coordinate relative to base
 * @param z Z coordinate relative to base
 * @param phi Phi Euler angle of tool
 * @param theta Theta Euler angle of tool
 * @param psi Psi Euler angle of tool
 */
void
KatanaGotoThreadOpenRAVE::set_target(float x, float y, float z,
			     float phi, float theta, float psi)
{
  __x     = x/1000.f;
  __y     = y/1000.f;
  __z     = z/1000.f;
  __phi   = (phi);
  __theta = (theta);
  __psi   = (psi);
}

void
KatanaGotoThreadOpenRAVE::init()
{
  try {
    __ORCon = new OpenRAVEConnector(_logger);

    __ORCon->setup(__cfg_robotFile, __cfg_autoloadIK);

    // configure manipulator
    // TODO: from config parameters? neccessary?
    __manip = new OpenRAVEManipulatorKatana6M180(6, 5);
    __manip->addMotor(0,0);
    __manip->addMotor(1,1);
    __manip->addMotor(2,2);
    __manip->addMotor(4,3);
    __manip->addMotor(5,4);

    // Set manipulator and offsets.
    // offsetZ: katana.kinbody is 0.185 above ground; coordinate system of real katana has origin in intersection of j1 and j2 (i.e. start of link L2: 0.2015 on z-axis)
    // offsetX: katana.kinbody is setup 0.0725 on +x axis
    __ORCon->setManipulator(__manip, 0.0725f, 0.f, 0.3865f);

  } catch (fawkes::Exception& e) {
    // TODO: not just simple throw....
    throw;
  } catch (/*KNI*/::Exception &e) {
    _logger->log_warn(name(), "Fetching position values failed: %s", e.what());
  }

  if( __cfg_useViewer)
    __ORCon->startViewer();
}

void
KatanaGotoThreadOpenRAVE::finalize()
{
  delete(__ORCon);
  __ORCon = NULL;

  delete(__manip);
  __manip = NULL;
}

void
KatanaGotoThreadOpenRAVE::once()
{
  // Fetch motor encoder values
  if( !updateMotorData() ) {
    _logger->log_warn("KatanaGotoThread", "Fetching current motor values failed");
    _finished = true;
    return;
  }

  // Convert encoder values to angles, and set starting point for planner
  encToRad(__motorEncoders, __motorAngles);
  __manip->setAnglesDevice(__motorAngles);

  // Checking if target has IK solution
  _logger->log_debug(name(), "Check IK(%f,%f,%f, %f,%f,%f)",
		       __x, __y, __z, __phi, __theta, __psi);
  if( !__ORCon->setTargetEuler(EULER_ZXZ, __x, __y, __z, __phi, __theta, __psi) ) {
    _logger->log_warn("KatanaGotoThread", "Initiating goto failed, no IK solution found");
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_NO_SOLUTION;
    return;
  }

  // Run planner
  try {
    __ORCon->runPlanner();
  } catch (fawkes::Exception &e) {
    _logger->log_warn("KatanaGotoThread", "Planner failed (ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_UNSPECIFIC;
    return;
  }

  // Get trajectories and move katana along them
  __targetTraj = __ORCon->getTrajectory();
  try {
    moveKatana();
  } catch (/*KNI*/::Exception &e) {
    _logger->log_warn("KatanaGotoThread", "Moving along trajectory failed (ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
    _katana->freezeRobot();
    return;
  }


  // TODO: Check for finished motion
  // Can't use current version like in goto_thread, because target is not set in libkni!
  // can check endeffector position instead, or check last angle values from __targetTraj with katana-arm values

  _finished = true;
}



/** Update motors and fetch current encoder values.
 * @return true if succesful, false otherwise
 */
bool
KatanaGotoThreadOpenRAVE::updateMotorData()
{
  CKatBase *base = _katana->GetBase();
  short num_errors  = 0;

  // update motors
  while (1) {
    usleep(__poll_interval_usec);
    try {
      base->GetSCT()->arr[0].recvDAT(); // update sensor values
      base->recvMPS(); // get position for all motors
      base->recvGMS(); // get status flags for all motors
    } catch (/*KNI*/::Exception &e) {
      if (++num_errors <= 10) {
        _logger->log_warn("KatanaGotoThread", "Receiving MPS/GMS failed, retrying");
        continue;
      } else {
        _logger->log_warn("KatanaGotoThread", "Receiving MPS/GMS failed too often, aborting");
        _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
        return 0;
      }
    }
    break;
  }

  // fetch encoder values
  num_errors = 0;
  while (1) {
    usleep(__poll_interval_usec);
    try {
      __motorEncoders = _katana->getRobotEncoders(false); //fetch encoder values, param refreshEncoders=false
    } catch (/*KNI*/::Exception &e) {
      if (++num_errors <= 10) {
        _logger->log_warn("KatanaGotoThread", "Receiving encoder values failed, retrying");
        continue;
      } else {
        _logger->log_warn("KatanaGotoThread", "Receiving encoder values failed, aborting");
        _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
        return 0;
      }
    }
    break;
  }

  return 1;
}


/** Move katana arm along the current trajectory __targetTraj. */
void
KatanaGotoThreadOpenRAVE::moveKatana()
{
  for( __it=__targetTraj->begin(); __it!=__targetTraj->end(); ++__it) {
    for( unsigned int i=0; i<__it->size(); i++) {
      _katana->moveMotorTo(i, __it->at(i));
    }
  }
}

#endif