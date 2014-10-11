
/***************************************************************************
 *  arm_dummy.cpp - Class for a Kinova Jaco arm, simulating a dummy
 *
 *  Created: Mon Aug 04 19:58:22 2014
 *  Copyright  2014  Bahram Maleki-Fard
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

#include "arm_dummy.h"

#include <unistd.h>

#define READY_J0 (282.522400)
#define READY_J1 (154.470856)
#define READY_J2 (44.191490)
#define READY_J3 (230.081223)
#define READY_J4 (83.242500)
#define READY_J5 (77.796173)

#define RETRACT_J0 (270.527344)
#define RETRACT_J1 (150.205078)
#define RETRACT_J2 (25.042963)
#define RETRACT_J3 (267.451172)
#define RETRACT_J4 (5.800781)
#define RETRACT_J5 (99.448242)

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class JacoArmDummy <plugins/jaco/arm_dummy.h>
 * Class for simulating a dummy Kinova Jaco Arm.
 * Each command is accepted, simply storing its values and returning them
 * when a getter is called. This class does not operate any actual arm
 * (whether a real one nor even a simulated 3D model).
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor. */
JacoArmDummy::JacoArmDummy(const char *name)
{
  __name = name;
  __initialized = true;

  // initialize target vectors for READY and RETRACT positions
  __pos_ready.push_back(READY_J0);
  __pos_ready.push_back(READY_J1);
  __pos_ready.push_back(READY_J2);
  __pos_ready.push_back(READY_J3);
  __pos_ready.push_back(READY_J4);
  __pos_ready.push_back(READY_J5);
  __pos_retract.push_back(RETRACT_J0);
  __pos_retract.push_back(RETRACT_J1);
  __pos_retract.push_back(RETRACT_J2);
  __pos_retract.push_back(RETRACT_J3);
  __pos_retract.push_back(RETRACT_J4);
  __pos_retract.push_back(RETRACT_J5);

  // initialize position vectors
  __coords.assign(6, 0.f);
  __joints.assign(6, 0.f);
  __fingers.assign(3, 0.f);
}

/** Destructor. */
JacoArmDummy::~JacoArmDummy()
{
}

void
JacoArmDummy::initialize()
{
  goto_ready();
}


bool
JacoArmDummy::final()
{
  return true;
}

bool
JacoArmDummy::initialized()
{
  return __initialized;
}


void
JacoArmDummy::get_coords(std::vector<float> &to) const
{
  to = __coords;
}

void
JacoArmDummy::get_joints(std::vector<float> &to) const
{
  to = __joints;
}

void
JacoArmDummy::get_fingers(std::vector<float> &to) const
{
  to = __fingers;
}


void
JacoArmDummy::stop()
{
}

void
JacoArmDummy::push_joystick(unsigned int button)
{
}

void
JacoArmDummy::release_joystick()
{
  stop();
}


void
JacoArmDummy::goto_trajec(std::vector< std::vector<float> >* trajec, std::vector<float> &fingers)
{
  for( unsigned int i=0; i<trajec->size(); ++i ) {
    goto_joints(trajec->at(i), fingers);
    usleep(10e3);
  }
}

void
JacoArmDummy::goto_joints(std::vector<float> &joints, std::vector<float> &fingers, bool followup)
{
  if(followup)
    usleep(10e3);

  __joints = joints;
  __fingers = fingers;
}

void
JacoArmDummy::goto_coords(std::vector<float> &coords, std::vector<float> &fingers)
{
  __coords = coords;
  __fingers = fingers;
}

void
JacoArmDummy::goto_ready()
{
  goto_joints(__pos_ready, __fingers);
}

void
JacoArmDummy::goto_retract()
{
  goto_joints(__pos_retract, __fingers);
}


} // end of namespace fawkes
