
/***************************************************************************
 *  motion_standup_task.cpp - Task for making the robot stand up
 *
 *  Created: Mon Jan 19 14:18:40 2009
 *  Copyright  2009-2011  Tim Niemueller [www.niemueller.de]
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

#include "motion_standup_task.h"

#include <core/exceptions/system.h>

#include <cstring>
#include <cstdlib>
#include <string>

using namespace AL;
using namespace fawkes;

/** @class NaoQiMotionStandupTask "motion_standup_task.h"
 * NaoQi standup task.
 * This task can be used to make the robot standup ina non-blocking way. It will
 * use (blocking) ALMotion calls to execute the move. Note that ALMotion should
 * not be used otherwise.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param almotion ALMotion proxy
 * @param from_pos position from which to standup
 * @param accel_x current accelerometer value
 * @param accel_y current accelerometer value
 * @param accel_z current accelerometer value
 */
NaoQiMotionStandupTask::NaoQiMotionStandupTask(AL::ALPtr<AL::ALMotionProxy> almotion,
					       fawkes::HumanoidMotionInterface::StandupEnum from_pos,
					       float accel_x, float accel_y,
					       float accel_z)
{
  __almotion  = almotion;
  __from_pos  = from_pos;
  __accel_x   = accel_x;
  __accel_y   = accel_y;
  __accel_z   = accel_z;

  // ALTask variable to cause auto-destruct when done
  fAutoDelete = true;
}


/** Destructor. */
NaoQiMotionStandupTask::~NaoQiMotionStandupTask()
{
}


#define _t(x) times.arrayPush(x)
#define _tc() times.clear()
#define _tp() all_times.arrayPush(times)

#define _a(x) angles.arrayPush(x)
#define _ac() angles.clear()
#define _ap() all_angles.arrayPush(angles)

void
NaoQiMotionStandupTask::goto_start_pos()
{
  ALValue joints, angles, times, all_angles, all_times;

  joints.arrayPush("HeadPitch");      joints.arrayPush("HeadYaw");
  joints.arrayPush("LAnklePitch");    joints.arrayPush("LAnkleRoll");
  joints.arrayPush("LElbowRoll");     joints.arrayPush("LElbowYaw");
  joints.arrayPush("LHipPitch");      joints.arrayPush("LHipRoll");
  joints.arrayPush("LHipYawPitch");   joints.arrayPush("LKneePitch");
  joints.arrayPush("LShoulderPitch"); joints.arrayPush("LShoulderRoll");
  joints.arrayPush("RAnklePitch");    joints.arrayPush("RAnkleRoll");
  joints.arrayPush("RElbowRoll");     joints.arrayPush("RElbowYaw");
  joints.arrayPush("RHipPitch");      joints.arrayPush("RHipRoll");
  joints.arrayPush("RKneePitch");     joints.arrayPush("RShoulderPitch");
  joints.arrayPush("RShoulderRoll");

  times = ALValue::array(1.9, 2.9);

  for (unsigned int i = 0; i < joints.getSize(); ++i) {
    all_times.arrayPush(times);
  }

  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(3.37175e-007); _a(3.37175e-007); _ap();
  _ac(); _a(0.523599);     _a(0.523599);     _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(1.5708);       _a(1.5708);       _ap();
  _ac(); _a(0.523599);     _a(0.523599);     _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(0);            _a(0);            _ap();
  _ac(); _a(-1.5708);      _a(-1.5708);      _ap();


  bool is_absolute = true;
  __almotion->angleInterpolation(joints, all_angles, all_times, is_absolute);
}

void
NaoQiMotionStandupTask::standup_from_back()
{

  ALValue joints, angles, times, all_angles, all_times;

  goto_start_pos();

  joints.arrayPush("HeadPitch");      joints.arrayPush("HeadYaw");
  joints.arrayPush("LAnklePitch");    joints.arrayPush("LAnkleRoll");
  joints.arrayPush("LElbowRoll");     joints.arrayPush("LElbowYaw");
  joints.arrayPush("LHipPitch");      joints.arrayPush("LHipRoll");
  joints.arrayPush("LHipYawPitch");   joints.arrayPush("LKneePitch");
  joints.arrayPush("LShoulderPitch"); joints.arrayPush("LShoulderRoll");
  joints.arrayPush("RAnklePitch");    joints.arrayPush("RAnkleRoll");
  joints.arrayPush("RElbowRoll");     joints.arrayPush("RElbowYaw");
  joints.arrayPush("RHipPitch");      joints.arrayPush("RHipRoll");
  joints.arrayPush("RHipYawPitch");   joints.arrayPush("RKneePitch");
  joints.arrayPush("RShoulderPitch"); joints.arrayPush("RShoulderRoll");

  _ac(); _a(0); _a(0); _a(-0.785398); _a(0); _a(0.349066); _a(0.328232); _a(0.331288); _a(0.378859); _a(0.378859); _a(0.378859); _a(0.279253); _ap();
  _ac(); _a(3.37175e-007); _a(-4.76838e-007); _a(6.7435e-007); _a(3.37177e-007); _a(3.37175e-007); _a(-0.00743961); _a(-0.0107031); _a(-0.00940132); _a(-0.00940132); _a(-0.00940132); _ap();
  _ac(); _a(0); _a(0.244346); _a(0.244346); _a(0.244346); _a(0.785398); _a(-0.570651); _a(-1.22173); _a(-1.22173); _a(-1.22173); _a(-1.22173); _a(-0.174533); _ap();
  _ac(); _a(0); _a(0); _a(0); _a(0); _a(0); _a(-0.395731); _a(-0.103788); _a(0.118105); _a(0.0872665); _a(0); _a(0); _ap();
  _ac(); _a(0); _a(0); _a(-1.65806); _a(-0.698132); _a(0); _a(-0.488692); _a(-0.823719); _a(-0.805354); _a(-0.805354); _a(-1.13446); _a(-1.25664); _ap();
  _ac(); _a(0); _a(0.15708); _a(0.0872665); _a(0.0872665); _a(0.0872665); _a(0.0829527); _a(0.0944466); _a(0.0830765); _a(0.0830765); _a(-1.25664); _a(-1.23918); _ap();
  _ac(); _a(0); _a(-0.174533); _a(-0.174533); _a(-1.5708); _a(-1.5708); _a(-0.857056); _a(0.385512); _a(-0.855211); _a(-0.835988); _a(-0.872665); _a(-0.174533); _ap();
  _ac(); _a(0); _a(1.56923e-007); _a(1.56923e-007); _a(1.56923e-007); _a(0.541052); _a(0.154976); _a(-0.291418); _a(0.191986); _a(0.366519); _a(0); _a(-0.010472); _ap();
  _ac(); _a(0); _a(0); _a(-4.76838e-007); _a(-0.663225); _a(-0.499093); _a(-0.858972); _a(-0.402255); _a(-0.402255); _a(0); _a(0); _ap();
  _ac(); _a(0); _a(1.67552); _a(1.67552); _a(1.67552); _a(1.67552); _a(2.20124); _a(1.77479); _a(2.20585); _a(2.20585); _a(2.0944); _a(0.349066); _ap();
  _ac(); _a(0); _a(2.0944); _a(2.0944); _a(2.0944); _a(2.0944); _a(0.698132); _a(0.740735); _a(0.733209); _a(0.733209); _a(1.71042); _a(1.8326); _ap();
  _ac(); _a(1.5708); _a(0.802851); _a(0.471239); _a(0.366519); _a(0); _a(1.0472); _a(0.498508); _a(0.498508); _a(0.498508); _a(0.0349066); _a(0.191986); _ap();
  _ac(); _a(0); _a(0.244346); _a(0.244346); _a(0.244346); _a(0.785398); _a(0.785398); _a(0.69115); _a(0.403171); _a(-0.579449); _a(-1.22173); _a(-0.174533); _ap();
  _ac(); _a(0); _a(8.63852e-008); _a(8.63852e-008); _a(8.63852e-008); _a(8.63852e-008); _a(0.00928971); _a(-0.129154); _a(0.679603); _a(0.277507); _a(0); _a(0); _ap();
  _ac(); _a(0); _a(0); _a(1.65806); _a(0.698132); _a(0); _a(0.0720474); _a(0.0581865); _a(0.453786); _a(0.559952); _a(1.13446); _a(1.25664); _ap();
  _ac(); _a(0); _a(-0.15708); _a(-0.0872665); _a(-0.0872665); _a(-0.0872665); _a(-0.0807962); _a(-0.0824083); _a(0.000615569); _a(0.000615569); _a(1.25664); _a(1.23918); _ap();
  _ac(); _a(0); _a(-0.174533); _a(-0.174533); _a(-1.5708); _a(-1.5708); _a(-1.52484); _a(-1.55965); _a(-0.905826); _a(-0.905826); _a(-0.872665); _a(-0.174533); _ap();
  _ac(); _a(0); _a(0); _a(0); _a(0); _a(-0.541052); _a(-0.558422); _a(-0.566003); _a(-0.296706); _a(-0.0174533); _a(0); _a(0.010472); _ap();
  _ac(); _a(-0.499093); _a(-0.858972); _a(-0.402255); _a(-0.402255); _a(-0.402255); _ap();
  _ac(); _a(0); _a(1.67552); _a(1.67552); _a(1.67552); _a(1.67552); _a(1.22173); _a(1.08036); _a(0.876155); _a(1.76278); _a(2.0944); _a(0.349066); _ap();
  _ac(); _a(0); _a(2.0944); _a(2.0944); _a(2.0944); _a(2.0944); _a(2.0944); _a(1.77434); _a(0.891306); _a(0.891306); _a(1.71042); _a(1.8326); _ap();
  _ac(); _a(-1.5708); _a(-0.802851); _a(-0.471239); _a(-0.366519); _a(0); _a(-0.575959); _a(-0.277696); _a(-0.872665); _a(-0.680678); _a(-0.0349066); _a(-0.191986); _ap();

  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();
  _tc(); _t(0.9); _t(1.9); _t(2.7); _t(3.4); _t(3.9); _t(4.9); _t(5.8); _t(6.8); _t(7.3); _t(8.4); _t(9.4); _tp();

  bool is_absolute = true;
  __almotion->angleInterpolation(joints, all_angles, all_times, is_absolute);

}

void
NaoQiMotionStandupTask::standup_from_front()
{
  ALValue joints, angles, times, all_angles, all_times;

  goto_start_pos();

  joints.arrayPush("HeadPitch");
  joints.arrayPush("LAnklePitch");    joints.arrayPush("LAnkleRoll");
  joints.arrayPush("LElbowRoll");     joints.arrayPush("LElbowYaw");
  joints.arrayPush("LHipPitch");      joints.arrayPush("LHipRoll");
  joints.arrayPush("LHipYawPitch");   joints.arrayPush("LKneePitch");
  joints.arrayPush("LShoulderPitch"); joints.arrayPush("LShoulderRoll");
  joints.arrayPush("RAnklePitch");    joints.arrayPush("RAnkleRoll");
  joints.arrayPush("RElbowRoll");     joints.arrayPush("RElbowYaw");
  joints.arrayPush("RHipPitch");      joints.arrayPush("RHipRoll");
  joints.arrayPush("RHipYawPitch");   joints.arrayPush("RKneePitch");
  joints.arrayPush("RShoulderPitch"); joints.arrayPush("RShoulderRoll");

  _ac(); _a(-0.575959); _a(0); _a(-0.349066); _a(-0.488692); _a(0); _a(0.279253); _ap();
  _ac(); _a(-1.13446); _a(-1.13446); _a(-0.783653); _a(0.0872665); _a(-0.312414); _a(-0.715585); _a(-1.0472); _a(-0.174533); _ap();
  _ac(); _a(0); _a(0); _a(-0.680678); _a(-0.555015); _a(-0.296706); _a(-0.10472); _a(0); _a(0); _ap();
  _ac(); _a(0); _a(-0.610865); _a(-1.65806); _a(-0.139626); _a(-0.715585); _a(-1.29154); _a(-1.39626); _a(-1.25664); _ap();
  _ac(); _a(-1.5708); _a(-1.5708); _a(-1.5708); _a(-1.5708); _a(-0.244346); _a(-0.925025); _a(-1.5708); _a(-1.23918); _ap();
  _ac(); _a(0); _a(-1.5708); _a(-1.5708); _a(-1.5708); _a(-1.5708); _a(-1.06989); _a(-1.0472); _a(-0.174533); _ap();
  _ac(); _a(1.56923e-007); _a(1.56923e-007); _a(1.56923e-007); _a(1.56923e-007); _a(0.0872665); _a(0.10472); _a(-0.010472); _a(-0.010472); _ap();
  _ac(); _a(0); _a(0); _a(-0.872665); _a(-0.872665); _a(-0.965167); _a(-0.785398); _a(0); _a(0); _ap();
  _ac(); _a(2.0944); _a(2.0944); _a(1.0472); _a(1.01229); _a(2.15548); _a(2.16421); _a(2.0944); _a(0.349066); _ap();
  _ac(); _a(-1.5708); _a(-0.872665); _a(-0.174533); _a(0); _a(0.610865); _a(1.11701); _a(1.62316); _a(1.8326); _ap();
  _ac(); _a(0); _a(0); _a(0); _a(0); _a(0.0349066); _a(0.1309); _a(0.174533); _a(0.191986); _ap();
  _ac(); _a(-1.13446); _a(-1.13446); _a(-0.783653); _a(0.0872665); _a(-0.312414); _a(-0.715585); _a(-1.0472); _a(-0.174533); _ap();
  _ac(); _a(8.63852e-008); _a(8.63852e-008); _a(0.680678); _a(0.555015); _a(0.296706); _a(0.10472); _a(0); _a(0); _ap();
  _ac(); _a(0); _a(0.610865); _a(1.65806); _a(0.139626); _a(0.715585); _a(1.29154); _a(1.39626); _a(1.25664); _ap();
  _ac(); _a(1.5708); _a(1.5708); _a(1.5708); _a(1.5708); _a(0.244346); _a(0.925025); _a(1.5708); _a(1.23918); _ap();
  _ac(); _a(1.44878e-007); _a(-1.5708); _a(-1.5708); _a(-1.5708); _a(-1.5708); _a(-1.06989); _a(-1.0472); _a(-0.174533); _ap();
  _ac(); _a(0); _a(0); _a(0); _a(0); _a(-0.0872665); _a(-0.10472); _a(0.010472); _a(0.010472); _ap();
  _ac(); _a(8.54618e-008); _a(9.7389e-008); _a(-0.872665); _ap();
  _ac(); _a(2.0944); _a(2.0944); _a(1.0472); _a(1.01229); _a(2.15548); _a(2.16421); _a(2.0944); _a(0.349066); _ap();
  _ac(); _a(-1.5708); _a(-0.872665); _a(-0.174533); _a(0); _a(0.610865); _a(1.11701); _a(1.62316); _a(1.8326); _ap();
  _ac(); _a(0); _a(0); _a(0); _a(0); _a(-0.0349066); _a(-0.1309); _a(-0.174533); _a(-0.191986); _ap();

  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(5.2); _t(6.2); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();
  _tc(); _t(1.4); _t(2.4); _t(3.7); _t(4.4); _t(5.2); _t(6.2); _t(7.4); _t(8.4); _tp();

  bool is_absolute = true;
  __almotion->angleInterpolation(joints, all_angles, all_times, is_absolute);
}

/** Run the standup. */
void
NaoQiMotionStandupTask::run()
{
  if ( __from_pos == fawkes::HumanoidMotionInterface::STANDUP_BACK ) {
    //__allogger->info("NaoQiMotionStandupTask", "Explicit standup back");
    standup_from_back();
  } else if (__from_pos == fawkes::HumanoidMotionInterface::STANDUP_FRONT ) {
    //__allogger->info("NaoQiMotionStandupTask", "Explicit standup front");
    standup_from_front();
  } else {
    if ( __accel_x > 0.8 ) {
      //__allogger->info("NaoQiMotionStandupTask", "Standup from front (detected)");
      standup_from_front();
    } else if ( __accel_x < -0.8 ) {
      //__allogger->info("NaoQiMotionStandupTask", "Standup from back (detected)");
      standup_from_back();
    } else {
      //__allogger->error("NaoQiMotionStandupTask",
      // "NaoQiMotionStandupTask: Does not seem that I'm lying on the ground, "
      //		"not standing up until you tell me from where");
    }
  }
}
