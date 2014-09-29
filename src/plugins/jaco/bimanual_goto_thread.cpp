
/***************************************************************************
 *  bimaual_goto_thread.cpp - Jaco plugin movement thread for coordinated bimanual manipulation
 *
 *  Created: Mon Sep 29 23:17:12 2014
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

#include "bimanual_goto_thread.h"
#include "arm.h"

#include <interfaces/JacoInterface.h>
#include <utils/math/angle.h>
#include <core/threading/mutex.h>

#include <unistd.h>

using namespace fawkes;

/** @class JacoBimanualGotoThread "bimanual_goto_thread.h"
 * Jaco Arm movement thread.
 * This thread handles the movement of the arm.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
JacoBimanualGotoThread::JacoBimanualGotoThread(fawkes::jaco_arm_t *arm_l, fawkes::jaco_arm_t *arm_r)
  : Thread("JacoBimanualGotoThread", Thread::OPMODE_CONTINUOUS)
{
  __final = true;
}


/** Destructor. */
JacoBimanualGotoThread::~JacoBimanualGotoThread()
{
}

void
JacoBimanualGotoThread::init()
{
}

void
JacoBimanualGotoThread::finalize()
{
}

bool
JacoBimanualGotoThread::final()
{
  return true;
}


void
JacoBimanualGotoThread::move_gripper(float l_f1, float l_f2, float l_f3, float r_f1, float r_f2, float r_f3)
{
}

void
JacoBimanualGotoThread::loop()
{
  usleep(30e3);
}
