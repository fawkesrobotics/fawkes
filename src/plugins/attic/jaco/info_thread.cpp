
/***************************************************************************
 *  info_thread.cpp - Kinova Jaco plugin information thread
 *
 *  Created: Thu Jun 13 19:14:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#include "info_thread.h"

#include "arm.h"
#include "types.h"

#include <interfaces/JacoInterface.h>

using namespace fawkes;

/** @class JacoInfoThread "info_thread.h"
 * Jaco Arm information thread.
 * This thread basically provides all informationen to interfaces.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param name thread name
 * @param arm pointer to jaco_arm_t struct, to be used in this thread
 */
JacoInfoThread::JacoInfoThread(const char *name, jaco_arm_t *arm)
: Thread(name, Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
	arm_ = arm;
}

/** Destructor. */
JacoInfoThread::~JacoInfoThread()
{
}

void
JacoInfoThread::init()
{
}

void
JacoInfoThread::finalize()
{
	arm_ = NULL;
}

void
JacoInfoThread::loop()
{
	if (arm_ == NULL || arm_->arm == NULL || arm_->iface == NULL)
		return;

	arm_->iface->set_connected(true);

	try {
		if (arm_->iface->is_final()) {
			arm_->arm->get_coords(cpos_);
			arm_->iface->set_x(cpos_.at(0));
			arm_->iface->set_y(cpos_.at(1));
			arm_->iface->set_z(cpos_.at(2));
			arm_->iface->set_euler1(cpos_.at(3));
			arm_->iface->set_euler2(cpos_.at(4));
			arm_->iface->set_euler3(cpos_.at(5));
		}

		arm_->arm->get_fingers(cpos_);
		arm_->iface->set_finger1(std::max(0.f, std::min(60.f, cpos_.at(0))));
		arm_->iface->set_finger2(std::max(0.f, std::min(60.f, cpos_.at(1))));
		arm_->iface->set_finger3(std::max(0.f, std::min(60.f, cpos_.at(2))));

		arm_->arm->get_joints(apos_);
		for (unsigned int i = 0; i < apos_.size(); i++) {
			arm_->iface->set_joints(i, apos_.at(i));
		}

	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "Could not get position and joint values. Er: %s", e.what());
	}
}
