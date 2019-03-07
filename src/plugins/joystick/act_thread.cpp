
/***************************************************************************
 *  act_thread.cpp - Joystick thread to execute force feedback
 *
 *  Created: Mon Feb 07 21:29:22 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "act_thread.h"

#include "acquisition_thread.h"
#include "force_feedback.h"
#include "sensor_thread.h"

#include <interfaces/JoystickInterface.h>

using namespace fawkes;

/** @class JoystickActThread "act_thread.h"
 * Joystick force feedback actuation thread.
 * This thread integrates into the Fawkes main loop at the act hook and
 * executes force feedback commands.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param aqt JoystickAcquisitionThread to get data from
 * @param senst sensor thread to share joystick interface with
 */
JoystickActThread::JoystickActThread(JoystickAcquisitionThread *aqt, JoystickSensorThread *senst)
: Thread("JoystickActThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
	aqt_   = aqt;
	senst_ = senst;
}

void
JoystickActThread::init()
{
	joystick_if_ = senst_->joystick_interface();
	msgproc_     = new MessageProcessor(aqt_, joystick_if_);
}

void
JoystickActThread::finalize()
{
	delete msgproc_;
}

void
JoystickActThread::loop()
{
	msgproc_->process();
}

/** @class JoystickActThread::MessageProcessor "act_thread.h"
 * Process incoming messages.
 * Internal utility class.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param aqt acqusition thread to intsruct
 * @param joystick_if interface to listen on for messages
 */
JoystickActThread::MessageProcessor::MessageProcessor(JoystickAcquisitionThread *aqt,
                                                      JoystickInterface *        joystick_if)
{
	aqt_                = aqt;
	joystick_if_        = joystick_if;
	joystick_connected_ = false;
}

/** Process a single message.
 * @param msg message to process
 */
void
JoystickActThread::MessageProcessor::process_message(Message *msg)
{
	JoystickForceFeedback *ff = aqt_->ff();

	if (!ff)
		return;

	if (dynamic_cast<JoystickInterface::StartRumbleMessage *>(msg) != NULL) {
		JoystickInterface::StartRumbleMessage *srm =
		  dynamic_cast<JoystickInterface::StartRumbleMessage *>(msg);

		ff->rumble(srm->strong_magnitude(),
		           srm->weak_magnitude(),
		           (JoystickForceFeedback::Direction)srm->direction(),
		           srm->length(),
		           srm->delay());

		uint8_t e = joystick_if_->ff_effects() | JoystickInterface::JFF_RUMBLE;
		joystick_if_->set_ff_effects(e);
		joystick_if_->write();

	} else if (dynamic_cast<JoystickInterface::StopRumbleMessage *>(msg) != NULL) {
		ff->stop_rumble();
		uint8_t e = joystick_if_->ff_effects() & ~JoystickInterface::JFF_RUMBLE;
		joystick_if_->set_ff_effects(e);
		joystick_if_->write();

	} else if (dynamic_cast<JoystickInterface::StopAllMessage *>(msg) != NULL) {
		ff->stop_all();
		joystick_if_->set_ff_effects(0);
		joystick_if_->write();
	}
}

/** Process message currently in the queue. */
void
JoystickActThread::MessageProcessor::process()
{
	JoystickForceFeedback *ff = aqt_->ff();

	if (ff == NULL) {
		joystick_if_->msgq_flush();
		if (joystick_connected_) {
			joystick_if_->set_supported_ff_effects(0);
			joystick_if_->write();
			joystick_connected_ = false;
		}
	} else if (!joystick_connected_) {
		uint8_t effects = 0;
		if (ff->can_rumble()) {
			effects |= JoystickInterface::JFF_RUMBLE;
		}
		if (ff->can_periodic()) {
			effects |= JoystickInterface::JFF_PERIODIC;
		}
		if (ff->can_ramp()) {
			effects |= JoystickInterface::JFF_RAMP;
		}
		if (ff->can_spring()) {
			effects |= JoystickInterface::JFF_SPRING;
		}
		if (ff->can_friction()) {
			effects |= JoystickInterface::JFF_FRICTION;
		}
		if (ff->can_damper()) {
			effects |= JoystickInterface::JFF_DAMPER;
		}
		if (ff->can_inertia()) {
			effects |= JoystickInterface::JFF_INERTIA;
		}
		if (ff->can_constant()) {
			effects |= JoystickInterface::JFF_CONSTANT;
		}
		joystick_if_->set_supported_ff_effects(effects);
		joystick_if_->write();
		joystick_connected_ = true;
	}

	while (!joystick_if_->msgq_empty()) {
		if (!joystick_connected_) {
			joystick_if_->msgq_flush();
			break;
		}

		process_message(joystick_if_->msgq_first());

		joystick_if_->msgq_pop();
	}
}
