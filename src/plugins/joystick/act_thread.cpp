
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
#include "sensor_thread.h"
#include "acquisition_thread.h"

#include <interfaces/JoystickInterface.h>

#include "force_feedback.h"

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
JoystickActThread::JoystickActThread(JoystickAcquisitionThread *aqt,
				     JoystickSensorThread *senst)
  : Thread("JoystickActThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  __aqt   = aqt;
  __senst = senst;
}


void
JoystickActThread::init()
{
  __joystick_if = __senst->joystick_interface();
  __msgproc = new MessageProcessor(__aqt, __joystick_if);
}


void
JoystickActThread::finalize()
{
  delete __msgproc;
}


void
JoystickActThread::loop()
{
  __msgproc->process();
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
                                                      JoystickInterface *joystick_if)
{
  __aqt = aqt;
  __joystick_if = joystick_if;
  __joystick_connected = false;
}

/** Process a single message.
 * @param msg message to process
 */
void
JoystickActThread::MessageProcessor::process_message(Message *msg)
{
  JoystickForceFeedback *ff = __aqt->ff();

  if (! ff)  return;

  if (dynamic_cast<JoystickInterface::StartRumbleMessage *>(msg) != NULL) {
    JoystickInterface::StartRumbleMessage *srm =
      dynamic_cast<JoystickInterface::StartRumbleMessage *>(msg);

    ff->rumble(srm->strong_magnitude(), srm->weak_magnitude(),
               (JoystickForceFeedback::Direction)srm->direction(),
               srm->length(), srm->delay());

    uint8_t e = __joystick_if->ff_effects() | JoystickInterface::JFF_RUMBLE;
    __joystick_if->set_ff_effects(e);
    __joystick_if->write();
    
  }
  else if (dynamic_cast<JoystickInterface::StopRumbleMessage *>(msg) != NULL)
  {
    ff->stop_rumble();
    uint8_t e = __joystick_if->ff_effects() & ~JoystickInterface::JFF_RUMBLE;
    __joystick_if->set_ff_effects(e);
    __joystick_if->write();

  }
  else if (dynamic_cast<JoystickInterface::StopAllMessage *>(msg) != NULL)
  {
    ff->stop_all();
    __joystick_if->set_ff_effects(0);
    __joystick_if->write();
  }
}


/** Process message currently in the queue. */
void
JoystickActThread::MessageProcessor::process()
{
  JoystickForceFeedback *ff = __aqt->ff();

  if (ff == NULL) {
    __joystick_if->msgq_flush();
    if (__joystick_connected) {
      __joystick_if->set_supported_ff_effects(0);
      __joystick_if->write();
      __joystick_connected = false;
    }
  } else  if (! __joystick_connected) {
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
    __joystick_if->set_supported_ff_effects(effects);    
    __joystick_if->write();
    __joystick_connected = true;
  }

  while (! __joystick_if->msgq_empty() ) {

    if (! __joystick_connected) {
      __joystick_if->msgq_flush();
      break;
    }

    process_message(__joystick_if->msgq_first());

    __joystick_if->msgq_pop();
  }
}
