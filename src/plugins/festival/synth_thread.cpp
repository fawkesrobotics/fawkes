
/***************************************************************************
 *  synth_thread.cpp - Festival synthesis thread
 *
 *  Created: Tue Oct 28 14:34:14 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include "synth_thread.h"

#include <interfaces/SpeechSynthInterface.h>
#include <utils/time/wait.h>

#include <festival/festival.h>

using namespace fawkes;

/** @class FestivalSynthThread "synth_thread.h"
 * Festival Synthesis Thread.
 * This thread synthesises audio for text-to-speech using Festival.
 * @author Tim Niemueller
 */


/** Constructor. */
FestivalSynthThread::FestivalSynthThread()
  : Thread("FestivalSynthThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("FestivalSynthThread")
{
}


void
FestivalSynthThread::init()
{
  try {
    __cfg_voice = config->get_string("/plugins/festival/voice");
  } catch (Exception &e) {
    __cfg_voice = "";
  }
  try {
    __cfg_extra_code = config->get_string("/plugins/festival/extra_code");
  } catch (Exception &e) {
    __cfg_extra_code = "";
  }

  __speechsynth_if = blackboard->open_for_writing<SpeechSynthInterface>("Festival");

  bbil_add_message_interface(__speechsynth_if);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

}


void FestivalSynthThread::once()
{
  festival_initialize(/* load init files */ 1, FESTIVAL_HEAP_SIZE);
  if (__cfg_voice != "") {
    std::string voice_cmd = "(voice_" + __cfg_voice + ")";
    if (! festival_eval_command(voice_cmd.c_str())) {
      logger->log_error(name(), "Failed to load voice %s", __cfg_voice.c_str());
    }
  }

  if (__cfg_extra_code != "") {
    logger->log_debug(name(), "Executing extra code '%s'", __cfg_extra_code.c_str());
    if (! festival_eval_command(__cfg_extra_code.c_str())) {
      logger->log_error(name(), "Failed to execute extra code '%s'", __cfg_extra_code.c_str());
    }
  }

  say("Festival speech synth loaded");
}

void
FestivalSynthThread::finalize()
{
  festival_tidy_up();
  blackboard->unregister_listener(this);
  blackboard->close(__speechsynth_if);
}

void
FestivalSynthThread::loop()
{
  // wait for message(s) to arrive, could take a (little) while after the wakeup
  while ( __speechsynth_if->msgq_empty() ) {
    usleep(100);
  }

  // process messages, blocking
  if ( ! __speechsynth_if->msgq_empty() ) {
    if ( __speechsynth_if->msgq_first_is<SpeechSynthInterface::SayMessage>() ) {
      SpeechSynthInterface::SayMessage *msg = __speechsynth_if->msgq_first<SpeechSynthInterface::SayMessage>();
      __speechsynth_if->set_msgid(msg->id());
      say(msg->text());
    }

    __speechsynth_if->msgq_pop();
  }
}


bool
FestivalSynthThread::bb_interface_message_received(Interface *interface,
						Message *message) throw()
{
  wakeup();
  return true;
}


/** Say something.
 * @param text text to synthesize and speak.
 */
void
FestivalSynthThread::say(const char *text)
{
  EST_Wave wave;
  festival_text_to_wave(text, wave);

  float duration = (float)wave.num_samples() / (float)wave.sample_rate();

  __speechsynth_if->set_text(text);
  __speechsynth_if->set_final(false);
  __speechsynth_if->set_duration(duration);
  __speechsynth_if->write();

  Time start;
  clock->get_systime(start);

  EST_Option al;
  play_wave(wave, al);

  // compensate for data in buffer that still needs to be player, should be
  // replaced with a call that actually determines the size of the buffer...
  Time now;
  clock->get_systime(now);
  float remaining = duration - (now - &start);
  if (remaining > 0) {
    Time waittime(remaining);
    waittime.wait_systime();
  }

  __speechsynth_if->set_final(true);
  __speechsynth_if->write();
}
