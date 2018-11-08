
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
    cfg_voice_ = config->get_string("/plugins/festival/voice");
  } catch (Exception &e) {
    cfg_voice_ = "";
  }
  try {
    cfg_extra_code_ = config->get_string("/plugins/festival/extra_code");
  } catch (Exception &e) {
    cfg_extra_code_ = "";
  }

  speechsynth_if_ = blackboard->open_for_writing<SpeechSynthInterface>("Festival");

  bbil_add_message_interface(speechsynth_if_);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

}


void FestivalSynthThread::once()
{
  festival_initialize(/* load init files */ 1, FESTIVAL_HEAP_SIZE);
  if (cfg_voice_ != "") {
    std::string voice_cmd = "(voice_" + cfg_voice_ + ")";
    if (! festival_eval_command(voice_cmd.c_str())) {
      logger->log_error(name(), "Failed to load voice %s", cfg_voice_.c_str());
    }
  }

  if (cfg_extra_code_ != "") {
    logger->log_debug(name(), "Executing extra code '%s'", cfg_extra_code_.c_str());
    if (! festival_eval_command(cfg_extra_code_.c_str())) {
      logger->log_error(name(), "Failed to execute extra code '%s'", cfg_extra_code_.c_str());
    }
  }

  say("Festival speech synth loaded");
}

void
FestivalSynthThread::finalize()
{
  festival_tidy_up();
  blackboard->unregister_listener(this);
  blackboard->close(speechsynth_if_);
}

void
FestivalSynthThread::loop()
{
  // wait for message(s) to arrive, could take a (little) while after the wakeup
  while ( speechsynth_if_->msgq_empty() ) {
    usleep(100);
  }

  // process messages, blocking
  if ( ! speechsynth_if_->msgq_empty() ) {
    if ( speechsynth_if_->msgq_first_is<SpeechSynthInterface::SayMessage>() ) {
      SpeechSynthInterface::SayMessage *msg = speechsynth_if_->msgq_first<SpeechSynthInterface::SayMessage>();
      speechsynth_if_->set_msgid(msg->id());
      say(msg->text());
    }

    speechsynth_if_->msgq_pop();
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

  speechsynth_if_->set_text(text);
  speechsynth_if_->set_final(false);
  speechsynth_if_->set_duration(duration);
  speechsynth_if_->write();

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

  speechsynth_if_->set_final(true);
  speechsynth_if_->write();
}
