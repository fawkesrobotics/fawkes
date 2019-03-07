
/***************************************************************************
 *  speechsynth_thread.cpp - Provide NaoQi speech synthesis to Fawkes
 *
 *  Created: Tue Jun 21 17:32:14 2011
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

#include "speechsynth_thread.h"

#include <alcore/alerror.h>
#include <alproxies/allauncherproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <interfaces/SpeechSynthInterface.h>

using namespace fawkes;

/** @class NaoQiSpeechSynthThread "motion_thread.h"
 * Thread to provide NaoQi motions to Fawkes.
 * This thread holds an ALMotion proxy and provides its capabilities via
 * the blackboard to other Fawkes threads.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
NaoQiSpeechSynthThread::NaoQiSpeechSynthThread()
: Thread("NaoQiSpeechSynthThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

/** Destructor. */
NaoQiSpeechSynthThread::~NaoQiSpeechSynthThread()
{
}

void
NaoQiSpeechSynthThread::init()
{
	tts_task_id_ = -1;

	// Is ALTextToSpeech available?
	try {
		AL::ALPtr<AL::ALLauncherProxy> launcher(new AL::ALLauncherProxy(naoqi_broker));
		bool                           is_tts_available = launcher->isModulePresent("ALTextToSpeech");

		if (!is_tts_available) {
			throw Exception("NaoQi ALTextToSpeech is not available");
		}
	} catch (AL::ALError &e) {
		throw Exception("Checking ALTextToSpeech aliveness failed: %s", e.toString().c_str());
	}

	altts_ = AL::ALPtr<AL::ALTextToSpeechProxy>(new AL::ALTextToSpeechProxy(naoqi_broker));

	speechsynth_if_ = blackboard->open_for_writing<SpeechSynthInterface>("NaoQi TTS");
}

void
NaoQiSpeechSynthThread::finalize()
{
	stop_speech();

	blackboard->close(speechsynth_if_);
	speechsynth_if_ = NULL;

	altts_.reset();
}

/** Stop currently running speech synthesis. */
void
NaoQiSpeechSynthThread::stop_speech()
{
	if (tts_task_id_ != -1) {
		if (altts_->isRunning(tts_task_id_)) {
			altts_->stop(tts_task_id_);
		}
		tts_task_id_ = -1;
	}
}

void
NaoQiSpeechSynthThread::say(const char *text)
{
	tts_task_id_ = altts_->say(text);
}

void
NaoQiSpeechSynthThread::loop()
{
	bool working = (tts_task_id_ != -1) && altts_->isRunning(tts_task_id_);
	if (!working) {
		process_messages();
	}
	speechsynth_if_->set_final(!working);
	speechsynth_if_->write();
}

/** Process incoming BB messages. */
void
NaoQiSpeechSynthThread::process_messages()
{
	// process bb messages
	if (!speechsynth_if_->msgq_empty()) {
		if (SpeechSynthInterface::SayMessage *msg = speechsynth_if_->msgq_first_safe(msg)) {
			say(msg->text());
			speechsynth_if_->set_msgid(msg->id());
		}

		speechsynth_if_->msgq_pop();
	}
}
