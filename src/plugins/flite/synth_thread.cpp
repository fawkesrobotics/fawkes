
/***************************************************************************
 *  synth_thread.cpp - Flite synthesis thread
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

#include <alsa/asoundlib.h>
#include <interfaces/SpeechSynthInterface.h>
#include <utils/time/wait.h>

#include <cmath>

using namespace fawkes;

extern "C" {
extern cst_voice *register_cmu_us_kal(const char *voxdir);
extern void       unregister_cmu_us_kal(cst_voice *voice);
}

/** @class FliteSynthThread "synth_thread.h"
 * Flite Synthesis Thread.
 * This thread synthesises audio for text-to-speech using Flite.
 * @author Tim Niemueller
 */

/** Constructor. */
FliteSynthThread::FliteSynthThread()
: Thread("FliteSynthThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("FliteSynthThread")
{
}

void
FliteSynthThread::init()
{
	speechsynth_if_ = blackboard->open_for_writing<SpeechSynthInterface>("Flite");
	voice_          = register_cmu_us_kal(NULL);

	cfg_soundcard_ = config->get_string("/flite/soundcard");

	bbil_add_message_interface(speechsynth_if_);
	blackboard->register_listener(this);

	say("Speech synth loaded");
}

void
FliteSynthThread::finalize()
{
	unregister_cmu_us_kal(voice_);
	blackboard->unregister_listener(this);
	blackboard->close(speechsynth_if_);
}

void
FliteSynthThread::loop()
{
	// wait for message(s) to arrive, could take a (little) while after the wakeup
	while (speechsynth_if_->msgq_empty()) {
		usleep(100);
	}

	// process message, blocking
	// only one at a time, loop() will be run as many times as wakeup() was called
	if (!speechsynth_if_->msgq_empty()) {
		if (speechsynth_if_->msgq_first_is<SpeechSynthInterface::SayMessage>()) {
			SpeechSynthInterface::SayMessage *msg =
			  speechsynth_if_->msgq_first<SpeechSynthInterface::SayMessage>();
			speechsynth_if_->set_msgid(msg->id());
			say(msg->text());
		}

		speechsynth_if_->msgq_pop();
	}
}

bool
FliteSynthThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	wakeup();
	return true;
}

/** Say something.
 * @param text text to synthesize and speak.
 */
void
FliteSynthThread::say(const char *text)
{
	cst_wave *wave = flite_text_to_wave(text, voice_);
	cst_wave_save_riff(wave, "/tmp/test.wav");

	speechsynth_if_->set_text(text);
	speechsynth_if_->set_final(false);
	speechsynth_if_->set_duration(get_duration(wave));
	speechsynth_if_->write();

	play_wave(wave);
	delete_wave(wave);

	speechsynth_if_->set_final(true);
	speechsynth_if_->write();
}

float
FliteSynthThread::get_duration(cst_wave *wave)
{
	return (float)cst_wave_num_samples(wave) / (float)cst_wave_sample_rate(wave);
}

/** Play a Flite wave to the default ALSA audio out.
 * @param wave the wave form to play
 */
void
FliteSynthThread::play_wave(cst_wave *wave)
{
	snd_pcm_t *pcm;
	float      duration = get_duration(wave);
	int        err;
	if ((err = snd_pcm_open(&pcm, cfg_soundcard_.c_str(), SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
		throw Exception("Failed to open PCM: %s", snd_strerror(err));
	}
	snd_pcm_nonblock(pcm, 0);
	if ((err = snd_pcm_set_params(pcm,
	                              SND_PCM_FORMAT_S16_LE,
	                              SND_PCM_ACCESS_RW_INTERLEAVED,
	                              cst_wave_num_channels(wave),
	                              cst_wave_sample_rate(wave),
	                              1,
	                              (unsigned int)roundf(duration * 1000000.)))
	    < 0) {
		throw Exception("Playback to set params: %s", snd_strerror(err));
	}

	snd_pcm_sframes_t frames;
	frames = snd_pcm_writei(pcm, cst_wave_samples(wave), cst_wave_num_samples(wave));
	if (frames < 0) {
		logger->log_warn(name(), "snd_pcm_writei failed (frames < 0)");
		frames = snd_pcm_recover(pcm, frames, 0);
	}
	if (frames < 0) {
		logger->log_warn(name(), "snd_pcm_writei failed: %s", snd_strerror(err));
	} else if (frames < (long)cst_wave_num_samples(wave)) {
		logger->log_warn(name(),
		                 "Short write (expected %li, wrote %li)",
		                 (long)cst_wave_num_samples(wave),
		                 frames);
	}

	TimeWait::wait_systime((unsigned int)roundf(duration * 1000000.f));
	snd_pcm_close(pcm);
}
