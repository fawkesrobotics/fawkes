
/***************************************************************************
 *  synth_thread.h - Flite synthesis thread
 *
 *  Created: Tue Oct 28 14:31:58 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_FLITE_SYNTH_THREAD_H_
#define __PLUGINS_FLITE_SYNTH_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <blackboard/interface_listener.h>

#include <string>

#include <flite/flite.h>

namespace fawkes {
  class SpeechSynthInterface;
}

class FliteSynthThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  FliteSynthThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  void say(const char *text);

  virtual bool bb_interface_message_received(fawkes::Interface *interface,
					     fawkes::Message *message) throw();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: /* methods */
  void  play_wave(cst_wave *wave);
  float get_duration(cst_wave *wave);

 private:
  fawkes::SpeechSynthInterface   *__speechsynth_if;

  std::string __cfg_soundcard;

  cst_voice  *__voice;
};


#endif
