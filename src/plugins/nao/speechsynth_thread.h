
/***************************************************************************
 *  speechsynth_thread.h - Provide NaoQi speech synthesis to Fawkes
 *
 *  Created: Tue Jun 21 17:27:23 2011
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

#ifndef __PLUGINS_NAO_SPEECHSYNTH_THREAD_H_
#define __PLUGINS_NAO_SPEECHSYNTH_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/nao/aspect/naoqi.h>

#include <vector>

namespace AL {
  class ALTextToSpeechProxy;
}
namespace fawkes {
  class SpeechSynthInterface;
}

class NaoQiSpeechSynthThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::NaoQiAspect
{
 public:
  NaoQiSpeechSynthThread();
  virtual ~NaoQiSpeechSynthThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void stop_speech();
  void process_messages();
  void say(const char *text);

 private:
  AL::ALPtr<AL::ALTextToSpeechProxy> __altts;

  fawkes::SpeechSynthInterface *__speechsynth_if;

  int __tts_task_id;
};

#endif
