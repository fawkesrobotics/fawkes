
/***************************************************************************
 *  synth_thread.cpp - Festival synthesis thread
 *
 *  Created: Tue Oct 28 14:34:14 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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
  __speechsynth_if = blackboard->open_for_writing<SpeechSynthInterface>("Festival");

  festival_initialize(/* load init files */ 1, FESTIVAL_HEAP_SIZE);

  bbil_add_message_interface(__speechsynth_if);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

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
  while ( ! __speechsynth_if->msgq_empty() ) {
    if ( __speechsynth_if->msgq_first_is<SpeechSynthInterface::SayMessage>() ) {
      SpeechSynthInterface::SayMessage *msg = __speechsynth_if->msgq_first<SpeechSynthInterface::SayMessage>();
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
  festival_say_text(text);
}
