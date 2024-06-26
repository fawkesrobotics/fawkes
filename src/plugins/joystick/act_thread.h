
/***************************************************************************
 *  act_thread.h - Joystick thread to execute force feedback
 *
 *  Created: Mon Feb 07 19:52:48 2011
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

#ifndef _PLUGINS_JOYSTICK_ACT_THREAD_H_
#define _PLUGINS_JOYSTICK_ACT_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>

namespace fawkes {
class JoystickInterface;
}

class JoystickAcquisitionThread;
class JoystickSensorThread;

class JoystickActThread : public fawkes::Thread,
                          public fawkes::BlockedTimingAspect,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::BlackBoardAspect
{
public:
	class MessageProcessor
	{
	public:
		MessageProcessor(JoystickAcquisitionThread *aqt, fawkes::JoystickInterface *joystick_if);

		void process();
		void process_message(fawkes::Message *msg);

	private:
		JoystickAcquisitionThread *aqt_;
		fawkes::JoystickInterface *joystick_if_;
		bool                       joystick_connected_;
	};

public:
	JoystickActThread(JoystickAcquisitionThread *aqt, JoystickSensorThread *senst);

	virtual void init();
	virtual void finalize();
	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::JoystickInterface *joystick_if_;

	JoystickAcquisitionThread *aqt_;
	JoystickSensorThread      *senst_;

	MessageProcessor *msgproc_;
};

#endif
