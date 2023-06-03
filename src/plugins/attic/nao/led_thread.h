
/***************************************************************************
 *  led_thread.h - Provide NaoQi LEDs to Fawkes
 *
 *  Created: Thu Jun 30 19:49:00 2011
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

#ifndef _PLUGINS_NAO_LED_THREAD_H_
#define _PLUGINS_NAO_LED_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <core/utils/lock_multimap.h>
#include <plugins/nao/aspect/naoqi.h>

#include <string>
#include <utility>

namespace fawkes {
class LedInterface;
}
namespace AL {
class ALMemoryFastAccess;
}

class NaoQiLedThread : public fawkes::Thread,
                       public fawkes::LoggingAspect,
                       public fawkes::BlockedTimingAspect,
                       public fawkes::BlackBoardAspect,
                       public fawkes::ConfigurableAspect,
                       public fawkes::NaoQiAspect,
                       public fawkes::BlackBoardInterfaceListener
{
public:
	NaoQiLedThread();
	virtual ~NaoQiLedThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

protected:
	bool bb_interface_message_received(fawkes::Interface *interface,
	                                   fawkes::Message   *message) noexcept;

private:
	AL::ALPtr<AL::DCMProxy>           dcm_;
	AL::ALPtr<AL::ALMemoryProxy>      almem_;
	AL::ALPtr<AL::ALMemoryFastAccess> memfa_;

	std::vector<float> values_;

	typedef std::multimap<fawkes::LedInterface *, unsigned int> LedMemMap;
	LedMemMap                                                   memids_;

	typedef fawkes::LockMultiMap<fawkes::LedInterface *, std::string> LedMap;
	LedMap                                                            leds_;

	bool        cfg_verbose_face_;
	std::string subd_prefix_;
};

#endif
