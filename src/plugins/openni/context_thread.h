
/***************************************************************************
 *  context_thread.h - OpenNI context providing thread
 *
 *  Created: Sat Feb 26 15:23:16 2011
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

#ifndef _PLUGINS_OPENNI_CONTEXT_THREAD_H_
#define _PLUGINS_OPENNI_CONTEXT_THREAD_H_

#include <aspect/aspect_provider.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <plugins/openni/aspect/openni_inifin.h>
#include <sys/types.h>
#include <utils/time/time.h>

#include <map>
#include <string>

namespace xn {
class Context;
class Device;
} // namespace xn

class OpenNiContextThread : public fawkes::Thread,
                            public fawkes::BlockedTimingAspect,
                            public fawkes::LoggingAspect,
                            public fawkes::ConfigurableAspect,
                            public fawkes::ClockAspect,
                            public fawkes::AspectProviderAspect
{
public:
	OpenNiContextThread();
	virtual ~OpenNiContextThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	void print_nodes();
	void verify_active();
	void start_sensor_server();
	void stop_sensor_server();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::LockPtr<xn::Context> openni_;
	fawkes::OpenNiAspectIniFin   openni_aspect_inifin_;

	bool        cfg_run_sensor_server_;
	std::string cfg_sensor_bin_;
	pid_t       sensor_server_pid_;
	xn::Device *device_;

	int last_refcount_;

	fawkes::Time check_last_;
	fawkes::Time check_now_;

	unsigned int device_no_data_loops_;

	std::map<std::string, unsigned int> dead_loops_;
};

#endif
