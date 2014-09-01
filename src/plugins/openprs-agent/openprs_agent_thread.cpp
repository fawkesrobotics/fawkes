
/***************************************************************************
 *  openprs_agent_thread.cpp -  OpenPRS agent thread
 *
 *  Created: Fri Aug 22 13:57:22 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include "openprs_agent_thread.h"

#include <logging/logger.h>
#include <plugins/openprs/utils/openprs_comm.h>
#include <utils/time/time.h>

#include <unistd.h>

using namespace fawkes;

/** @class OpenPRSAgentThread "openprs_agent_thread.h"
 * OpenPRS agent thread.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param oprs_mode whether to operate in console or graphical mode
 */
OpenPRSAgentThread::OpenPRSAgentThread(OpenPRSAspect::Mode oprs_mode)
  : Thread("OpenPRSAgentThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
    OpenPRSAspect("openprs-agent", oprs_mode)
{
  add_openprs_data_path(SRCDIR"/data");
}


/** Destructor. */
OpenPRSAgentThread::~OpenPRSAgentThread()
{
}


void
OpenPRSAgentThread::init()
{
  openprs.lock();
  openprs->transmit_command(openprs_kernel_name, "include \"agent-init.inc\"");
  openprs.unlock();
}

void
OpenPRSAgentThread::finalize()
{
}


void
OpenPRSAgentThread::loop()
{
  openprs.lock();
  fawkes::Time now, now_sys;
  clock->get_time(now);
  clock->get_systime(now_sys);
  openprs->send_message_f(openprs_kernel_name, "(fawkes-time %li %li %li %li)",
  			  now.get_sec(), now.get_usec(), now_sys.get_sec(), now_sys.get_usec());
  openprs.unlock();
}
