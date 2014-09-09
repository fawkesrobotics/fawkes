
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
 * @param gdb_delay whether to instruct mod_utils to wait for a while for
 * a gdb connection or not.
 */
OpenPRSAgentThread::OpenPRSAgentThread(OpenPRSAspect::Mode oprs_mode, bool gdb_delay)
  : Thread("OpenPRSAgentThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
    OpenPRSAspect("openprs-agent", oprs_mode)
{
  add_openprs_data_path(SRCDIR"/data");
  set_openprs_gdb_delay(gdb_delay);
}


/** Destructor. */
OpenPRSAgentThread::~OpenPRSAgentThread()
{
}


void
OpenPRSAgentThread::init()
{
  agent_alive_ = false;
  cfg_agent_ = config->get_string("/openprs-agent/agent");
  openprs.lock();
  openprs->signal_msg_rcvd()
      .connect(boost::bind(&OpenPRSAgentThread::handle_message, this, _1, _2));
  openprs->transmit_command_f(openprs_kernel_name, "add (! (= @@AGENT_NAME \"%s\"))", cfg_agent_.c_str());
  openprs->transmit_command(openprs_kernel_name, "include \"agent-settings.inc\"");
  openprs.unlock();
}

void
OpenPRSAgentThread::finalize()
{
}


void
OpenPRSAgentThread::loop()
{
  if (agent_alive_) {
    openprs.lock();
    fawkes::Time now, now_sys;
    clock->get_time(now);
    clock->get_systime(now_sys);
    openprs->send_message_f(openprs_kernel_name, "(fawkes-time %lill %lill %lill %lill)",
			    now.get_sec(), now.get_usec(), now_sys.get_sec(), now_sys.get_usec());
    //openprs->transmit_command(openprs_kernel_name, "show intention");
    openprs.unlock();
  }
}


void
OpenPRSAgentThread::handle_message(std::string sender, std::string message)
{
  // remove newlines and anything beyond
  message.erase(std::remove(message.begin(), message.end(), '\n'), message.end());

  logger->log_debug(name(), "Received message from %s: %s", sender.c_str(), message.c_str());
  if (sender == openprs_kernel_name && message == "openprs-agent-init-done") {
      openprs.lock();
      openprs->transmit_command(openprs_kernel_name, "include \"agent-init.inc\"");
      openprs->transmit_command_f(openprs_kernel_name, "include \"%s.inc\"", cfg_agent_.c_str());
      openprs->transmit_command(openprs_kernel_name, "add (agent-init)");
      openprs.unlock();
      agent_alive_ = true;
  } else if (sender == "mp-oprs" && message == ("(unknown " + openprs_kernel_name + ")")) {
    logger->log_error(name(), "OpenPRS kernel has died, agent no longer alive");
    agent_alive_ = false;
  }
}
