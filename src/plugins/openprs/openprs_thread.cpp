
/***************************************************************************
 *  openprs_thread.cpp -  OpenPRS environment providing Thread
 *
 *  Created: Thu Aug 14 15:52:35 2014
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

#include "openprs_thread.h"
#include "utils/proc.h"

#include <logging/logger.h>

#include <unistd.h>
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <csignal>

#include <boost/format.hpp>

using namespace fawkes;

/** @class OpenPRSThread "openprs_thread.h"
 * OpenPRS environment thread.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenPRSThread::OpenPRSThread()
  : Thread("OpenPRSThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE),
{
}


/** Destructor. */
OpenPRSThread::~OpenPRSThread()
{
}


void
OpenPRSThread::init()
{
  cfg_mp_run_      = config->get_bool("/openprs/message-passer/run");
  cfg_mp_bin_      = config->get_string("/openprs/message-passer/binary");
  cfg_mp_port_     = boost::str(boost::format("%u") % config->get_uint("/openprs/message-passer/tcp-port"));
  cfg_server_run_  = config->get_bool("/openprs/server/run");
  cfg_server_bin_  = config->get_string("/openprs/server/binary");
  cfg_server_port_ = boost::str(boost::format("%u") % config->get_uint("/openprs/server/tcp-port"));

  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });

  if (cfg_mp_run_) {
    logger->log_warn(name(), "Running OPRS-mp");
    const char *filename = cfg_mp_bin_.c_str();
    const char *argv[] = { filename, "-j", cfg_mp_port_s_.c_str(), NULL };
    proc_mp_  = new SubProcess("OPRS-mp", filename, argv, NULL, logger);
  } else {
    proc_mp_ = NULL;
  }

  if (cfg_server_run_) {
    logger->log_warn(name(), "Running OPRS-server");
    const char *filename = cfg_server_bin_.c_str();
    const char *argv[] = { filename, "-j", cfg_mp_port_s_.c_str(), "-i", cfg_server_port_s_.c_str(), NULL };
    proc_srv_ = new SubProcess("OPRS-server", filename, argv, NULL, logger);
  } else {
    proc_srv_ = NULL;
  }

  // give some time for OpenPRS to come up
  usleep(500000);

  logger->log_info(name(), "Starting OpenPRS server proxy");


  } else {
  }
}

void
OpenPRSThread::finalize()
{
  if (proc_srv_)  proc_srv_->kill(SIGINT);
  if (proc_mp_)   proc_mp_->kill(SIGINT);

  delete proc_srv_;
  delete proc_mp_;
}


void
OpenPRSThread::loop()
{
  if (proc_srv_) proc_srv_->check_proc();
  if (proc_mp_)  proc_mp_->check_proc();
}

