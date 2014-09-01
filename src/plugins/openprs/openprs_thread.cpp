
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
#include "utils/openprs_server_proxy.h"
#include "utils/openprs_mp_proxy.h"
#include "utils/proc.h"

#include <logging/logger.h>
#include <baseapp/run.h>
#include <netcomm/fawkes/network_manager.h>

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
    AspectProviderAspect(inifin_list())
{
}


/** Destructor. */
OpenPRSThread::~OpenPRSThread()
{
}


void
OpenPRSThread::init()
{
  char hostname[HOST_NAME_MAX];
  if (gethostname(hostname, HOST_NAME_MAX) == -1) {
    strcpy(hostname, "localhost");
  }

  cfg_mp_run_      = config->get_bool("/openprs/message-passer/run");
  cfg_mp_bin_      = config->get_string("/openprs/message-passer/binary");
  try {
    cfg_mp_host_     = config->get_string("/openprs/message-passer/hostname");
  } catch (Exception &e) {
    cfg_mp_host_ = hostname;
  }
  cfg_mp_port_       = config->get_uint("/openprs/message-passer/tcp-port");
  cfg_mp_port_s_     = boost::str(boost::format("%u") % cfg_mp_port_);
  cfg_mp_use_proxy_  = config->get_bool("/openprs/message-passer/use-proxy");
  cfg_mp_proxy_port_ = config->get_uint("/openprs/message-passer/proxy-tcp-port");

  cfg_server_run_    = config->get_bool("/openprs/server/run");
  cfg_server_bin_    = config->get_string("/openprs/server/binary");
  try {
    cfg_server_host_ = config->get_string("/openprs/server/hostname");
  } catch (Exception &e) {
    cfg_server_host_ = hostname;
  }
  cfg_server_port_   = config->get_uint("/openprs/server/tcp-port");
  cfg_server_port_s_ = boost::str(boost::format("%u") % cfg_server_port_);
  cfg_server_proxy_port_ = config->get_uint("/openprs/server/proxy-tcp-port");

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
    const char *argv[] = { filename,
			   "-j", cfg_mp_port_s_.c_str(),
			   "-i", cfg_server_port_s_.c_str(),
			   "-l", "lower", NULL };
    proc_srv_ = new SubProcess("OPRS-server", filename, argv, NULL, logger);
  } else {
    proc_srv_ = NULL;
  }

  // give some time for OpenPRS to come up
  usleep(500000);

  logger->log_info(name(), "Starting OpenPRS server proxy");

  openprs_server_proxy_ = new OpenPRSServerProxy(cfg_server_proxy_port_,
						 cfg_server_host_, cfg_server_port_, logger);

  if (cfg_mp_use_proxy_) {
    logger->log_info(name(), "Starting OpenPRS message passer proxy");
    openprs_mp_proxy_     = new OpenPRSMessagePasserProxy(cfg_mp_proxy_port_,
							  cfg_mp_host_, cfg_mp_port_, logger);
  } else {
    openprs_mp_proxy_ = NULL;
  }

  logger->log_warn(name(), "Initializing kernel manager");
  openprs_kernel_mgr_ = new OpenPRSKernelManager(hostname, cfg_server_proxy_port_,
						 cfg_mp_use_proxy_ ? hostname : cfg_mp_host_,
						 cfg_mp_use_proxy_ ? cfg_mp_proxy_port_ : cfg_mp_port_,
						 logger, clock, config);
  openprs_aspect_inifin_.prepare("localhost", fawkes::runtime::network_manager->fawkes_port(),
				 openprs_kernel_mgr_, openprs_server_proxy_, openprs_mp_proxy_);
  openprs_manager_aspect_inifin_.set_manager(openprs_kernel_mgr_);
}

void
OpenPRSThread::finalize()
{
  if (proc_srv_)  proc_srv_->kill(SIGINT);
  if (proc_mp_)   proc_mp_->kill(SIGINT);

  delete proc_srv_;
  delete proc_mp_;

  delete openprs_server_proxy_;
  delete openprs_mp_proxy_;
  openprs_kernel_mgr_.clear();
}


void
OpenPRSThread::loop()
{
  if (proc_srv_) proc_srv_->check_proc();
  if (proc_mp_)  proc_mp_->check_proc();
}

const std::list<AspectIniFin *>
OpenPRSThread::inifin_list()
{
  std::list<AspectIniFin *> rv;
  rv.push_back(&openprs_aspect_inifin_);
  rv.push_back(&openprs_manager_aspect_inifin_);
  return rv;
}
