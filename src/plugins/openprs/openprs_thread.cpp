
/***************************************************************************
 *  openprs_thread.cpp -  OpenPRS environment providing Thread
 *
 *  Created: Thu Aug 14 15:52:35 2014
 *  Copyright  2014-2015  Tim Niemueller [www.niemueller.de]
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

#include <logging/logger.h>
#include <baseapp/run.h>
#include <netcomm/fawkes/network_manager.h>
#include <utils/sub_process/proc.h>

#include <unistd.h>
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <csignal>

#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

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
    AspectProviderAspect(inifin_list()),
    server_socket_(io_service_), deadline_(io_service_)
{
}


/** Destructor. */
OpenPRSThread::~OpenPRSThread()
{
}


void
OpenPRSThread::init()
{
  proc_srv_ = NULL;
  proc_mp_  = NULL;
  openprs_server_proxy_ = NULL;
  openprs_mp_proxy_ = NULL;

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

  cfg_server_timeout_ = config->get_float("/openprs/server/timeout");
  cfg_kernel_timeout_ = config->get_float("/openprs/kernels/start-timeout");

  openprs_aspect_inifin_.set_kernel_timeout(cfg_kernel_timeout_);

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

#if BOOST_VERSION >= 104800
  logger->log_info(name(), "Verifying OPRS-server availability");

  boost::asio::ip::tcp::resolver resolver(io_service_);
  boost::asio::ip::tcp::resolver::query query(cfg_server_host_, cfg_server_port_s_);
  boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

  // this is just the overly complicated way to get a timeout on
  // a synchronous connect, cf.
  // http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/example/cpp03/timeouts/blocking_tcp_client.cpp
  deadline_.expires_at(boost::posix_time::pos_infin);
  check_deadline(deadline_, server_socket_);

  deadline_.expires_from_now(boost::posix_time::seconds(cfg_server_timeout_));

  boost::system::error_code ec = boost::asio::error::would_block;
  server_socket_.async_connect(iter->endpoint(),
  			       boost::lambda::var(ec) = boost::lambda::_1);

  // Block until the asynchronous operation has completed.
  do {
    io_service_.run_one();
#if BOOST_VERSION >= 105400 && BOOST_VERSION < 105500
    // Boost 1.54 has a bug that causes async_connect to report success
    // if it cannot connect at all to the other side, cf.
    // https://svn.boost.org/trac/boost/ticket/8795
    // Work around by explicitly checking for connected status
    if (! ec) {
      server_socket_.remote_endpoint(ec);
      if (ec == boost::system::errc::not_connected) {
        // continue waiting for timeout
        ec = boost::asio::error::would_block;
	server_socket_.async_connect(iter->endpoint(),
                                     boost::lambda::var(ec) = boost::lambda::_1);
      }
    }
#endif
  } while (ec == boost::asio::error::would_block);

  // Determine whether a connection was successfully established.
  if (ec || ! server_socket_.is_open()) {
    finalize();
    if (ec.value() == boost::system::errc::operation_canceled) {
      throw Exception("OpenPRS waiting for server to come up timed out");
    } else {
      throw Exception("OpenPRS waiting for server failed: %s", ec.message().c_str());
    }
  }
#else
  logger->log_warn(name(), "Cannot verify server aliveness, Boost too old");
#endif

  boost::asio::socket_base::keep_alive keep_alive_option(true);
  server_socket_.set_option(keep_alive_option);

  // receive greeting
  std::string greeting = OpenPRSServerProxy::read_string_from_socket(server_socket_);
  //logger->log_info(name(), "Received server greeting: %s", greeting.c_str());
  // send our greeting
  OpenPRSServerProxy::write_string_to_socket(server_socket_, "fawkes");
  OpenPRSServerProxy::write_int_to_socket(server_socket_, getpid());
  OpenPRSServerProxy::write_int_to_socket(server_socket_, 0);

  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });

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
  server_socket_.close();
  io_service_.stop();
  if (io_service_thread_.joinable()) {
    io_service_thread_.join();
  }

  if (proc_srv_) {
    logger->log_info(name(), "Killing OpenPRS server");
    proc_srv_->kill(SIGINT);
  }
  if (proc_mp_) {
    logger->log_info(name(), "Killing OpenPRS message passer");
    proc_mp_->kill(SIGINT);
  }

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


bool
OpenPRSThread::server_alive()
{
  if (server_socket_.is_open()) {
    boost::system::error_code ec;
    server_socket_.remote_endpoint(ec);
    return !ec;
  } else {
    return false;
  }
}


void
OpenPRSThread::check_deadline(boost::asio::deadline_timer &deadline,
			      boost::asio::ip::tcp::socket &socket)
{
  if (deadline.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
    socket.close();
    deadline.expires_at(boost::posix_time::pos_infin);
  }

#if BOOST_VERSION >= 104800
  deadline.async_wait(boost::lambda::bind(&OpenPRSThread::check_deadline, this,
					   boost::ref(deadline), boost::ref(socket)));
#else
  deadline.async_wait(boost::bind(&OpenPRSThread::check_deadline, this,
				   boost::ref(deadline), boost::ref(socket)));
#endif
}
