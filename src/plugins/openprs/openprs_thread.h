
/***************************************************************************
 *  openprs_thread.h - OpenPRS aspect provider thread
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

#ifndef __PLUGINS_OPENPRS_OPENPRS_THREAD_H_
#define __PLUGINS_OPENPRS_OPENPRS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/aspect_provider.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/openprs/aspect/openprs_inifin.h>
#include <plugins/openprs/aspect/openprs_manager_inifin.h>

#include <string>
#include <thread>
#include <boost/asio.hpp>

namespace fawkes {
  class AspectIniFin;
  class SubProcess;
  class OpenPRSServerProxy;
  class OpenPRSMessagePasserProxy;
}

class OpenPRSThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::AspectProviderAspect
{
 public:
  OpenPRSThread();
  virtual ~OpenPRSThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // methods
  const std::list<fawkes::AspectIniFin *>  inifin_list();
  bool server_alive();
  void check_deadline(boost::asio::deadline_timer &deadline,
		      boost::asio::ip::tcp::socket &socket);

 private: // members
  bool         cfg_mp_run_;
  std::string  cfg_mp_bin_;
  std::string  cfg_mp_host_;
  unsigned int cfg_mp_port_;
  std::string  cfg_mp_port_s_;
  bool         cfg_mp_use_proxy_;
  unsigned int cfg_mp_proxy_port_;
  bool         cfg_server_run_;
  std::string  cfg_server_bin_;
  std::string  cfg_server_host_;
  unsigned int cfg_server_port_;
  std::string  cfg_server_port_s_;
  unsigned int cfg_server_proxy_port_;
  float        cfg_server_timeout_;
  float        cfg_kernel_timeout_;

  fawkes::SubProcess *proc_mp_;
  fawkes::SubProcess *proc_srv_;

  fawkes::OpenPRSAspectIniFin                    openprs_aspect_inifin_;
  fawkes::OpenPRSManagerAspectIniFin             openprs_manager_aspect_inifin_;
  fawkes::LockPtr<fawkes::OpenPRSKernelManager>  openprs_kernel_mgr_;

  fawkes::OpenPRSServerProxy                    *openprs_server_proxy_;
  fawkes::OpenPRSMessagePasserProxy             *openprs_mp_proxy_;

  boost::asio::io_service        io_service_;
  std::thread                    io_service_thread_;
  boost::asio::ip::tcp::socket   server_socket_;
  boost::asio::deadline_timer    deadline_;

};

#endif
