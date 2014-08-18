
/***************************************************************************
 *  openprs_thread.h - OpenPRS aspect provider thread
 *
 *  Created: Sat Jun 16 14:38:21 2012 (Mexico City)
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

#ifndef __PLUGINS_OPENPRS_OPENPRS_THREAD_H_
#define __PLUGINS_OPENPRS_OPENPRS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/aspect_provider.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
// include <plugins/openprs/aspect/openprs_inifin.h>

#include <boost/asio.hpp>
#include <thread>
#include <string>

namespace fawkes {
  class AspectIniFin;
}

class OpenPRSThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect
//public fawkes::AspectProviderAspect
{
 public:
  OpenPRSThread();
  virtual ~OpenPRSThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // types
  class SubProc
  {
   public:
    SubProc(std::string progname, boost::asio::io_service &io_service)
    : progname(progname),
      sd_stdin(io_service), sd_stdout(io_service), sd_stderr(io_service) {}

    std::string  progname;

    pid_t        pid;
    int          pipe_stdin_w;
    int          pipe_stdout_r;
    int          pipe_stderr_r;

    boost::asio::posix::stream_descriptor sd_stdin;
    boost::asio::posix::stream_descriptor sd_stdout;
    boost::asio::posix::stream_descriptor sd_stderr;

    boost::asio::streambuf buf_stdout;
    boost::asio::streambuf buf_stderr;
  };

 private: // methods
  pid_t run_proc(const char *file, const char *argv[], const char *envp[],
		 int & pipe_stdin_w, int & pipe_stdout_r, int & pipe_stderr_r);

  void  run_proc(const char *file, const char *argv[], const char *envp[],
		 SubProc &proc_info);
  void  check_proc(SubProc &proc);

  void start_log(const char *logname, fawkes::Logger::LogLevel log_level,
		 boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf);
  void handle_log_line(const char *logname, fawkes::Logger::LogLevel log_level,
		       boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf,
		       boost::system::error_code ec, size_t bytes_read);

 private: // members
  bool         cfg_mp_run_;
  std::string  cfg_mp_bin_;
  std::string  cfg_mp_port_;
  bool         cfg_server_run_;
  std::string  cfg_server_bin_;
  std::string  cfg_server_port_;

  //fawkes::OpenPRSAspectIniFin openprs_aspect_inifin_;

  boost::asio::io_service               io_service_;
  std::thread                           io_service_thread_;
  boost::asio::io_service::work         io_service_work_;

  SubProc proc_mp_;
  SubProc proc_srv_;

};

#endif
