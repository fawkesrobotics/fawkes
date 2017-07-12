
/***************************************************************************
 *  proc.h - Sub-process facilities
 *
 *  Created: Mon Aug 18 16:54:47 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENPRS_UTILS_PROC_H_
#define __PLUGINS_OPENPRS_UTILS_PROC_H_

#include <logging/logger.h>

#include <boost/asio.hpp>
#include <string>
#include <thread>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

class SubProcess
{
 public:
	SubProcess(const char *progname, const char *file, const char *argv[], const char *envp[]);
	SubProcess(const char *progname, const char *file, const char *argv[], const char *envp[],
	           fawkes::Logger *logger);
	SubProcess(const std::string &progname, const std::string &file,
	           const std::vector<std::string> &argv, const std::vector<std::string> &envp);
	SubProcess(const std::string &progname, const std::string &file,
	           const std::vector<std::string> &argv, const std::vector<std::string> &envp,
	           fawkes::Logger *logger);
	~SubProcess();

  /** Get PID of sub-process.
   * @return process ID of sub-process. */
  pid_t pid() const
  { return pid_; }

  /** Get stdin pipe file descriptor.
   * @return stdin pipe file descriptor, only valid for writing. */
  int pipe_stdin_w() const
  { return pipe_stdin_w_; }

  /** Get stdout pipe file descriptor.
   * @return stdout pipe file descriptor, only valid for reading. */
  int pipe_stdout_r() const
  { return pipe_stdout_r_; }

  /** Get stderr pipe file descriptor.
   * @return stderr pipe file descriptor, only valid for reading. */
  int pipe_stderr_r() const
  { return pipe_stderr_r_; }

  /** Get stdin stream descriptor.
   * @return stdin stream descriptor, only valid for writing. */
  boost::asio::posix::stream_descriptor & sd_stdin()
  { return sd_stdin_; }

  /** Get stdout stream descriptor.
   * @return stdout stream descriptor, only valid for reading. */
  boost::asio::posix::stream_descriptor & sd_stdout()
  { return sd_stdout_; }

  /** Get stderr stream descriptor.
   * @return stderr stream descriptor, only valid for reading. */
  boost::asio::posix::stream_descriptor & sd_stderr()
  { return sd_stderr_; }

  void kill(int signum);
  void check_proc();
  bool alive();

 private:
  pid_t run_proc(const char *file, const char *argv[], const char *envp[],
		 int & pipe_stdin_w, int & pipe_stdout_r, int & pipe_stderr_r);

  void  run_proc(const char *file, const char *argv[], const char *envp[]);

  void start_log(const char *logname, fawkes::Logger::LogLevel log_level,
		 boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf);
  void handle_log_line(const char *logname, fawkes::Logger::LogLevel log_level,
		       boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf,
		       boost::system::error_code ec, size_t bytes_read);

  
 private:
  std::string  progname_;

  pid_t        pid_;
  int          pipe_stdin_w_;
  int          pipe_stdout_r_;
  int          pipe_stderr_r_;

  boost::asio::io_service               io_service_;
  std::thread                           io_service_thread_;
  boost::asio::io_service::work         io_service_work_;

  fawkes::Logger *logger_;

  boost::asio::posix::stream_descriptor sd_stdin_;
  boost::asio::posix::stream_descriptor sd_stdout_;
  boost::asio::posix::stream_descriptor sd_stderr_;

  boost::asio::streambuf buf_stdout_;
  boost::asio::streambuf buf_stderr_;

};

} // end namespace fawkes

#endif
