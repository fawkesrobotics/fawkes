
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

#include <logging/logger.h>

#include <unistd.h>
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <libdaemon/dfork.h>

#include <boost/format.hpp>
#include <boost/bind.hpp>

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
    io_service_work_(io_service_),
    proc_mp_("OPRS-mp", io_service_), proc_srv_("OPRS-server", io_service_)
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
    const char *filename = cfg_mp_bin_.c_str();
    const char *argv[] = { filename, "-j", cfg_mp_port_.c_str(), NULL };
    run_proc(filename, argv, NULL, proc_mp_);
  } else {
    proc_mp_.pid = -1;
  }

  if (cfg_server_run_) {
    const char *filename = cfg_server_bin_.c_str();
    const char *argv[] = { filename, "-j", cfg_mp_port_.c_str(), "-i", cfg_server_port_.c_str(), NULL };
    run_proc(filename, argv, NULL, proc_srv_);
  } else {
    proc_srv_.pid = -1;
  }
}


pid_t
OpenPRSThread::run_proc(const char *file, const char *argv[], const char *envp[],
			int & pipe_stdin_w, int & pipe_stdout_r, int & pipe_stderr_r)
{
  int pipe_stdin[2];
  int pipe_stdout[2];
  int pipe_stderr[2];

  if (pipe(pipe_stdin) < 0) {
    throw Exception(errno, "Failed to create OpenPRS stdin pipe (%s)", file);
  }
  if (pipe(pipe_stdout) < 0) {
    close(pipe_stdin[0]);
    close(pipe_stdin[1]);
    throw Exception(errno, "Failed to create OpenPRS stdout pipe (%s)", file);
  }
  if (pipe(pipe_stderr) < 0) {
    close(pipe_stdin[0]);
    close(pipe_stdin[1]);
    close(pipe_stdout[0]);
    close(pipe_stdout[1]);
    throw Exception(errno, "Failed to create OpenPRS stderr pipe (%s)", file);
  }

  pid_t pid = fork();
  if (pid < 0) {    // fail
    close(pipe_stdin[0]);
    close(pipe_stdin[1]);
    close(pipe_stdout[0]);
    close(pipe_stdout[1]);
    close(pipe_stderr[0]);
    close(pipe_stderr[1]);
    throw Exception(errno, "Failed to fork for OpenPRS %s", file);
  } else if (pid) { // parent
    close(pipe_stdin[0]);
    close(pipe_stdout[1]);
    close(pipe_stderr[1]);

    pipe_stdin_w  = pipe_stdin[1];
    pipe_stdout_r = pipe_stdout[0];
    pipe_stderr_r = pipe_stderr[0];

    return pid;
  } else {          // child
#ifdef HAVE_LIBDAEMON
    daemon_close_all(STDIN_FILENO, STDOUT_FILENO, STDERR_FILENO,
		     pipe_stdin[0], pipe_stdout[1], pipe_stderr[1], -1);
#endif

    if (dup2(pipe_stdin[0],  STDIN_FILENO) == -1) {
      perror("Failed to dup stdin");
      ::exit(-1);
    }
    if (dup2(pipe_stdout[1], STDOUT_FILENO) == -1) {
      perror("Failed to dup stdout");
      ::exit(-1);
    }
    if (dup2(pipe_stderr[1], STDERR_FILENO) == -1) {
      perror("Failed to dup stderr");
      ::exit(-1);
    }

    close(pipe_stdin[0]);
    close(pipe_stdout[0]);
    close(pipe_stderr[0]);
    close(pipe_stdin[1]);
    close(pipe_stdout[1]);
    close(pipe_stderr[1]);

    execvpe(file, (char * const *)argv, envp ? (char * const *)envp : environ);

    // execvpe only returns on error, which is when we should exit
    perror("Failed to execute command");
    ::exit(-2);
  }
}


void
OpenPRSThread::run_proc(const char *file, const char *argv[], const char *envp[], SubProc &proc_info)
{
  proc_info.pid = run_proc(file, argv, NULL,
			   proc_info.pipe_stdin_w, proc_info.pipe_stdout_r, proc_info.pipe_stderr_r);

  proc_info.sd_stdin.assign(dup(proc_info.pipe_stdin_w));
  proc_info.sd_stdout.assign(dup(proc_info.pipe_stdout_r));
  proc_info.sd_stderr.assign(dup(proc_info.pipe_stderr_r));

  start_log(proc_info.progname.c_str(), Logger::LL_INFO, proc_info.sd_stdout, proc_info.buf_stdout);
  start_log(proc_info.progname.c_str(), Logger::LL_INFO, proc_info.sd_stderr, proc_info.buf_stderr);
}


void
OpenPRSThread::start_log(const char *logname, Logger::LogLevel log_level,
			 boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf)
{
  boost::asio::async_read_until(sd, buf, '\n',
				boost::bind(
			          &OpenPRSThread::handle_log_line, this,
				  logname, log_level, boost::ref(sd), boost::ref(buf),
				  boost::asio::placeholders::error,
				  boost::asio::placeholders::bytes_transferred
			        ));
}


void
OpenPRSThread::handle_log_line(const char *logname, Logger::LogLevel log_level,
			       boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf,
			       boost::system::error_code ec, size_t bytes_read)
{
  if (ec) {
    if (ec == boost::asio::error::eof) {
      // stop logging
      return;
    } else {
      logger->log_error(logname, "Failed to read log line %i (%s), continuing", ec.value(),
			ec.message().c_str());
    }
  } else {
    char line[bytes_read];
    std::istream in_stream(&buf);
    in_stream.read(line, bytes_read);
    if (bytes_read == 1) { // newline
      logger->log(log_level, logname, "");
    } else {
      line[bytes_read - 2] = 0;
      logger->log(log_level, logname, "%s", line);
    }
  }
  start_log(logname, log_level, sd, buf);
}

void
OpenPRSThread::finalize()
{
  if (proc_srv_.pid > 0)  ::kill(proc_srv_.pid, SIGINT);
  if (proc_mp_.pid > 0)   ::kill(proc_mp_.pid, SIGINT);

  io_service_.stop();
  io_service_thread_.join();
}


void
OpenPRSThread::loop()
{
  check_proc(proc_srv_);
  check_proc(proc_mp_);
}


void
OpenPRSThread::check_proc(SubProc &proc)
{
  if (proc.pid > 0) {
    int status = 0;
    if (waitpid(proc.pid, &status, WUNTRACED | WCONTINUED | WNOHANG) > 0) {
      if (WIFEXITED(status)) {
	logger->log_error(name(), "PID %i|%s exited, status=%d",
			  proc.pid, proc.progname.c_str(), WEXITSTATUS(status));
	proc.pid = -1;
      } else if (WIFSIGNALED(status)) {
	logger->log_error(name(), "PID %i|%s killed by signal %s",
	       proc.pid, proc.progname.c_str(), strsignal(WTERMSIG(status)));
	proc.pid = -1;
      } else if (WIFSTOPPED(status)) {
	logger->log_warn(name(), "PID %i|%s stopped by signal %s",
	       proc.pid, proc.progname.c_str(), strsignal(WSTOPSIG(status)));
      } else if (WIFCONTINUED(status)) {
	logger->log_warn(name(), "PID %i|%s continued", proc.pid, proc.progname.c_str());
      }
    }
  }
}
