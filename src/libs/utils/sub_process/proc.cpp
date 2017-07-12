
/***************************************************************************
 *  proc.cpp - Sub-process facilities
 *
 *  Created: Mon Aug 18 16:56:46 2014
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

#include "proc.h"

#include <core/exception.h>

#include <boost/bind.hpp>

#ifdef HAVE_LIBDAEMON
#  include <libdaemon/dfork.h>
#endif
#include <sys/types.h>
#include <sys/wait.h>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SubProcess <libs/utils/sub_process/proc.h>
 * Sub-process execution with stdin/stdout/stderr redirection.
 * This class executes a sub-process and monitors it and supports redirecting
 * stdout/stderr to a logger.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param progname name of program, component name for logging
 * @param file file to execute, can be a program in the path or a
 * fully qualified path
 * @param argv array of arguments for the process, the last element
 * must be NULL
 * @param envp array of environment variables for the process, the
 * last element must be NULL. Can be NULL to omit.
 */
SubProcess::SubProcess(const char *progname, const char *file, const char *argv[], const char *envp[])
  : progname_(progname),
    io_service_work_(io_service_), logger_(NULL),
    sd_stdin_(io_service_), sd_stdout_(io_service_), sd_stderr_(io_service_)
{
  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });
  run_proc(file, argv, envp);
}


/** Constructor.
 * @param progname name of program, component name for logging
 * @param file file to execute, can be a program in the path or a
 * fully qualified path
 * @param argv array of arguments for the process, the last element
 * must be NULL
 * @param envp array of environment variables for the process, the
 * last element must be NULL. Can be NULL to omit.
 * @param logger logger to redirect stdout and stderr to
 */
SubProcess::SubProcess(const char *progname, const char *file, const char *argv[], const char *envp[],
		       fawkes::Logger *logger)
  : progname_(progname),
    io_service_work_(io_service_), logger_(logger),
    sd_stdin_(io_service_), sd_stdout_(io_service_), sd_stderr_(io_service_)
{
  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });
  run_proc(file, argv, envp);
}

/** Constructor.
 * @param progname name of program, component name for logging
 * @param file file to execute, can be a program in the path or a
 * fully qualified path
 * @param argv array of arguments for the process, the last element
 * must be NULL
 * @param envp array of environment variables for the process, the
 * last element must be NULL. Can be NULL to omit.
 */
SubProcess::SubProcess(const std::string &progname, const std::string &file,
                       const std::vector<std::string> &argv, const std::vector<std::string> &envp)
	: progname_(progname),
	  io_service_work_(io_service_), logger_(NULL),
	  sd_stdin_(io_service_), sd_stdout_(io_service_), sd_stderr_(io_service_)
{
  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });

  const char *argvc[argv.size() + 1];
  for (size_t i = 0; i < argv.size(); ++i) {
	  argvc[i] = argv[i].c_str();
  }
  argvc[argv.size()] = NULL;
  const char *envpc[envp.size() + 1];
  for (size_t i = 0; i < envp.size(); ++i) {
	  envpc[i] = envp[i].c_str();
  }
  envpc[envp.size()] = NULL;
  run_proc(file.c_str(), argvc, envpc);
}


/** Constructor.
 * @param progname name of program, component name for logging
 * @param file file to execute, can be a program in the path or a
 * fully qualified path
 * @param argv array of arguments for the process, the last element
 * must be NULL
 * @param envp array of environment variables for the process, the
 * last element must be NULL. Can be NULL to omit.
 * @param logger logger to redirect stdout and stderr to
 */
SubProcess::SubProcess(const std::string &progname, const std::string &file,
                       const std::vector<std::string> &argv, const std::vector<std::string> &envp,
                       fawkes::Logger *logger)
  : progname_(progname),
    io_service_work_(io_service_), logger_(logger),
    sd_stdin_(io_service_), sd_stdout_(io_service_), sd_stderr_(io_service_)
{
  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });

  const char *argvc[argv.size() + 1];
  for (size_t i = 0; i < argv.size(); ++i) {
	  argvc[i] = argv[i].c_str();
  }
  argvc[argv.size()] = NULL;
  const char *envpc[envp.size() + 1];
  for (size_t i = 0; i < envp.size(); ++i) {
	  envpc[i] = envp[i].c_str();
  }
  envpc[envp.size()] = NULL;
  run_proc(file.c_str(), argvc, envpc);
}


/** Destructor. */
SubProcess::~SubProcess()
{
  this->kill(SIGTERM);
  io_service_.stop();
  io_service_thread_.join();
}


/** Send a signal to the process.
 * @param signum signal number
 */
void
SubProcess::kill(int signum)
{
  if (pid_ > 0)  ::kill(pid_, signum);
}


pid_t
SubProcess::run_proc(const char *file, const char *argv[], const char *envp[],
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
SubProcess::run_proc(const char *file, const char *argv[], const char *envp[])
{
  pid_ = run_proc(file, argv, envp,
		  pipe_stdin_w_, pipe_stdout_r_, pipe_stderr_r_);

  sd_stdin_.assign(dup(pipe_stdin_w_));
  sd_stdout_.assign(dup(pipe_stdout_r_));
  sd_stderr_.assign(dup(pipe_stderr_r_));

  if (logger_) {
    start_log(progname_.c_str(), Logger::LL_INFO, sd_stdout_, buf_stdout_);
    start_log(progname_.c_str(), Logger::LL_WARN, sd_stderr_, buf_stderr_);
  }
}


void
SubProcess::start_log(const char *logname, Logger::LogLevel log_level,
			 boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf)
{
  boost::asio::async_read_until(sd, buf, '\n',
				boost::bind(
			          &SubProcess::handle_log_line, this,
				  logname, log_level, boost::ref(sd), boost::ref(buf),
				  boost::asio::placeholders::error,
				  boost::asio::placeholders::bytes_transferred
			        ));
}


void
SubProcess::handle_log_line(const char *logname, Logger::LogLevel log_level,
			    boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf,
			    boost::system::error_code ec, size_t bytes_read)
{
  if (ec) {
    if (ec == boost::asio::error::eof) {
      // stop logging
      return;
    } else {
      logger_->log_error(logname, "Failed to read log line %i (%s), continuing", ec.value(),
			 ec.message().c_str());
    }
  } else {
    std::string line;
    std::istream in_stream(&buf);
    std::getline(in_stream, line);
    logger_->log(log_level, logname, "%s", line.c_str());
  }
  start_log(logname, log_level, sd, buf);
}


/** Check if process is alive.
 * @return true if process is alive, false otherwise
 */
bool
SubProcess::alive()
{
	check_proc();
	return pid_ > 0;
}


/** Check if the process is still alive. */
void
SubProcess::check_proc()
{
	if (pid_ > 0) {
		int status = 0;
		if (waitpid(pid_, &status, WUNTRACED | WCONTINUED | WNOHANG) > 0) {
			if (WIFEXITED(status)) {
				if (WEXITSTATUS(status) != 0) {
					logger_->log_error(progname_.c_str(), "PID %i exited, status=%d",
					                   pid_, WEXITSTATUS(status));
				}
				pid_ = -1;
			} else if (WIFSIGNALED(status)) {
				logger_->log_error(progname_.c_str(), "PID %i killed by signal %s",
				                   pid_, strsignal(WTERMSIG(status)));
				pid_ = -1;
			} else if (WIFSTOPPED(status)) {
				logger_->log_warn(progname_.c_str(), "PID %i stopped by signal %s",
				                  pid_, strsignal(WSTOPSIG(status)));
			} else if (WIFCONTINUED(status)) {
				logger_->log_warn(progname_.c_str(), "PID %i continued", pid_);
			}
    }
  }
}

} // end namespace fawkes
