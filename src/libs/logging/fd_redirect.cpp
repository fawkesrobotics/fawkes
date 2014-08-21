
/***************************************************************************
 *  fd_redirect.cpp - Redirect file descriptor writes to log
 *
 *  Created: Thu Aug 21 15:03:37 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <logging/fd_redirect.h>
#include <logging/logger.h>
#include <boost/bind.hpp>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LogFileDescriptorToLog <logging/fd_redirect.h>
 * Redirect a file descriptor to the log.
 * This re-binds the file descriptor to a pipe, where it listens on the
 * reading end and prints input to the logger. On destruction, it restores
 * the original file descriptor (which is therefore not closed but re-bound.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param fd file descriptor to redirect to the log
 * @param logger logger to redirect to
 * @param logname name to use as log component name
 * @param log_level log level to log with
 */
LogFileDescriptorToLog::LogFileDescriptorToLog(int fd, Logger *logger,
					       const char *logname, Logger::LogLevel log_level)
  : io_service_work_(io_service_), stream_(io_service_),
    logger_(logger), log_name_(logname), log_level_(log_level)
{
  old_fd_ = fd;
  old_fd_dup_ = dup(fd);
  int log_pipe[2];
  if (pipe(log_pipe) == -1) {
    throw Exception(errno, "Failed to create log pipe");
  }
  
  if (dup2(log_pipe[1], fd) == -1) {
    throw Exception(errno, "Failed to dup2 pipe to fd");
  }

  log_fd_ = dup(log_pipe[0]);
  stream_.assign(log_fd_);

  // pipe fds have both been dup'ed
  close(log_pipe[0]);
  close(log_pipe[1]);

  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });

  start_log(log_name_.c_str(), log_level_, stream_, buffer_);
}


/** Destructor. */
LogFileDescriptorToLog::~LogFileDescriptorToLog()
{
  io_service_.stop();
  io_service_thread_.join();
  // restore original file handle
  dup2(old_fd_dup_, old_fd_);

  close(old_fd_dup_);
  close(log_fd_);
}


void
LogFileDescriptorToLog::start_log(const char *logname, Logger::LogLevel log_level,
				  boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf)
{
  boost::asio::async_read_until(sd, buf, '\n',
				boost::bind(
			          &LogFileDescriptorToLog::handle_log_line, this,
				  logname, log_level, boost::ref(sd), boost::ref(buf),
				  boost::asio::placeholders::error,
				  boost::asio::placeholders::bytes_transferred
			        ));
}


void
LogFileDescriptorToLog::handle_log_line(const char *logname, Logger::LogLevel log_level,
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



} // end namespace fawkes
