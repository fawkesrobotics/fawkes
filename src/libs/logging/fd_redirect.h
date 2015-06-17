
/***************************************************************************
 *  fd_redirect.h - Redirect file descriptor writes to log
 *
 *  Created: Thu Aug 21 14:59:57 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __LOGGING_FD_REDIRECT_H_
#define __LOGGING_FD_REDIRECT_H_

#include <logging/logger.h>

#include <boost/asio.hpp>
#include <thread>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class LogFileDescriptorToLog {
 public:
  LogFileDescriptorToLog(int fd, Logger *logger, const char *logname, Logger::LogLevel log_level);
  ~LogFileDescriptorToLog();

 private:
  void start_log(const char *logname, Logger::LogLevel log_level,
		 boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf);
  void handle_log_line(const char *logname, Logger::LogLevel log_level,
		       boost::asio::posix::stream_descriptor &sd, boost::asio::streambuf &buf,
		       boost::system::error_code ec, size_t bytes_read);

 private:
  int log_fd_;
  int old_fd_;
  int old_fd_dup_;

  boost::asio::io_service               io_service_;
  std::thread                           io_service_thread_;
  boost::asio::io_service::work         io_service_work_;

  boost::asio::posix::stream_descriptor stream_;
  boost::asio::streambuf                buffer_;

  Logger           *logger_;
  std::string       log_name_;
  Logger::LogLevel  log_level_;

};

} // end namespace fawkes

#endif

