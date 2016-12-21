
/***************************************************************************
 *  access_log.cpp - Web server access logger
 *
 *  Created: Fr Feb 14 22:23:45 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include <webview/access_log.h>
#include <webview/request.h>

#include <core/exception.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

#include <cerrno>
#include <unistd.h>
#include <stdint.h>
#include <microhttpd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebviewAccessLog <webview/access_log.h>
 * Webview access_log writer.
 * This class can be used to create an access_log using the Apache
 * common log format.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filename log file name/path
 */
WebviewAccessLog::WebviewAccessLog(const char *filename)
{
  logfile_ = fopen(filename, "a");
  if (! logfile_) {
    throw Exception(errno, "Failed to open access log %s", filename);
  }
  mutex_ = new Mutex();
}

/** Destructor. */
WebviewAccessLog::~WebviewAccessLog()
{
  fclose(logfile_);
  delete mutex_;
}


/** Log a request.
 * @param request request to log
 */
void
WebviewAccessLog::log(const WebRequest *request)
{
  MutexLocker lock(mutex_);
  // Apache combined log:
  //"%h %l %u %t \"%r\" %>s %b \"%{Referer}i\" \"%{User-agent}i\"
  struct tm ltime;
  time_t timesec = request->time().get_sec();
  localtime_r(&timesec, &ltime);
  char timestr[1024];
  // [day/month/year:hour:minute:second zone]
  strftime(timestr, sizeof(timestr), "[%d/%b/%Y:%H:%M:%S %z]", &ltime);
  fprintf(logfile_, "%s - %s %s \"%s %s %s\" %i %zu \"%s\" \"%s\"\n",
	  request->client_addr().c_str(),
	  request->user().length() == 0 ? "-" : request->user().c_str(),
	  timestr,
	  request->method_str(), request->uri().c_str(), request->http_version_str(),
	  request->reply_code(), request->reply_size(),
	  request->has_header(MHD_HTTP_HEADER_REFERER)
	    ? request->header(MHD_HTTP_HEADER_REFERER).c_str() : "",
	  request->has_header(MHD_HTTP_HEADER_USER_AGENT)
	    ? request->header(MHD_HTTP_HEADER_USER_AGENT).c_str() : "");

  fflush(logfile_);
}

} // end namespace fawkes
