
/***************************************************************************
 *  access_log.h - Web server access logger
 *
 *  Created: Fr Feb 14 22:18:04 2014
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

#ifndef __LIBS_WEBVIEW_ACCESS_LOG_H_
#define __LIBS_WEBVIEW_ACCESS_LOG_H_

#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Mutex;
class WebRequest;

class WebviewAccessLog
{
 public:
  WebviewAccessLog(const char *filename);
  ~WebviewAccessLog();

  void log(const WebRequest *request);

 private:
  Mutex *mutex_;
  FILE  *logfile_;
};

}

#endif
