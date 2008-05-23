
/***************************************************************************
 *  fam.h - File Alteration Monitor
 *
 *  Created: Fri May 23 11:38:41 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __UTILS_SYSTEM_FAM_H_
#define __UTILS_SYSTEM_FAM_H_

#include <core/utils/lock_list.h>

#ifdef HAVE_INOTIFY
#  include <sys/inotify.h>
#  include <map>
#  include <string>
#endif
#include <sys/types.h>
#include <regex.h>

namespace fawkes {

class FamListener
{
 public:
  virtual ~FamListener();

  virtual void fam_event(const char *filename, unsigned int mask) = 0;
};

class FileAlterationMonitor
{
 public:
  FileAlterationMonitor();
  ~FileAlterationMonitor();

  void watch_dir(const char *dirpath);
  void add_filter(const char *regex);

  void process_events();

  void add_listener(FamListener *listener);
  void remove_listener(FamListener *listener);

 private:
  fawkes::LockList<FamListener *>            __listeners;
  fawkes::LockList<FamListener *>::iterator  __lit;
  fawkes::LockList<regex_t *>                __regexes;
  fawkes::LockList<regex_t *>::iterator      __rxit;

#ifdef HAVE_INOTIFY
  int     __inotify_fd;
  char   *__inotify_buf;
  size_t  __inotify_bufsize;
  std::map<int, std::string> __inotify_watches;
  std::map<int, std::string>::iterator __inotify_wit;
#endif

};

} // end of namespace fawkes

#endif
