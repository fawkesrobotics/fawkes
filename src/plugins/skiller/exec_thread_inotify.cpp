
/***************************************************************************
 *  exec_thread_inotify.cpp - Fawkes Skiller: Execution Thread inotify bits
 *
 *  Created: Wed Mar 26 18:55:09 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/skiller/exec_thread.h>

#ifdef HAVE_INOTIFY
#  include <sys/inotify.h>
#  include <sys/types.h>
#  include <sys/stat.h>
#  include <poll.h>
#  include <dirent.h>
#  include <unistd.h>
#  include <cerrno>
#  include <cstring>
#endif

void
SkillerExecutionThread::inotify_watch_dir(std::string dir)
{
#ifdef HAVE_INOTIFY
  DIR *d = opendir(dir.c_str());
  if ( d == NULL ) {
    throw Exception(errno, "Failed to open dir %s", dir.c_str());
  }

  uint32_t mask = IN_MODIFY | IN_MOVE | IN_CREATE | IN_DELETE | IN_DELETE_SELF;
  int iw;

  logger->log_debug("SkillerExecutionThread", "Adding watch for %s", dir.c_str());
  if ( (iw = inotify_add_watch(__inotify_fd, dir.c_str(), mask)) >= 0) {
    __inotify_watches[iw] = dir;

    dirent de, *res;
    while ( (readdir_r(d, &de, &res) == 0) && (res != NULL) ) {
      std::string fp = dir + "/" + de.d_name;
      struct stat st;
      if ( stat(fp.c_str(), &st) == 0 ) {
	if ( (de.d_name[0] != '.') && S_ISDIR(st.st_mode) ) {
	  try {
	    inotify_watch_dir(fp);
	  } catch (Exception &e) {
	    closedir(d);
	    throw;
	  }
	//} else {
	  //logger->log_debug("SkillerExecutionThread", "Skipping file %s", fp.c_str());	  
	}
      } else {
	logger->log_debug("SkillerExecutionThread", "Skipping watch on %s, cannot stat (%s)", fp.c_str(),
			  strerror(errno));
      }
    }
  } else {
    throw Exception("SkillerExecutionThread", "Cannot add watch for %s", dir.c_str());
  }

  closedir(d);
#endif
}

void
SkillerExecutionThread::init_inotify()
{
#ifdef HAVE_INOTIFY
  if ( __cfg_watch_files ) {

    if ( (__inotify_fd = inotify_init()) == -1 ) {
      throw Exception(errno, "Failed to initialize inotify");
    }

    int regerr = 0;
    if ( (regerr = regcomp(&__inotify_regex, "^[^.].*\\.lua$", REG_EXTENDED)) != 0 ) {
      char errtmp[1024];
      regerror(regerr, &__inotify_regex, errtmp, sizeof(errtmp));
      close_inotify();
      throw Exception("Failed to compile lua file regex: %s", errtmp);
    }

    // from http://www.linuxjournal.com/article/8478
    __inotify_bufsize = 1024 * (sizeof(struct inotify_event) + 16);
    __inotify_buf     = (char *)malloc(__inotify_bufsize);

    try {
      inotify_watch_dir(SKILLDIR);
    } catch (Exception &e) {
      close_inotify();
    }
  } else {
    logger->log_warn("SkillerExecutionThread", "Auto-reload disabled in config.");
  }
#else
  logger->log_warn("SkillerExecutionThread", "inotify support not available, auto-reload "
		   "disabled.");
#endif
}


void
SkillerExecutionThread::close_inotify()
{
#ifdef HAVE_INOTIFY
  if ( ! __cfg_watch_files )  return;

  for (__inotify_wit = __inotify_watches.begin(); __inotify_wit != __inotify_watches.end(); ++__inotify_wit) {
    inotify_rm_watch(__inotify_fd, __inotify_wit->first);
  }
  close(__inotify_fd);
  if ( __inotify_buf ) {
    free(__inotify_buf);
    __inotify_buf = NULL;
  }
  regfree(&__inotify_regex);
#endif
}


void
SkillerExecutionThread::proc_inotify()
{
#ifdef HAVE_INOTIFY
  if ( ! __cfg_watch_files )  return;

  bool need_restart = false;

  // Check for inotify events
  pollfd ipfd;
  ipfd.fd = __inotify_fd;
  ipfd.events = POLLIN;
  ipfd.revents = 0;
  int prv = poll(&ipfd, 1, 0);
  if ( prv == -1 ) {
    logger->log_error("SkillerExecutionThread", "inotify poll failed: %s (%i)",
		      strerror(errno), errno);
  } else while ( prv > 0 ) {
    // Our fd has an event, we can read
    if ( ipfd.revents & POLLERR ) {      
      logger->log_error("SkillerExecutionThread", "inotify poll error");
    } else {
      // must be POLLIN
      int bytes = 0, i = 0;
      if ((bytes = read(__inotify_fd, __inotify_buf, __inotify_bufsize)) != -1) {
	while (i < bytes) {
	  struct inotify_event *event = (struct inotify_event *) &__inotify_buf[i];

	  if ( (regexec(&__inotify_regex, event->name, 0, NULL, 0) == 0) ||
	       (event->mask & IN_ISDIR) ) {
	    // it is a directory or a *.lua file

	    if (event->mask & IN_DELETE_SELF) {
	      logger->log_debug("SkillerExecutionThread", "Watched %s has been deleted", event->name);
	      __inotify_watches.erase(event->wd);
	      inotify_rm_watch(__inotify_fd, event->wd);
	    }
	    if (event->mask & IN_MODIFY) {
	      logger->log_debug("SkillerExecutionThread", "%s has been modified", event->name);
	    }
	    if (event->mask & IN_MOVE) {
	      logger->log_debug("SkillerExecutionThread", "%s has been moved", event->name);
	    }
	    if (event->mask & IN_DELETE) {
	      logger->log_debug("SkillerExecutionThread", "%s has been deleted", event->name);
	    }
	    if (event->mask & IN_CREATE) {
	      logger->log_debug("SkillerExecutionThread", "%s has been created", event->name);
	      // Check if it is a directory, if it is, watch it
	      std::string fp = __inotify_watches[event->wd] + "/" + event->name;
	      if (  (event->mask & IN_ISDIR) && (event->name[0] != '.') ) {
		try {
		  inotify_watch_dir(fp);
		} catch (Exception &e) {
		  logger->log_warn("SkillerExecutionThread", "Adding watch for %s failed, ignoring.", fp.c_str());
		  logger->log_warn("SkillerExecutionThread", e);
		}
	      } else {
		logger->log_debug("SkillerExecutionThread", "Ignoring non-dir %s", event->name);
	      }
	    }

	    need_restart = true;
	  }

	  i += sizeof(struct inotify_event) + event->len;
	}
      } else {
	logger->log_error("SkillerExecutionThread", "inotify failed to read any bytes");
      }
    }

    prv = poll(&ipfd, 1, 0);
  }

  if ( need_restart ) {
    logger->log_info("SkillerExecutionThread", "inotify event triggers Lua restart");
    restart_lua();
  }

#else
  logger->log_error("SkillerExecutionThread", "inotify support not available, but "
		   "proc_inotify() was called. Ignoring.");
#endif
}
