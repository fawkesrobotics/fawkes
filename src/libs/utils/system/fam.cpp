
/***************************************************************************
 *  fam.h - File Alteration Monitor
 *
 *  Created: Fri May 23 11:38:41 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/fam.h>
#include <core/exception.h>

#ifdef HAVE_INOTIFY
#  include <sys/inotify.h>
#  include <sys/stat.h>
#  include <poll.h>
#  include <dirent.h>
#  include <unistd.h>
#  include <cstring>
#endif
#include <cerrno>
#include <cstdlib>

namespace fawkes {

/* Supported events suitable for MASK parameter of INOTIFY_ADD_WATCH.  */
/** File was accessed.  */
const unsigned int FamListener::FAM_ACCESS        = 0x00000001;
/** File was modified.  */
const unsigned int FamListener::FAM_MODIFY	  = 0x00000002;
/** Metadata changed.  */
const unsigned int FamListener::FAM_ATTRIB	  = 0x00000004;
/** Writtable file was closed.  */
const unsigned int FamListener::FAM_CLOSE_WRITE	  = 0x00000008;
/** Unwrittable file closed.  */
const unsigned int FamListener::FAM_CLOSE_NOWRITE = 0x00000010;
/** Close.  */
const unsigned int FamListener::FAM_CLOSE	  = (FAM_CLOSE_WRITE | FAM_CLOSE_NOWRITE);
/** File was opened.  */
const unsigned int FamListener::FAM_OPEN	  = 0x00000020;
/** File was moved from X.  */
const unsigned int FamListener::FAM_MOVED_FROM	  = 0x00000040;
/** File was moved to Y.  */
const unsigned int FamListener::FAM_MOVED_TO      = 0x00000080;
/** Moves.  */
const unsigned int FamListener::FAM_MOVE          = (FAM_MOVED_FROM | FAM_MOVED_TO);
/** Subfile was created.  */
const unsigned int FamListener::FAM_CREATE	  = 0x00000100;
/** Subfile was deleted.  */
const unsigned int FamListener::FAM_DELETE	  = 0x00000200;
/** Self was deleted.  */
const unsigned int FamListener::FAM_DELETE_SELF	  = 0x00000400;
/** Self was moved.  */
const unsigned int FamListener::FAM_MOVE_SELF	  = 0x00000800;

/* Events sent by the kernel.  */
/** Backing fs was unmounted.  */
const unsigned int FamListener::FAM_UNMOUNT	  = 0x00002000;
/** Event queued overflowed.  */
const unsigned int FamListener::FAM_Q_OVERFLOW	  = 0x00004000;
/** File was ignored.  */
const unsigned int FamListener::FAM_IGNORED	  = 0x00008000;

/* Special flags.  */
/** Only watch the path if it is a directory.  */
const unsigned int FamListener::FAM_ONLYDIR	  = 0x01000000;
/** Do not follow a sym link.  */
const unsigned int FamListener::FAM_DONT_FOLLOW	  = 0x02000000;
/** Add to the mask of an already existing watch.  */
const unsigned int FamListener::FAM_MASK_ADD	  = 0x20000000;
/** Event occurred against dir.  */
const unsigned int FamListener::FAM_ISDIR	  = 0x40000000;
/** Only send event once.  */
const unsigned int FamListener::FAM_ONESHOT	  = 0x80000000;

/** All events which a program can wait on.  */
const unsigned int FamListener::FAM_ALL_EVENTS	   = (FAM_ACCESS | FAM_MODIFY | FAM_ATTRIB | FAM_CLOSE_WRITE \
						      | FAM_CLOSE_NOWRITE | FAM_OPEN | FAM_MOVED_FROM \
						      | FAM_MOVED_TO | FAM_CREATE | FAM_DELETE \
						      | FAM_DELETE_SELF | FAM_MOVE_SELF);


/** @class FileAlterationMonitor <utils/system/fam.h>
 * Monitors files for changes.
 * This is a wrapper around inotify. It will watch directories and files
 * for modifications. If a modifiacation, removal or addition of a file
 * is detected one or more listeners are called. The files which trigger
 * the event can be constrained with regular expressions.
 * @author Tim Niemueller
 */

/** Constructor.
 * Opens the inotify context.
 */
FileAlterationMonitor::FileAlterationMonitor()
{
#ifdef HAVE_INOTIFY
  if ( (__inotify_fd = inotify_init()) == -1 ) {
    throw Exception(errno, "Failed to initialize inotify");
  }

  // from http://www.linuxjournal.com/article/8478
  __inotify_bufsize = 1024 * (sizeof(struct inotify_event) + 16);
  __inotify_buf     = (char *)malloc(__inotify_bufsize);
#endif

  __interrupted   = false;
  __interruptible = (pipe(__pipe_fds) == 0);

  __regexes.clear();
}


/** Destructor. */
FileAlterationMonitor::~FileAlterationMonitor()
{
  for (__rxit = __regexes.begin(); __rxit != __regexes.end(); ++__rxit) {
    regfree(*__rxit);
    free(*__rxit);
  }

#ifdef HAVE_INOTIFY
  for (__inotify_wit = __inotify_watches.begin(); __inotify_wit != __inotify_watches.end(); ++__inotify_wit) {
    inotify_rm_watch(__inotify_fd, __inotify_wit->first);
  }
  close(__inotify_fd);
  if ( __inotify_buf ) {
    free(__inotify_buf);
    __inotify_buf = NULL;
  }
#endif
}


/** Watch a directory.
 * This adds the given directory recursively to this FAM.
 * @param dirpath path to directory to add
 */
void
FileAlterationMonitor::watch_dir(const char *dirpath)
{
#ifdef HAVE_INOTIFY
  DIR *d = opendir(dirpath);
  if ( d == NULL ) {
    throw Exception(errno, "Failed to open dir %s", dirpath);
  }

  uint32_t mask = IN_MODIFY | IN_MOVE | IN_CREATE | IN_DELETE | IN_DELETE_SELF;
  int iw;

  //LibLogger::log_debug("FileAlterationMonitor", "Adding watch for %s", dirpath);
  if ( (iw = inotify_add_watch(__inotify_fd, dirpath, mask)) >= 0) {
    __inotify_watches[iw] = dirpath;

    dirent de, *res;
    while ( (readdir_r(d, &de, &res) == 0) && (res != NULL) ) {
      std::string fp = std::string(dirpath) + "/" + de.d_name;
      struct stat st;
      if ( stat(fp.c_str(), &st) == 0 ) {
	if ( (de.d_name[0] != '.') && S_ISDIR(st.st_mode) ) {
	  try {
	    watch_dir(fp.c_str());
	  } catch (Exception &e) {
	    closedir(d);
	    throw;
	  }
	//} else {
	  //LibLogger::log_debug("SkillerExecutionThread", "Skipping file %s", fp.c_str());	  
	}
      } else {
	//LibLogger::log_debug("FileAlterationMonitor",
	//		     "Skipping watch on %s, cannot stat (%s)",
	//		     fp.c_str(), strerror(errno));
      }
    }
  } else {
    throw Exception(errno, "FileAlterationMonitor: cannot add watch for %s", dirpath);
  }

  closedir(d);
#endif
}

/** Watch a file.
 * This adds the given fileto this FAM.
 * @param filepath path to file to add
 */
void
FileAlterationMonitor::watch_file(const char *filepath)
{
#ifdef HAVE_INOTIFY
  uint32_t mask = IN_MODIFY | IN_MOVE | IN_CREATE | IN_DELETE | IN_DELETE_SELF;
  int iw;

  //LibLogger::log_debug("FileAlterationMonitor", "Adding watch for %s", dirpath);
  if ( (iw = inotify_add_watch(__inotify_fd, filepath, mask)) >= 0) {
    __inotify_watches[iw] = filepath;
  } else {
    throw Exception("FileAlterationMonitor: cannot add watch for file %s", filepath);
  }
#endif
}


/** Remove all currently active watches. */
void
FileAlterationMonitor::reset()
{
#ifdef HAVE_INOTIFY
  std::map<int, std::string>::iterator wit;
  for (wit = __inotify_watches.begin(); wit != __inotify_watches.end(); ++wit) {
    inotify_rm_watch(__inotify_fd, wit->first);
  }
  __inotify_watches.clear();
#endif
}

/** Add a filter.
 * Filters are applied to path names that triggered an event. All
 * pathnames are checked against this regex and if any does not match
 * the event is not posted to listeners.
 * An example regular expression is
 * @code
 * ^[^.].*\\.lua$
 * @endcode
 * This regular expression matches to all files that does not start with
 * a dot and have an .lua ending.
 * @param regex regular expression to add
 */
void
FileAlterationMonitor::add_filter(const char *regex)
{
  int regerr = 0;
  regex_t *rx = (regex_t *)malloc(sizeof(regex_t));
  if ( (regerr = regcomp(rx, regex, REG_EXTENDED)) != 0 ) {
    char errtmp[1024];
    regerror(regerr, rx, errtmp, sizeof(errtmp));
    free(rx);
    throw Exception("Failed to compile lua file regex: %s", errtmp);
  }
  __regexes.push_back_locked(rx);
}


/** Add a listener.
 * @param listener listener to add
 */
void
FileAlterationMonitor::add_listener(FamListener *listener)
{
  __listeners.push_back_locked(listener);
}


/** Remove a listener.
 * @param listener listener to remove
 */
void
FileAlterationMonitor::remove_listener(FamListener *listener)
{
  __listeners.remove_locked(listener);
}


/** Process events.
 * Call this when you want file events to be processed.
 * @param timeout timeout in milliseconds to wait for an event, 0 to just check
 * and no wait, -1 to wait forever until an event is received
 */
void
FileAlterationMonitor::process_events(int timeout)
{
#ifdef HAVE_INOTIFY
  // Check for inotify events
  __interrupted = false;
  pollfd ipfd[2];
  ipfd[0].fd = __inotify_fd;
  ipfd[0].events = POLLIN;
  ipfd[0].revents = 0;
  ipfd[1].fd = __pipe_fds[0];
  ipfd[1].events = POLLIN;
  ipfd[1].revents = 0;
  int prv = poll(ipfd, 2, timeout);
  if ( prv == -1 ) {
    if ( errno != EINTR ) {
      //LibLogger::log_error("FileAlterationMonitor",
	//		   "inotify poll failed: %s (%i)",
	//		   strerror(errno), errno);
    } else {
      __interrupted = true;
    }
  } else while ( !__interrupted && (prv > 0) ) {
    // Our fd has an event, we can read
    if ( ipfd[0].revents & POLLERR ) {      
      //LibLogger::log_error("FileAlterationMonitor", "inotify poll error");
    } else if (__interrupted) {
      // interrupted
      return;
    } else {
      // must be POLLIN
      int bytes = 0, i = 0;
      if ((bytes = read(__inotify_fd, __inotify_buf, __inotify_bufsize)) != -1) {
	while (!__interrupted && (i < bytes)) {
	  struct inotify_event *event = (struct inotify_event *) &__inotify_buf[i];
	  
	  bool valid = true;
	  if (! (event->mask & IN_ISDIR)) {
	    for (__rxit = __regexes.begin(); __rxit != __regexes.end(); ++__rxit) {
	      if (regexec(*__rxit, event->name, 0, NULL, 0) == REG_NOMATCH ) {
		//LibLogger::log_debug("FileAlterationMonitor", "A regex did not match for %s", event->name);
		valid = false;
		break;
	      }
	    }
	  }

	  /*
	  if (event->mask & IN_MODIFY) {
	    LibLogger::log_debug("FileAlterationMonitor", "%s has been modified", event->name);
	    }
	  if (event->mask & IN_MOVE) {
	    LibLogger::log_debug("FileAlterationMonitor", "%s has been moved", event->name);
	    }
	  if (event->mask & IN_DELETE) {
	    LibLogger::log_debug("FileAlterationMonitor", "%s has been deleted", event->name);
	  }
	  if (event->mask & IN_CREATE) {
	    LibLogger::log_debug("FileAlterationMonitor", "%s has been created", event->name);
	  }
	  */

	  if ( valid ) {
	    for (__lit = __listeners.begin(); __lit != __listeners.end(); ++__lit) {
	      (*__lit)->fam_event(event->name, event->mask);
	    }
	  }

	  if (event->mask & IN_DELETE_SELF) {
	    //LibLogger::log_debug("FileAlterationMonitor", "Watched %s has been deleted", event->name);
	    __inotify_watches.erase(event->wd);
	    inotify_rm_watch(__inotify_fd, event->wd);
	  }

	  if (event->mask & IN_CREATE) {
	    // Check if it is a directory, if it is, watch it
	    std::string fp = __inotify_watches[event->wd] + "/" + event->name;
	    if (  (event->mask & IN_ISDIR) && (event->name[0] != '.') ) {
	      /*
	      LibLogger::log_debug("FileAlterationMonitor",
				   "Directory %s has been created, "
				   "adding to watch list", event->name);
	      */
	      try {
		watch_dir(fp.c_str());
	      } catch (Exception &e) {
		//LibLogger::log_warn("FileAlterationMonitor", "Adding watch for %s failed, ignoring.", fp.c_str());
		//LibLogger::log_warn("FileAlterationMonitor", e);
	      }
	    }
	  }

	  i += sizeof(struct inotify_event) + event->len;
	}
      } else {
	//LibLogger::log_error("FileAlterationMonitor", "inotify failed to read any bytes");
      }
    }

    prv = poll(ipfd, 2, 0);
  }
#else
  //LibLogger::log_error("FileAlterationMonitor",
//		       "inotify support not available, but "
//		       "process_events() was called. Ignoring.");
  throw Exception("FileAlterationMonitor: inotify support not available, "
		  "but process_events() was called.");
#endif
}


/** Interrupt a running process_events().
 * This method will interrupt e.g. a running inifinetly blocking call of
 * process_events().
 */
void
FileAlterationMonitor::interrupt()
{
  if (__interruptible) {
    __interrupted = true;
    char tmp = 0;
    if (write(__pipe_fds[1], &tmp, 1) != 1) {
      throw Exception(errno, "Failed to interrupt file alteration monitor,"
			     " failed to write to pipe");
    }
  } else {
    throw Exception("Currently not interruptible");
  }
}


/** @class FamListener <utils/system/fam.h>
 * File Alteration Monitor Listener.
 * Listener called by FileAlterationMonitor for events.
 * @author Tim Niemueller
 *
 * @fn FamListener::fam_event(const char *filename, unsigned int mask)
 * Event has been raised.
 * @param filename name of the file that triggered the event
 * @param mask mask indicating the event. Currently inotify event flags
 * are used, see inotify.h.
 *
 */

/** Virtual empty destructor. */
FamListener::~FamListener()
{
}

} // end of namespace fawkes
