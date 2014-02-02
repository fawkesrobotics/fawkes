
/***************************************************************************
 *  daemonize.cpp - Fawkes daemonization functions
 *
 *  Created: Wed May 04 23:33:33 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <baseapp/daemonize.h>

#include <utils/system/argparser.h>

#include <unistd.h>
#include <sys/types.h>
#include <cstdio>
#ifdef HAVE_LIBDAEMON
#  include <cerrno>
#  include <cstring>
#  include <csignal>
#  include <libdaemon/dfork.h>
#  include <libdaemon/dlog.h>
#  include <libdaemon/dpid.h>
#  include <sys/stat.h>
#  include <sys/wait.h>
#endif

namespace fawkes {
  namespace daemon {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

#ifdef HAVE_LIBDAEMON
/** Global variable containing the path to the PID file.
 * unfortunately needed for libdaemon */
const char *fawkes_pid_file;

/** Function that returns the PID file name.
 * @return PID file name
 */
const char *
fawkes_daemon_pid_file_proc()
{
  return fawkes_pid_file;
}
#endif // HAVE_LIBDAEMON


pid_t
daemonize()
{
#ifdef HAVE_LIBDAEMON
  pid_t pid;
  mode_t old_umask = umask(0);

  // Prepare for return value passing
  daemon_retval_init();

  // Do the fork
  if ((pid = daemon_fork()) < 0) {
    return -1;
        
  } else if (pid) { // the parent
    int ret;

    // Wait for 20 seconds for the return value passed from the daemon process
    if ((ret = daemon_retval_wait(20)) < 0) {
      daemon_log(LOG_ERR, "Could not recieve return value from daemon process.");
      return -1;
    }

    if ( ret != 0 ) {
      daemon_log(LOG_ERR, "*** Daemon startup failed, see syslog for details. ***");
      switch (ret) {
      case 1:
	daemon_log(LOG_ERR, "Daemon failed to close file descriptors");
	break;
      case 2:
	daemon_log(LOG_ERR, "Daemon failed to create PID file");
	break;
      }
      return -1;
    } else {
      return pid;
    }

  } else { // the daemon
#ifdef DAEMON_CLOSE_ALL_AVAILABLE
    if (daemon_close_all(-1) < 0) {
      daemon_log(LOG_ERR, "Failed to close all file descriptors: %s", strerror(errno));
      // Send the error condition to the parent process
      daemon_retval_send(1);
      return -1;
    }
#endif

    // Create the PID file
    if (daemon_pid_file_create() < 0) {
      printf("Could not create PID file (%s).", strerror(errno));
      daemon_log(LOG_ERR, "Could not create PID file (%s).", strerror(errno));

      // Send the error condition to the parent process
      daemon_retval_send(2);
      return -1;
    }

    // Send OK to parent process
    daemon_retval_send(0);

    daemon_log(LOG_INFO, "Sucessfully started");

    umask(old_umask);
    return 0;
  }
#else
  throw Exception("Daemonizing support is not available.\n"
		  "(libdaemon[-devel] was not available at compile time)\n");
#endif
}


void
init(const char *pidfile, const char *progname)
{
#ifdef HAVE_LIBDAEMON
  // Set identification string for the daemon for both syslog and PID file
  daemon_pid_file_ident = daemon_log_ident =
    daemon_ident_from_argv0((char *)progname);
  if ( pidfile != NULL ) {
    fawkes_pid_file      = pidfile;
    daemon_pid_file_proc = fawkes_daemon_pid_file_proc;
  }
#else
  throw Exception("Daemonizing support is not available.\n"
		  "(libdaemon[-devel] was not available at compile time)\n");
#endif
}

bool
start()
{
#ifdef HAVE_LIBDAEMON
  pid_t pid;

  // Check that the daemon is not run twice a the same time
  if ((pid = daemon_pid_file_is_running()) >= 0) {
    daemon_log(LOG_ERR, "Daemon already running on (PID %u)", pid);
    throw Exception("Daemon already running on (PID %u)", pid);
  }

  pid = daemonize();
  if ( pid < 0 ) {
    cleanup();
    throw Exception("Failed to daemonize");
  } else if (pid) {
    // parent
    return true;
  } else {
    // child
    return false;
  }
#else
  throw Exception("Daemonizing support is not available.\n"
		  "(libdaemon[-devel] was not available at compile time)\n");
#endif
}

bool
running()
{
#ifdef HAVE_LIBDAEMON
  return (daemon_pid_file_is_running() >= 0);
#else
  throw Exception("Daemonizing support is not available.\n"
		  "(libdaemon[-devel] was not available at compile time)\n");
#endif
}


void
kill()
{
#ifdef HAVE_LIBDAEMON
  pid_t pid;
  int ret;

  // Check that the daemon is not run twice a the same time
  if ((pid = daemon_pid_file_is_running()) < 0) {
    daemon_log(LOG_WARNING, "Fawkes daemon not running.");
  }

  // Kill daemon with SIGINT
  if ((ret = daemon_pid_file_kill_wait(SIGINT, 5)) < 0) {
    daemon_log(LOG_WARNING, "Failed to kill daemon");
  }

  daemon_pid_file_remove();
#else
  throw Exception("Daemonizing support is not available.\n"
		  "(libdaemon[-devel] was not available at compile time)\n");
#endif
}


void
cleanup()
{
#ifdef HAVE_LIBDAEMON
  daemon_retval_send(-1);
  daemon_retval_done();
  daemon_pid_file_remove();
#else
  throw Exception("Daemonizing support is not available.\n"
		  "(libdaemon[-devel] was not available at compile time)\n");
#endif
}



} // end namespace fawkes::daemon
} // end namespace fawkes
