
/***************************************************************************
 *  main.cpp - Fawkes main application
 *
 *  Created: Thu Nov  2 16:44:48 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <mainapp/main_thread.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

#include <iostream>
#ifdef HAVE_LIBDAEMON
#  include <cerrno>
#  include <cstring>
#  include <libdaemon/dfork.h>
#  include <libdaemon/dlog.h>
#  include <libdaemon/dpid.h>
#  include <sys/types.h>
#  include <sys/stat.h>
#endif

using namespace std;
using namespace fawkes;

/** Fawkes main application.
 *
 * @author Tim Niemueller
 */
class FawkesMainApp : public SignalHandler
{
 public:

  /** Constructor. */
  FawkesMainApp()
  {
    __init_running = true;
    __init_quit    = false;
  }

  /** Run main thread.
   * @param argp argument parser
   */
  void run(ArgumentParser *argp)
  {
    try {
      fmt = new FawkesMainThread(argp);
    } catch (Exception &e) {
      throw;
    }

    __init_mutex.lock();
    __init_running = false;
    if ( ! __init_quit ) {
      fmt->start();
      __init_mutex.unlock();
      fmt->join();
    } else {
      __init_mutex.unlock();
    }

    delete fmt;
  }

  /** Handle signals.
   * @param signum signal number
   */
  void handle_signal(int signum)
  {
    if ( (signum == SIGINT) ||
	 (signum == SIGTERM) ) {
      MutexLocker lock(&__init_mutex);
      if (__init_running) {
	__init_quit = true;
      } else {
	fmt->cancel();
      }
    }
  }

 private:
  FawkesMainThread *fmt;
  Mutex             __init_mutex;
  bool              __init_running;
  bool              __init_quit;
};


void
usage(const char *progname)
{
  cout << "Fawkes Main Application - Usage Instructions" << endl
       << "===============================================================================" << endl
       << "Call with: " << progname << " [options]" << endl
       << "where [options] is one or more of:" << endl
       << " -h               These help instructions" << endl
       << " -C               Cleanup old BB segments" << endl
       << " -c conffile      Mutable configuration file, created if it does not exist" << endl
       << "                  if it does however it must contain valid SQLite database" << endl
       << " -d conffile      Default configuration file, created if it does not exist" << endl
       << "                  if it does however it must contain valid SQLite database" << endl
       << " -q[qqq]          Quiet mode, -q omits debug, -qq debug and info," << endl
       << "                  -qqq omit debug, info and warn, -qqqq no output of logger" << endl
       << " -l level         Set log level directly mutually exclusive with -q" << endl
       << "                  level is one of debug, info, warn, error and none" << endl
       << " -L loggers       Define loggers. By default this setting is read from " << endl
       << "                  config file (or console logger if unset in config)." << endl
       << "                  format for loggers is: logger:args[;logger2:args2[!...]]" << endl
       << "                  the loggeroptions depend on the logger. Currently supported:" << endl
       << "                  console (default), file:file.log, network logger always starts" << endl
       << " -p plugins       Comma-separated list of plugins, for example " << endl
       << "                  fvbase,fvfountain,fvretriever. These plugins will be loaded" << endl
       << "                  in the given order after startup." << endl
#ifdef HAVE_LIBDAEMON
       << " -D[pid file]     Run daemonized in the background, pid file is optional, " << endl
       << "                  defaults to /var/run/fawkes.pid, must be absolute path." << endl
       << " -D[pid file] -k  Kill a daemonized process running in the background," << endl
       << "                  pid file is optional as above." << endl
       << " -D[pid file] -s  Check status of daemon." << endl
#endif
       << endl;
}


#ifdef HAVE_LIBDAEMON
void
daemonize_cleanup()
{
  daemon_retval_send(-1);
  daemon_retval_done();
  daemon_pid_file_remove();
}

pid_t
daemonize(int argc, char **argv)
{
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
    if (daemon_close_all(-1) < 0) {
      daemon_log(LOG_ERR, "Failed to close all file descriptors: %s", strerror(errno));
      // Send the error condition to the parent process
      daemon_retval_send(1);
      return -1;
    }

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
}

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

/** Fawkes application.
 * @param argc argument count
 * @param argv array of arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "hCc:d:q::l:L:p:D::ks");

#ifdef HAVE_LIBDAEMON
  pid_t pid;
  int ret;

  if ( argp->has_arg("D") ) {
    // Set identification string for the daemon for both syslog and PID file
    daemon_pid_file_ident = daemon_log_ident = daemon_ident_from_argv0(argv[0]);
    if ( argp->arg("D") != NULL ) {
      fawkes_pid_file      = argp->arg("D");
      daemon_pid_file_proc = fawkes_daemon_pid_file_proc;
    }

    // We should daemonize, check if we were called to kill a daemonized copy
    if ( argp->has_arg("k") ) {
      // Check that the daemon is not run twice a the same time
      if ((pid = daemon_pid_file_is_running()) < 0) {
	daemon_log(LOG_ERR, "Fawkes daemon not running.");
	return 1;
      }

      // Kill daemon with SIGINT
      if ((ret = daemon_pid_file_kill_wait(SIGINT, 5)) < 0) {
	daemon_log(LOG_WARNING, "Failed to kill daemon");
      }
      return (ret < 0) ? 1 : 0;
    }

    if ( argp->has_arg("s") ) {
      // Check daemon status
      return (daemon_pid_file_is_running() < 0);
    }

    // Check that the daemon is not run twice a the same time
    if ((pid = daemon_pid_file_is_running()) >= 0) {
      daemon_log(LOG_ERR, "Daemon already running on PID file %u", pid);
      return 201;
    }

    pid = daemonize(argc, argv);
    if ( pid < 0 ) {
      daemonize_cleanup();
      return 201;
    } else if (pid) {
      // parent
      return 0;
    } // else child, continue as usual
  }
#else
  if ( argp->has_arg("D") ) {
    printf("Daemonizing support is not available.\n"
	   "(libdaemon[-devel] was not available at compile time)\n");
    return 202;
  }
#endif

  Thread::init_main();

  if ( argp->has_arg("h") ) {
    usage(argv[0]);
    delete argp;
    return 0;
  }

  FawkesMainApp fawkes;
  SignalManager::register_handler(SIGINT, &fawkes);
  SignalManager::register_handler(SIGTERM, &fawkes);

  try {
    fawkes.run(argp);
  } catch (Exception &e) {
    printf("Running Fawkes failed\n");
    e.print_trace();
  }

  Thread::destroy_main();

#ifdef HAVE_LIBDAEMON
  if ( argp->has_arg("D") ) {
    daemonize_cleanup();
  }
#endif

  delete argp;
  return 0;
}
