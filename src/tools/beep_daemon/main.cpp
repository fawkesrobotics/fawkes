
/***************************************************************************
 *  main.cpp - Fawkes main application
 *
 *  Created: Sun Apr 11 19:34:09 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "beep.h"

#include <core/threading/thread.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>
#include <blackboard/remote.h>
#include <interfaces/SwitchInterface.h>
#include <utils/time/time.h>

#include <cstdio>
#include <unistd.h>
#include <cmath>
#ifdef HAVE_LIBDAEMON
#  include <cerrno>
#  include <cstring>
#  include <libdaemon/dfork.h>
#  include <libdaemon/dlog.h>
#  include <libdaemon/dpid.h>
#  include <sys/stat.h>
#  include <sys/wait.h>
#endif

using namespace std;
using namespace fawkes;

/** Fawkes beep daemon.
 *
 * @author Tim Niemueller
 */
class FawkesBeepDaemon
  : public Thread,
    public SignalHandler
{
 public:
  /** Constructor. */
  FawkesBeepDaemon()
    : Thread("FawkesBeepDaemon", Thread::OPMODE_CONTINUOUS)
  {
    __until     = NULL;
    __bb        = NULL;
    __switch_if = NULL;
  }

  virtual void loop()
  {
    while (! (__bb && __bb->is_alive() && __switch_if->is_valid())) {
      if (__bb) {
	printf("Lost connection to blackboard\n");
	__bb->close(__switch_if);
	delete __bb;
	__bb = NULL;
      }
      try {
	printf("Trying to connect to remote BB...");
	__bb = new RemoteBlackBoard("localhost", 1910);
	__switch_if = __bb->open_for_writing<SwitchInterface>("Beep");
	printf("succeeded\n");
      } catch (Exception &e) {
	printf("failed\n");
	delete __bb;
	__bb = NULL;
	sleep(5);
      }
    }

    if (__until) {
      Time now;
      if ((now - __until) >= 0) {
	__beep.beep_off();
	delete __until;
	__until = NULL;
      }
    }

    while (! __switch_if->msgq_empty()) {
      if (  __switch_if->msgq_first_is<SwitchInterface::SetMessage>() ) {
	SwitchInterface::SetMessage *msg = __switch_if->msgq_first<SwitchInterface::SetMessage>();
	if (msg->value() > 0.0) {
	  __beep.beep_on(msg->value());
	} else if (msg->is_enabled()) {
	  __beep.beep_on();
	} else {
	  __beep.beep_off();
	}

      } else if (  __switch_if->msgq_first_is<SwitchInterface::EnableDurationMessage>() ) {
	SwitchInterface::EnableDurationMessage *msg =
	  __switch_if->msgq_first<SwitchInterface::EnableDurationMessage>();
	float duration = fabs(msg->duration());
	float value    = fabs(msg->value());

	delete __until;
	__until = new Time();
	*__until += duration;
	__beep.beep_on(value);
      } else if (__switch_if->msgq_first_is<SwitchInterface::EnableSwitchMessage>() ) {
	__beep.beep_on();
      } else if (__switch_if->msgq_first_is<SwitchInterface::DisableSwitchMessage>() ) {
	__beep.beep_off();
      }
      
      __switch_if->msgq_pop();
    }

    usleep(10000);
  }


  /** Handle signals.
   * @param signum signal number
   */
  void handle_signal(int signum)
  {
    this->cancel();
  }

 private:
  BeepController __beep;
  BlackBoard *__bb;
  SwitchInterface *__switch_if;

  Time *__until;
};


void
usage(const char *progname)
{
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
  ArgumentParser *argp = new ArgumentParser(argc, argv, "hD::ks");

  // default user/group
  const char *user  = NULL;
  const char *group = NULL;
  if (argp->has_arg("u")) {
    user = argp->arg("u");
  }
  if (argp->has_arg("g")) {
    group = argp->arg("g");
  }

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
      daemon_log(LOG_ERR, "Daemon already running on (PID %u)", pid);
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

  FawkesBeepDaemon beepd;
  SignalManager::register_handler(SIGINT, &beepd);
  SignalManager::register_handler(SIGTERM, &beepd);

  beepd.start();
  beepd.join();

  Thread::destroy_main();

#ifdef HAVE_LIBDAEMON
  if ( argp->has_arg("D") ) {
    daemonize_cleanup();
  }
#endif

  delete argp;
  return 0;
}
