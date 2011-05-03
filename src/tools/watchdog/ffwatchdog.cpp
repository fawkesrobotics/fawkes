
/***************************************************************************
 *  ffwatchdog.cpp - Fawkes process watchdog
 *
 *  Created: Thu Mar 31 09:53:53 2011 (RoboCup German Open 2011)
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>

#include <unistd.h>
#include <sys/wait.h>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <cerrno>

#ifdef HAVE_LIBDAEMON
#  include <cerrno>
#  include <cstring>
#  include <libdaemon/dfork.h>
#  include <libdaemon/dlog.h>
#  include <libdaemon/dpid.h>
#  include <sys/stat.h>
#  include <sys/wait.h>
#endif

int  g_quit = 0;
bool g_force_quit = false;
int  g_signum = SIGINT;

void
handle_signal(int signum)
{
  printf("Received %s signal\n", strsignal(signum));
  g_signum = signum;
  switch (signum) {
  case SIGINT:   g_quit += 1; break; // sigint escalates
  case SIGTERM:  g_quit  = 3; break;
  case SIGKILL:  g_quit  = 4; break;
  default: break;
  }
}

/** Print usage instructions.
 * @param progname program name
 */
void
usage(const char *progname)
{
  printf("Usage: %s [options] <progfile> [args...]\n"
	 "progfile   full absolute path to executable\n"
	 "args       any number of arguments, passed to program as-is\n\n"
	 "where [options] passed in before <progfile> are one or more of:\n"
#ifdef HAVE_LIBDAEMON
	 " -D[pid file]     Run daemonized in the background, pid file is optional,\n"
	 "                  defaults to /var/run/ffwatchdog_basename.pid, must be absolute path.\n"
	 " -D[pid file] -k  Kill a daemonized process running in the background,\n"
	 "                  pid file is optional as above.\n"
	 " -D[pid file] -s  Check status of daemon.\n"
#endif
	 " -h               Show help instructions.\n\n",
	 progname);
}


pid_t
fork_and_exec(int argc, char **argv, int prog_start)
{
  pid_t pid = fork();
  if (pid == -1) {
    // error
    printf("Forking for new process failed: %s\n", strerror(errno));
    throw fawkes::Exception(errno, "Forking for new process failed: %s");
  } else if (pid == 0) {
    // child
    setsid();
    signal(SIGINT, SIG_IGN);    
    if (execve(argv[prog_start], &argv[prog_start], environ) == -1) {
      printf("Failed to execute %s, exited with %i: %s\n",
	     argv[prog_start], errno, strerror(errno));
      exit(-1);
    }
  }

  return pid;
}


#ifdef HAVE_LIBDAEMON
void
daemonize_cleanup()
{
  daemon_retval_send(-1);
  daemon_retval_done();
  daemon_pid_file_remove();}

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
      daemon_log(LOG_ERR, "Failed to close all file descriptors: %s",
		 strerror(errno));
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
const char *ffwatchdog_pid_file;

/** Function that returns the PID file name.
 * @return PID file name
 */
const char *
ffwatchdog_daemon_pid_file_proc()
{
  return ffwatchdog_pid_file;
}
#endif // HAVE_LIBDAEMON



/** Watchdog main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  if (argc < 2) {
    usage(argv[0]);
    exit(1);
  }

  bool arg_verbose = false;
  bool arg_daemonize = false;
  bool arg_daemon_kill = false;
  bool arg_daemon_status = false;
  const char *daemon_pid_file = NULL;

  int prog_start;
  for (prog_start = 1; prog_start < argc; ++prog_start) {
    if (argv[prog_start][0] == '-') {
      // argument starts
      char param = argv[prog_start][1];
      if (param == '-') {
	++prog_start;
	break;
      } else {
	if (param == 'D') {
	  arg_daemonize = true;
	  daemon_pid_file = NULL;
	  if (strlen(&argv[prog_start][1]) > 1) {
	    daemon_pid_file = &argv[prog_start][2];
	  }
	} else if (param == 'k') {
	  arg_daemon_kill = true;
	} else if (param == 's') {
	  arg_daemon_status = true;
	} else if (param == 'v') {
	  arg_verbose = true;
	} else if (param == 'h') {
	  usage(argv[0]);
	  exit(0);
	} else {
	  printf("Unknown argument '%c'\n", param);
	  usage(argv[0]);
	  exit(3);
	}
      }
    } else {
      break;
    }
  }

  if (prog_start >= argc) {
    usage(argv[0]);
    exit(1);
  }

  if (access(argv[prog_start], X_OK) != 0) {
    printf("Cannot execute '%s': %s\n\n", argv[1], strerror(errno));
    usage(argv[0]);
    exit(2);
  }

#ifdef HAVE_LIBDAEMON
  pid_t dpid;
  int ret;

  char *daemon_ident = NULL;

  if ( arg_daemonize ) {
    // Set identification string for the daemon for both syslog and PID file

    char *argv_copy = strdup(argv[prog_start]);
    if (asprintf(&daemon_ident, "ffwatchdog_%s", basename(argv_copy)) == -1) {
      free(argv_copy);
      printf("Failed to create daemon ident, not enough memory\n");
      exit(5);
    }
    free(argv_copy);
    daemon_pid_file_ident = daemon_log_ident = daemon_ident;
    if ( daemon_pid_file != NULL ) {
      ffwatchdog_pid_file  = daemon_pid_file;
      daemon_pid_file_proc = ffwatchdog_daemon_pid_file_proc;
    }

    // We should daemonize, check if we were called to kill a daemonized copy
    if (arg_daemon_kill) {
      // Check that the daemon is not run twice a the same time
      if ((dpid = daemon_pid_file_is_running()) < 0) {
	daemon_log(LOG_ERR, "Watchdog daemon for %s not running.",
		   argv[prog_start]);
	return 1;
      }

      // Kill daemon with SIGINT
      if ((ret = daemon_pid_file_kill_wait(SIGINT, 5)) < 0) {
	daemon_log(LOG_WARNING, "Failed to kill watchdog daemon for %s",
		   argv[prog_start]);
      }
      return (ret < 0) ? 1 : 0;
    }

    if (arg_daemon_status) {
      // Check daemon status
      if (daemon_pid_file_is_running() < 0) {
	if (arg_verbose) {
	  printf("Watchdog daemon for %s is not running\n", argv[prog_start]);
	}
	return 1;
      } else {
	if (arg_verbose) {
	  printf("Watchdog daemon for %s is running\n", argv[prog_start]);
	}
	return 0;
      }
    }

    // Check that the daemon is not run twice a the same time
    if ((dpid = daemon_pid_file_is_running()) >= 0) {
      daemon_log(LOG_ERR, "Watchdog daemon for %s already running on (PID %u)",
		 argv[prog_start], dpid);
      return 201;
    }

    dpid = daemonize(argc, argv);
    if ( dpid < 0 ) {
      daemonize_cleanup();
      return 201;
    } else if (dpid) {
      // parent
      return 0;
    } // else child, continue as usual
  }
#else
  if (daemonize) {
    printf("Daemonize support was not available at compile time.\n");
    exit(13);
  }
#endif

  struct sigaction sa;
  sa.sa_handler = handle_signal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGKILL, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGUSR1, &sa, NULL);
  sigaction(SIGUSR2, &sa, NULL);

  pid_t pid = -1;
  while (! g_quit) {
    pid = fork_and_exec(argc, argv, prog_start);

    while (pid != -1 && ! g_quit) {

      int status = 0;
      pid_t cpid = waitpid(pid, &status, WUNTRACED | WCONTINUED);
      printf("Wait returned\n");

      if (cpid == -1) {
	printf("Failed to wait for child: %s\n", strerror(errno));
      } else if (WIFEXITED(status)) {
	printf("%i|%s exited, status=%d\n", cpid, argv[prog_start],
	       WEXITSTATUS(status));
	pid = -1;
      } else if (WIFSIGNALED(status)) {
	printf("%i|%s killed by signal %s\n", cpid, argv[prog_start],
	       strsignal(WTERMSIG(status)));
	pid = -1;
      } else if (WIFSTOPPED(status)) {
	printf("%i|%s stopped by signal %s\n", cpid, argv[prog_start],
	       strsignal(WSTOPSIG(status)));
	pid = -1;
      } else if (WIFCONTINUED(status)) {
	printf("%i|%s continued\n", cpid, argv[prog_start]);
      }
    }
  }

  if (pid != -1) {

    int last_quit = 0;
    printf("Stopping child. Press Ctrl-C again to escalate.\n");

    for (unsigned int i = 0; i < 600; ++i) {
      if (last_quit != g_quit) {
	int signum;
	if (g_quit <= 2) {
	  signum = SIGINT;
	} else if (g_quit == 3) {
	  signum = SIGTERM;
	} else {
	  signum = SIGKILL;
	}

	printf("Killing %s with signal %s\n", argv[prog_start],
	       strsignal(signum));
	if (kill(pid, signum) == -1) {
	  printf("Failed to kill %s: %s\n", argv[prog_start], strerror(errno));
	}
      }
      last_quit = g_quit;

      usleep(10000);
      int status;
      int rv = waitpid(pid, &status, WNOHANG);
      if (rv == -1) {
	if (errno == EINTR)  continue;
	if (errno == ECHILD) {
	  pid = -1;
	  break;
	}
      } else if (rv > 0) {
	pid = -1;
	break;
      }
      if (i >= 300) g_quit = 2;
      if (i >= 500) g_quit = 3;
    }
  }

#ifdef HAVE_LIBDAEMON
  if (arg_daemonize) {
    daemonize_cleanup();
  }
#endif

  return 0;
}
