
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

bool g_quit = false;
int  g_signum = SIGINT;

void
handle_signal(int signum)
{
  printf("Received %s signal\n", strsignal(signum));
  if (signum == SIGINT || signum == SIGTERM || signum == SIGKILL) {
    g_signum = signum;
    g_quit = true;
  }
}

/** Print usage instructions.
 * @param progname program name
 */
void
usage(const char *progname)
{
  printf("Usage: %s <progfile> [args...]\n"
	 "progfile   full absolute path to executable\n"
	 "args       any number of arguments, passed to program as-is\n",
	 progname);
}


pid_t
fork_and_exec(int argc, char **argv)
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
    if (execve(argv[1], &argv[1], environ) == -1) {
      printf("Failed to execute %s, exited with %i: %s\n",
	     argv[1], errno, strerror(errno));
      exit(-1);
    }
  }

  return pid;
}



/** Watchdog main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  if (argc == 1) {
    usage(argv[0]);
    exit(1);
  }

  if (access(argv[1], X_OK) != 0) {
    printf("Cannot execute '%s': %s\n\n", argv[1], strerror(errno));
    usage(argv[0]);
    exit(2);
  }

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
    pid = fork_and_exec(argc, argv);

    while (pid != -1 && ! g_quit) {

      int status = 0;
      pid_t cpid = waitpid(pid, &status, WUNTRACED | WCONTINUED);
      printf("Wait returned\n");

      if (cpid == -1) {
	printf("Failed to wait for child: %s\n", strerror(errno));
      } else if (WIFEXITED(status)) {
	printf("%i|%s exited, status=%d\n", cpid, argv[1], WEXITSTATUS(status));
	pid = -1;
      } else if (WIFSIGNALED(status)) {
	printf("%i|%s killed by signal %s\n", cpid, argv[1],
	       strsignal(WTERMSIG(status)));
	pid = -1;
      } else if (WIFSTOPPED(status)) {
	printf("%i|%s stopped by signal %s\n", cpid, argv[1],
	       strsignal(WSTOPSIG(status)));
	pid = -1;
      } else if (WIFCONTINUED(status)) {
	printf("%i|%s continued\n", cpid, argv[1]);
      }
    }
  }

  if (pid != -1) {
    printf("Killing %s with signal %s\n", argv[1], strsignal(g_signum));
    if (kill(pid, g_signum) == -1) {
      printf("Failed to kill %s: %s\n", argv[1], strerror(errno));
    }
  }

  return 0;
}
