/***************************************************************************
 *  main.cpp - Fawkes main application
 *
 *  Generated: Thu Nov  2 16:44:48 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <mainapp/main_thread.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>

// for -C: bb_cleanup
#include <utils/ipc/shm.h>
#include <blackboard/shmem_header.h>
#include <blackboard/shmem_lister.h>
#include <blackboard/bbconfig.h>

#include <iostream>

using namespace std;

/** Fawkes main application.
 *
 * @author Tim Niemueller
 */
class FawkesMainApp : public SignalHandler
{
 public:

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

    fmt->start();
    fmt->join();

    delete fmt;
  }

  /** Handle signals.
   * @param signum signal number
   */
  void handle_signal(int signum)
  {
    if ( (signum == SIGINT) ||
	 (signum == SIGTERM) ) {
      fmt->cancel();
    }
  }

 private:
  FawkesMainThread *fmt;
};


void
usage(const char *progname)
{
  cout << "Fawkes Main Application - Usage Instructions" << endl
       << "=========================================================================" << endl
       << "Call with: " << progname << " [options]" << endl
       << "where [options] is one or more of:" << endl
       << " -H             these help instructions" << endl
       << " -C             cleanup old BB segments" << endl
       << " -c conffile    mutable configuration file, created if it does not exist" << endl
       << "                if it does however it must contain valid SQLite database" << endl
       << " -d conffile    default configuration file, created if it does not exist" << endl
       << "                if it does however it must contain valid SQLite database" << endl
       << " -q[qqq]        Quiet mode, -q omits debug, -qq debug and info,"
       << "                -qqq omit debug, info and warn, -qqqq no output of logger" << endl
       << "                if it does however it must contain valid SQLite database" << endl
       << " -l level       set log level directly mutually exclusive with -q" << endl
       << "                level is one of debug, info, warn, error and none" << endl
       << " -L loggers     define loggers. By default this setting is read from " << endl
       << "                config file (or console logger if unset in config)." << endl
       << "                format for loggers is: logger:args[;logger2:args2[!...]]" << endl
       << "                the loggeroptions depend on the logger. Currently supported are:" << endl
       << "                  console, file:file.log" << endl
       << endl;
}


/** Fawkes application.
 * @param argc argument count
 * @param argv array of arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "HCc:d:q::l:L:");

  if ( argp->hasArgument("H") ) {
    usage(argv[0]);
    delete argp;
    return 0;
  }

  if ( argp->hasArgument("C") ) {
    BlackBoardSharedMemoryHeader *bbsh = new BlackBoardSharedMemoryHeader( BLACKBOARD_MEMORY_SIZE,
									   BLACKBOARD_VERSION );
    BlackBoardSharedMemoryLister *bblister = new BlackBoardSharedMemoryLister();
    SharedMemory::erase_orphaned(BLACKBOARD_MAGIC_TOKEN, bbsh, bblister);
    delete bblister;
    delete bbsh;
  }

  FawkesMainApp fawkes;
  SignalManager::register_handler(SIGINT, &fawkes);
  SignalManager::register_handler(SIGTERM, &fawkes);

  try {
    fawkes.run(argp);
  } catch (Exception &e) {
    printf("Running Fawkes failed\n");
    e.printTrace();
  }

  delete argp;
  return 0;
}
