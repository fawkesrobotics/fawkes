
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
    fmt = new FawkesMainThread(argp);

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
       << " -c conffile    mutable configuration file, created if it does not exist" << endl
       << "                if it does however it must contain valid SQLite database" << endl
       << " -d conffile    default configuration file, created if it does not exist" << endl
       << "                if it does however it must contain valid SQLite database" << endl
       << endl;
}

/** Fawkes application.
 * @param argc argument count
 * @param argv array of arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "Hc:d:");

  if ( argp->hasArgument("H") ) {
    usage(argv[0]);
    delete argp;
    return 0;
  }

  FawkesMainApp fawkes;
  SignalManager::register_handler(SIGINT, &fawkes);
  SignalManager::register_handler(SIGTERM, &fawkes);

  fawkes.run(argp);

  delete argp;
  return 0;
}
