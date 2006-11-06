
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

/** Fawkes main application.
 *
 * @author Tim Niemueller
 */
class FawkesMainApp : public SignalHandler
{
 public:
  /** Run main thread.
   * @param argc argument count
   * @param argv array of arguments
   */
  void run(int argc, char **argv)
  {
    fmt = new FawkesMainThread();

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


/** Fawkes application.
 * @param argc argument count
 * @param argv array of arguments
 */
int
main(int argc, char **argv)
{
  FawkesMainApp fawkes;
  SignalManager::register_handler(SIGINT, &fawkes);
  SignalManager::register_handler(SIGTERM, &fawkes);

  fawkes.run(argc, argv);
}
