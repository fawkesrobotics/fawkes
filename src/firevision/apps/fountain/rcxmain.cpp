
/***************************************************************************
 *  rcxmain.cpp - Main application for Fountain in RCSoftX
 *
 *  Created: Fri May 20 14:25:28 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

/// @cond RCSoftX

#include <apps/fountain/config.h>
#include <fvutils/net/fuse_server.h>

#include <utils/system/argparser.h>
#include <utils/system/console_colors.h>
#include <utils/system/signal.h>
#include <utils/config_reader/config_reader.h>
#include <utils/logging/console.h>

#ifdef HAVE_AVAHI
#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/dns-sd/avahi_service_publisher.h>
#include <netcomm/dns-sd/avahi_browser.h>
#include <netcomm/service_discovery/service.h>
#include <netcomm/utils/resolver.h>
#endif

#include <iostream>

using namespace std;

class FountainRCSoftX : public SignalHandler
{
 public:
  FountainRCSoftX(int argc, char **argv)
  {
    __argp   = new ArgumentParser(argc, argv, "Hx:");
    __logger = new ConsoleLogger();

#ifdef HAVE_AVAHI
    __avahi_thread          = new AvahiThread();
    __avahi_thread->start();
    __nnresolver = new NetworkNameResolver(__avahi_thread);
#endif

    __cr = NULL;
    __config = NULL;
    __fuse_server = NULL;
  }

  ~FountainRCSoftX()
  {
#ifdef HAVE_AVAHI
    __avahi_thread->cancel();
    __avahi_thread->join();
    delete __nnresolver;
    delete __avahi_thread;
#endif

    delete __argp;
    delete __logger;
    delete __cr;
    delete __config;
    delete __fuse_server;
  }

  virtual void handle_signal(int signal)
  {
    if ( __fuse_server ) {
      __fuse_server->cancel();
    }
  }

  void
  print_usage()
  {
    printf("Usage: %s [-x config] [-H]\n"
	   "  -H             Show this help message\n"
	   "  -x config      Load given RCSoftX config file\n",
	   __argp->program_name());
  }

  void run()
  {
    if ( __argp->has_arg("H") ) {
      print_usage();
      return;
    }

    __cr = new CConfigReader();

    const char *config_filename;
    if ( (config_filename = __argp->arg("x")) == NULL ) {
      config_filename = "../cfg/config.xml";
      cout << cblue << "FireVision Fountain" << cnormal
	   << ": No config file given, using default ("
	   << config_filename << ")" << endl;
    }
    __cr->LoadConfFile( config_filename );
    __config = new RCSoftXFountainConfig( __cr );

    __fuse_server = new FuseServer(__config->FountainPort);
    __fuse_server->start();

#ifdef HAVE_AVAHI
    char *fountain_service_name;
    asprintf(&fountain_service_name, "Fountain on %s", __nnresolver->short_hostname());
    NetworkService *fountain_service = new NetworkService(fountain_service_name,
							  "_fountain._tcp",
							  __config->FountainPort);
    free(fountain_service_name);
    __avahi_thread->publish(fountain_service);
#endif

    __fuse_server->join();
  }


 private:
  ArgumentParser        *__argp;
  ConsoleLogger         *__logger;
  CConfigReader         *__cr;
  RCSoftXFountainConfig *__config;
  FuseServer            *__fuse_server;
#ifdef HAVE_AVAHI
  AvahiThread           *__avahi_thread;
  NetworkNameResolver   *__nnresolver;
#endif
};


int
main( int argc, char **argv )
{
  FountainRCSoftX f(argc, argv);
  SignalManager::register_handler(SIGINT, &f);
  f.run();
  SignalManager::finalize();
}

/// @endcond
