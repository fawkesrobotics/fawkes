
/***************************************************************************
 *  main.cpp - main application for front camera computer vision pipeline
 *
 *  Generated: Fri May 20 14:25:28 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include <iostream>

#include <apps/fountain/config.h>
#include <apps/fountain/fountain.h>

#include <utils/system/argparser.h>
#include <utils/system/console_colors.h>
#include <utils/config_reader/config_reader.h>
#include <utils/logging/console.h>

using namespace std;

int
main( int argc, char **argv )
{

  // see help output below for the meaning of the parameters
  ArgumentParser *argp = new ArgumentParser(argc, argv, "d:x:Hb");
  ConsoleLogger *logger = new ConsoleLogger();

  if (argp->hasArgument("H")) {

    cout << endl << cblue << "FireFountain Help" << cnormal << endl << endl
	 << " -d delay  Delay in ms between image sends" << endl
	 << " -x file   use named XML config file" << endl
	 << " -H        Shows this help" << endl
	 << endl;

  } else {

    cout << cblue << "FireFountain" << cnormal << ": Running pipeline directly" << endl;

    CConfigReader *cr = new CConfigReader();

    char *config_filename;
    if ( (config_filename = argp->getArgument("x")) == NULL ) {
      config_filename = "../cfg/rcsoftconfigfile.xml";
      cout << cblue << "FireFountain" << cnormal
	   << ": No config file given, using default ("
	   << config_filename << ")" << endl;
    }
    cr->LoadConfFile( config_filename );
    RCSoftXFountainConfig *c = new RCSoftXFountainConfig( cr );

    char *tmp;
    int delay = 0;
    if ( (tmp = argp->getArgument("d")) != NULL ) {
      delay = atoi(tmp);
      if (delay < 0) {
	cout << cblue << "FireFountain: " << cred << "Invalid delay. Resetting to 0" << cnormal << endl;
	delay = 0;
      }
    }

    cout << cblue << "FireFountain" << cnormal << ": Delay is " << delay << " ms" << endl;

    Fountain *fountain = new Fountain(argp, logger, c->ImageDelay, c->FountainPort);

    fountain->init();
    fountain->run();
    fountain->finalize();

    // causes segfault, already deleted otherwise, probably by signal
    // delete fountain;
    delete cr;
    delete c;

  }

  delete argp;

}

/// @endcond
