
/***************************************************************************
 *  ffkbjoystick.cpp - Keyboard joystick emulation
 *
 *  Created: Sat Jan 29 12:04:07 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "remote_bb_poster.h"

#include <core/exceptions/system.h>
#include <utils/system/argparser.h>
#include <logging/console.h>
#include <utils/system/getkey.h>
#include <utils/time/time.h>

#include <cstdlib>
#include <cstdio>

#include <interfaces/JoystickInterface.h>

using namespace fawkes;

bool quit = false;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] [-r host[:port]]\n"
	 " -h              This help message\n"
	 " -r host[:port]  Remote host (and optionally port) to connect to\n"
	 " -d device       Joystick device to use\n"
	 " -l              Start in logging mode - print data read from bb\n",
	 program_name);
}



/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  try {
    ArgumentParser argp(argc, argv, "hr:");
    
    if ( argp.has_arg("h") ) {
      print_usage(argv[0]);
      exit(0);
    }
    ConsoleLogger logger;

    char *host = (char *)"localhost";
    unsigned short int port = 1910;
    bool free_host = argp.parse_hostport("r", &host, &port);

    JoystickRemoteBlackBoardPoster jbp(host, port, &logger);

    jbp.joystick_plugged(3, 10);
    float axis[3], new_axis[3];
    unsigned int button, new_button;
    Time last, now;

    axis[0] = axis[1] = 0.;
    axis[2] = 0.5;
    new_axis[0] = new_axis[1] = new_axis[2] = 0.;
    button = new_button = 0;

    last.stamp();

    char key = 0;
    int wait_time = 5;
    while (key != 'q') {
      key = getkey(wait_time);
      //printf("Key: %u = %u\n", key, key);
      if (key != 0) {
	now.stamp();
	if ( (now - &last) < 0.5) {
	  wait_time = 1;
	}
	last.stamp();
      }
      if (key == 0) {
	wait_time = 5;
	new_axis[0] = new_axis[1] = 0;
	new_button = 0;

      } else if (key == 27) {
	key = getkey();
	if (key == 0) {
	  // Escape key
	  new_axis[0] = new_axis[1] = 0;
	  new_button = 0;
	} else {
	  if (key != 91) continue;

	  key = getkey();
	  if (key == 0) continue;

	  switch (key) {
	  case 65: new_axis[0] = +1.; break;
	  case 66: new_axis[0] = -1.; break;
	  case 67: new_axis[1] = -1.; break;
	  case 68: new_axis[1] = +1.; break;
	  default: continue;
	  }
	}
      } else if (key == '+') {
	if ((axis[2] + 0.1) <= 1.0) {
	  new_axis[2] += 0.1;
	} else {
	  new_axis[2]  = 1.;
	}
      } else if (key == '-') {
	if ((axis[2] - 0.1) >= 0.) {
	  new_axis[2] -= 0.1;
	} else {
	  new_axis[2]  = 0.;
	}
      } else if (key == '1') {
	new_button = JoystickInterface::BUTTON_1;
      } else if (key == ' ') {
	new_button = JoystickInterface::BUTTON_1;
      } else if (key == '2') {
	new_button = JoystickInterface::BUTTON_2;
      } else if (key == '3') {
	new_button = JoystickInterface::BUTTON_3;
      } else if (key == '4') {
	new_button = JoystickInterface::BUTTON_4;
      } else if (key == '5') {
	new_button = JoystickInterface::BUTTON_5;
      } else if (key == '6') {
	new_button = JoystickInterface::BUTTON_6;
      } else if (key == '7') {
	new_button = JoystickInterface::BUTTON_7;
      } else if (key == '8') {
	new_button = JoystickInterface::BUTTON_8;
      } else if (key == '9') {
	new_button = JoystickInterface::BUTTON_9;
      } else if (key == '0') {
	new_button = JoystickInterface::BUTTON_10;
      }

      if ((axis[0] != new_axis[0]) || (axis[1] != new_axis[1]) ||
	  (axis[2] != new_axis[2]) || (button != new_button))
      {
	axis[0] = new_axis[0];
	axis[1] = new_axis[1];
	axis[2] = new_axis[2];
	button    = new_button;
	jbp.joystick_changed(button, axis);
      }
    }

    jbp.joystick_unplugged();

    if (free_host)  free(host);

  } catch (UnknownArgumentException &e) {
    printf("Error: Unknown Argument\n\n");
    print_usage(argv[0]);
    exit(0);
  }

  return 0;
}
