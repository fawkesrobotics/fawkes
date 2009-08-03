
/***************************************************************************
 *  qa_evid100p.cpp - QA for Sony EviD100P PTU
 *
 *  Created: Mon Jun 22 11:12:43 2009
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include "../sony/evid100p.h"
#include <utils/time/tracker.h>

#include <cstdio>
#include <unistd.h>

using namespace fawkes;

int
main(int argc, char **argv)
{
  SonyEviD100PVisca ptu("/dev/ttyUSB1", 100, false);

  for (int i = 0; i < 10; ++i) {
    ptu.process();
    usleep(100000);
  }

  printf("Min pan: %f  max pan: %f   min tilt: %f  max tilt: %f\n",
	 SonyEviD100PVisca::MIN_PAN_RAD, SonyEviD100PVisca::MAX_PAN_RAD,
	 SonyEviD100PVisca::MIN_TILT_RAD, SonyEviD100PVisca::MAX_TILT_RAD);

  float pan = 0, tilt = 0;
  ptu.get_pan_tilt_rad(pan, tilt);
  printf("Pan: %f, tilt: %f\n", pan, tilt);

  float panval  = SonyEviD100PVisca::MIN_PAN_RAD;
  float tiltval = SonyEviD100PVisca::MIN_TILT_RAD;

  float pan_smin, pan_smax, tilt_smin, tilt_smax;
  ptu.get_speed_limits(pan_smin, pan_smax, tilt_smin, tilt_smax);

  ptu.set_speed_radsec(pan_smax, tilt_smax);
  printf("Moving to %f, %f... ", panval, tiltval);
  ptu.set_pan_tilt_rad(panval, tiltval);
  while (! ptu.is_nonblocking_finished(SonyEviD100PVisca::NONBLOCKING_PANTILT)) {
    printf("."); fflush(stdout);
    usleep(10000);
    try {
      ptu.process();
    } catch (Exception &e) {}
  }
  printf("\n");

  sleep(1);

  ptu.set_speed_radsec(1.0, 0.8);

  panval *= -1; tiltval *= -1;
  printf("Moving to %f, %f... ", panval, tiltval);
  ptu.set_pan_tilt_rad(panval, tiltval);
  while (! ptu.is_nonblocking_finished(SonyEviD100PVisca::NONBLOCKING_PANTILT)) {
    printf("."); fflush(stdout);
    usleep(10000);
    try {
      ptu.process();
    } catch (Exception &e) {
      e.print_trace();
    }
  }

  /*
  TimeTracker tt;
  unsigned int ttc_full_pan = tt.add_class("Full pan");

  ptu.set_pan_tilt_rad(panval, tiltval);
  while (! ptu.is_nonblocking_finished(SonyEviD100PVisca::NONBLOCKING_PANTILT)) {
    printf("."); fflush(stdout);
    usleep(10000);
    ptu.process();
  }

  for (unsigned int s = SonyEviD100PVisca::MAX_TILT_SPEED; s > 0; --s) {
    tt.reset();
    ptu.set_pan_tilt_speed(SonyEviD100PVisca::MAX_PAN_SPEED, s);
    for (unsigned int i = 0; i < 6; ++i) {
      tiltval *= -1.0;
      tt.ping_start(ttc_full_pan);
      try {
	ptu.set_pan_tilt_rad(panval, tiltval);
	//printf("Waiting for setting of pan=%f  tilt=0 to finish... ", panval);
	fflush(stdout);
	while (! ptu.is_nonblocking_finished(SonyEviD100PVisca::NONBLOCKING_PANTILT)) {
	  //printf("."); fflush(stdout);
	  usleep(500);
	  ptu.process();
	}
	//printf("\n");
      } catch (Exception &e) {
	e.print_trace();
      }
      tt.ping_end(ttc_full_pan);
    }
    printf("Average panning time for speed %u\n", s);
    tt.print_to_stdout();
  }
  ptu.get_pan_tilt_rad(pan, tilt);
  printf("Pan: %f, tilt: %f\n", pan, tilt);
  */

  return 0;
}

/// @endcond
