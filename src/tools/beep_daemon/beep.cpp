
/***************************************************************************
 *  beep.cpp - Beeper utility class
 *
 *  Created: Sun Apr 11 19:41:23 2010
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

#include <cstdio>
#include <cerrno>
#include <termios.h>
#include <sys/types.h>
#include <fcntl.h>
#include <cstring>
#include <sys/ioctl.h>
#include <linux/kd.h>
#include <unistd.h>


/* From console_ioctl man page, explained in the beep too by
 * Johnathan Nightingale:
 * This number represents the fixed frequency of the original PC XT's
 * timer chip (the 8254 AFAIR), which is approximately 1.193 MHz. This
 * number is divided with the desired frequency to obtain a counter value,
 * that is subsequently fed into the timer chip, tied to the PC speaker.
 * The chip decreases this counter at every tick (1.193 MHz) and when it
 * reaches zero, it toggles the state of the speaker (on/off, or in/out),
 * resets the counter to the original value, and starts over. The end
 * result of this is a tone at approximately the desired frequency. :)
 */
#define CLOCK_TICK_RATE 1193180

#define CONSOLE_FILE "/dev/console"

/** @class BeepController "beep.h"
 * Simple speaker beep controller.
 * @author Tim Niemueller
 */

/** Constructor. */
BeepController::BeepController()
{
  __disable_beeping = false;
}

/** Enable beeping.
 * @param freq frequency to beep with
 */
void
BeepController::beep_on(float freq)
{
  if (__disable_beeping)  return;
    
  int beep_fd = open(CONSOLE_FILE, O_WRONLY);
  if (beep_fd == -1) {
    char errstr[1024];
    strerror_r(errno, errstr, sizeof(errstr));
    //logger->log_warn(name(), "Could not open console (%s). "
    //		     "Disabling warning beeps.", errstr);
    __disable_beeping = true;
  } else {
    if (ioctl(beep_fd, KIOCSOUND, (int)(CLOCK_TICK_RATE/freq)) < 0) {
      //logger->log_warn(name(), "Starting to beep failed. Disabling warning beeps.");
      __disable_beeping = true;
    }
    close(beep_fd);
  }
}


/** Disable beeping. */
void
BeepController::beep_off()
{
  if (__disable_beeping)  return;

  int beep_fd = open(CONSOLE_FILE, O_WRONLY);
  if (beep_fd == -1) {
    char errstr[1024];
    strerror_r(errno, errstr, sizeof(errstr));
    //logger->log_warn(name(), "Could not open console (%s) [stop]. "
    //		     "Disabling warning beeps.", errstr);
    __disable_beeping = true;
  } else {
    if (ioctl(beep_fd, KIOCSOUND, 0) < 0) {
      //logger->log_warn(name(), "Stopping beeping failed. "
      //	       "Disabling warning beeps.");
      __disable_beeping = true;
    }
    close(beep_fd);
  }
}
