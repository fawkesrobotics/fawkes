
/***************************************************************************
 *  force_feedback.cpp - Force feedback for joysticks using Linux input API
 *
 *  Created: Mon Feb 07 01:35:29 2011 (Super Bowl XLV)
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

#include "force_feedback.h"

#include <fnmatch.h>
#include <sys/types.h>
#include <dirent.h>
#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

#include <core/exception.h>

#define BITS_PER_LONG (sizeof(long) * 8)
#define NBITS(x) ((((x)-1)/BITS_PER_LONG)+1)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)  ((array[LONG(bit)] >> OFF(bit)) & 1)

using namespace fawkes;

/** @class JoystickForceFeedback "force_feedback.h"
 * Cause force feedback on a joystick.
 * An instance of this class opens an input device which belongs to
 * the given device name. It searches all input devices to find the
 * correct device file. Once opened, it detects the available features
 * of the joystick and provides conventient access to it allowing for
 * rumbling effects, for instance.
 * @author Tim Niemueller
 *
 * @fn bool JoystickForceFeedback::is_rumbling()
 * Check if rumbling effect is active.
 * @return true if effect is active, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_rumble()
 * Check if rumbling effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_periodic()
 * Check if periodic effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_constant()
 * Check if constant effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_spring()
 * Check if spring effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_friction()
 * Check if friction effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_damper()
 * Check if damper effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_inertia()
 * Check if inertia effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_ramp()
 * Check if ramp effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_square()
 * Check if square effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_triangle()
 * Check if triangle effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_sine()
 * Check if sine effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_saw_up()
 * Check if upward saw effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_saw_down()
 * Check if downward saw effect is supported.
 * @return true if effect is supported, false otherwise
 *
 * @fn bool JoystickForceFeedback::can_custom()
 * Check if custom effect is supported.
 * @return true if effect is supported, false otherwise
 *
 */

/** Constructor.
 * @param device_name device name, note that this is not the device
 * file, but rather the event files are tried and the device name is
 * compared.
 */
JoystickForceFeedback::JoystickForceFeedback(const char *device_name)
{
  __fd = -1;

  DIR *d = opendir("/dev/input");

  if (d == NULL) {
    throw Exception("Could not open directory /dev/input");
  }

  struct dirent *de;
  while ((de = readdir(d)) != NULL) {
    if (fnmatch("event*", de->d_name, 0) != FNM_NOMATCH) {
      char *path;
      if (asprintf(&path, "/dev/input/%s", de->d_name) == -1) {
        continue;
      }

      __fd = open(path, O_RDWR);
      if (__fd == -1) {
        free(path);
        continue;
      }
      free(path);

      char name[256]= "Unknown";
      if(ioctl(__fd, EVIOCGNAME(sizeof(name)), name) < 0) {
	close(__fd);
	__fd = -1;
	continue;
      }

      if (strcmp(name, device_name) != 0) {
	close(__fd);
	__fd = -1;
	continue;
      }

      long features[NBITS(EV_MAX)];
      memset(features, 0, sizeof(features));
      if (ioctl(__fd, EVIOCGBIT(0, EV_MAX), features) < 0) {
	close(__fd);
	__fd = -1;
	throw Exception("Cannot get feedback feature vector");
      }

      if (! test_bit(EV_FF, features)) {
	close(__fd);
	__fd = -1;
	throw Exception("Device '%s' does not support force-feedback", device_name);
      }

      long ff_features[NBITS(FF_MAX)];
        
      memset(ff_features, 0, sizeof(ff_features));
      if (ioctl(__fd, EVIOCGBIT(EV_FF, FF_MAX), ff_features) < 0) {
	close(__fd);
	__fd = -1;
	throw Exception("Cannot get device force feedback feature vector");
      }

      long no_ff_features[NBITS(FF_MAX)];
      memset(no_ff_features, 0, sizeof(no_ff_features));
      if (memcmp(ff_features, no_ff_features, sizeof(no_ff_features)) == 0) {
	close(__fd);
	__fd = -1;
	throw Exception("Device has no force feedback features");
      }

      __can_rumble   = test_bit(FF_RUMBLE, ff_features);
      __can_periodic = test_bit(FF_PERIODIC, ff_features);
      __can_constant = test_bit(FF_CONSTANT, ff_features);
      __can_spring   = test_bit(FF_SPRING, ff_features);
      __can_friction = test_bit(FF_FRICTION, ff_features);
      __can_damper   = test_bit(FF_DAMPER, ff_features);
      __can_inertia  = test_bit(FF_INERTIA, ff_features);
      __can_ramp     = test_bit(FF_RAMP, ff_features);
      __can_square   = test_bit(FF_SQUARE, ff_features);
      __can_triangle = test_bit(FF_TRIANGLE, ff_features);
      __can_sine     = test_bit(FF_SINE, ff_features);
      __can_saw_up   = test_bit(FF_SAW_UP, ff_features);
      __can_saw_down = test_bit(FF_SAW_DOWN, ff_features);
      __can_custom   = test_bit(FF_CUSTOM, ff_features);
      
      if (ioctl(__fd, EVIOCGEFFECTS, &__num_effects) < 0) {
	__num_effects = 1;
      }

      break;
    }
  }

  closedir(d);

  if (__fd == -1) {
    throw Exception("Force feedback joystick '%s' not found", device_name);
  }

  memset(&__rumble, 0, sizeof(__rumble));
  __rumble.type = FF_RUMBLE;
  __rumble.id   = -1;
}


/** Destructor. */
JoystickForceFeedback::~JoystickForceFeedback()
{
  close(__fd);
}


/** Rumble the joystick.

 * This is the most basic force feedback for example in force feedback
 * joypads. Often such joysticks provide two effect magnitudes, a
 * strong heavier motor for larger effects, and a smaller one for
 * vibrating effects.
 * @param strong_magnitude magnitude to use on the larger motor
 * @param weak_magnitude magnitude to use on the smaller motor
 * @param direction direction of the effect, meaningful on joysticks
 * (rather than joypads)
 * @param length length of the effect in ms
 * @param delay delay before the effect starts in ms
 */
void
JoystickForceFeedback::rumble(uint16_t strong_magnitude, uint16_t weak_magnitude,
			      Direction direction, uint16_t length, uint16_t delay)
{
  if ( (__rumble.id == -1) ||
       (__rumble.u.rumble.strong_magnitude != strong_magnitude) ||
       (__rumble.u.rumble.weak_magnitude != weak_magnitude) ||
       (__rumble.direction != direction) ||
       (__rumble.replay.length != length) ||
       (__rumble.replay.delay != length) )
  {
    // we need to upload
    __rumble.u.rumble.strong_magnitude = strong_magnitude;
    __rumble.u.rumble.weak_magnitude   = weak_magnitude;
    __rumble.direction = direction;
    __rumble.replay.length = length;
    __rumble.replay.delay = delay;

    if (ioctl(__fd, EVIOCSFF, &__rumble) < 0) {
      throw Exception("Failed to upload rumble effect");
    }
  }

  struct input_event play;
  play.type  = EV_FF;
  play.code  = __rumble.id;
  play.value = 1;

  if (write(__fd, &play, sizeof(play)) < 0) {
    throw Exception("Failed to start rumble effect");
  }
}


/** Stop rumbling. */
void
JoystickForceFeedback::stop_rumble()
{
  if (__rumble.id != -1) {
    if (ioctl(__fd, EVIOCRMFF, __rumble.id) < 0) {
      throw Exception("Failed to stop rumble effect");
    }
    __rumble.id = -1;
  }
}


/** Stop all current effects. */
void
JoystickForceFeedback::stop_all()
{
  stop_rumble();
}
