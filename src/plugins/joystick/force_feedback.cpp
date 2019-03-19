
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

#include <core/exception.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fnmatch.h>
#include <unistd.h>

#define BITS_PER_LONG (sizeof(long) * 8)
#define NBITS(x) ((((x)-1) / BITS_PER_LONG) + 1)
#define OFF(x) ((x) % BITS_PER_LONG)
#define BIT(x) (1UL << OFF(x))
#define LONG(x) ((x) / BITS_PER_LONG)
#define test_bit(bit, array) ((array[LONG(bit)] >> OFF(bit)) & 1)

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
	fd_ = -1;

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

			fd_ = open(path, O_RDWR);
			if (fd_ == -1) {
				free(path);
				continue;
			}
			free(path);

			char name[256] = "Unknown";
			if (ioctl(fd_, EVIOCGNAME(sizeof(name)), name) < 0) {
				close(fd_);
				fd_ = -1;
				continue;
			}

			if (strcmp(name, device_name) != 0) {
				close(fd_);
				fd_ = -1;
				continue;
			}

			long features[NBITS(EV_MAX)];
			memset(features, 0, sizeof(features));
			if (ioctl(fd_, EVIOCGBIT(0, EV_MAX), features) < 0) {
				close(fd_);
				fd_ = -1;
				throw Exception("Cannot get feedback feature vector");
			}

			if (!test_bit(EV_FF, features)) {
				close(fd_);
				fd_ = -1;
				throw Exception("Device '%s' does not support force-feedback", device_name);
			}

			long ff_features[NBITS(FF_MAX)];

			memset(ff_features, 0, sizeof(ff_features));
			if (ioctl(fd_, EVIOCGBIT(EV_FF, FF_MAX), ff_features) < 0) {
				close(fd_);
				fd_ = -1;
				throw Exception("Cannot get device force feedback feature vector");
			}

			long no_ff_features[NBITS(FF_MAX)];
			memset(no_ff_features, 0, sizeof(no_ff_features));
			if (memcmp(ff_features, no_ff_features, sizeof(no_ff_features)) == 0) {
				close(fd_);
				fd_ = -1;
				throw Exception("Device has no force feedback features");
			}

			can_rumble_   = test_bit(FF_RUMBLE, ff_features);
			can_periodic_ = test_bit(FF_PERIODIC, ff_features);
			can_constant_ = test_bit(FF_CONSTANT, ff_features);
			can_spring_   = test_bit(FF_SPRING, ff_features);
			can_friction_ = test_bit(FF_FRICTION, ff_features);
			can_damper_   = test_bit(FF_DAMPER, ff_features);
			can_inertia_  = test_bit(FF_INERTIA, ff_features);
			can_ramp_     = test_bit(FF_RAMP, ff_features);
			can_square_   = test_bit(FF_SQUARE, ff_features);
			can_triangle_ = test_bit(FF_TRIANGLE, ff_features);
			can_sine_     = test_bit(FF_SINE, ff_features);
			can_saw_up_   = test_bit(FF_SAW_UP, ff_features);
			can_saw_down_ = test_bit(FF_SAW_DOWN, ff_features);
			can_custom_   = test_bit(FF_CUSTOM, ff_features);

			if (ioctl(fd_, EVIOCGEFFECTS, &num_effects_) < 0) {
				num_effects_ = 1;
			}

			break;
		}
	}

	closedir(d);

	if (fd_ == -1) {
		throw Exception("Force feedback joystick '%s' not found", device_name);
	}

	memset(&rumble_, 0, sizeof(rumble_));
	rumble_.type = FF_RUMBLE;
	rumble_.id   = -1;
}

/** Destructor. */
JoystickForceFeedback::~JoystickForceFeedback()
{
	close(fd_);
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
JoystickForceFeedback::rumble(uint16_t  strong_magnitude,
                              uint16_t  weak_magnitude,
                              Direction direction,
                              uint16_t  length,
                              uint16_t  delay)
{
	if ((rumble_.id == -1) || (rumble_.u.rumble.strong_magnitude != strong_magnitude)
	    || (rumble_.u.rumble.weak_magnitude != weak_magnitude) || (rumble_.direction != direction)
	    || (rumble_.replay.length != length) || (rumble_.replay.delay != length)) {
		// we need to upload
		rumble_.u.rumble.strong_magnitude = strong_magnitude;
		rumble_.u.rumble.weak_magnitude   = weak_magnitude;
		rumble_.direction                 = direction;
		rumble_.replay.length             = length;
		rumble_.replay.delay              = delay;

		if (ioctl(fd_, EVIOCSFF, &rumble_) < 0) {
			throw Exception("Failed to upload rumble effect");
		}
	}

	struct input_event play;
	play.type  = EV_FF;
	play.code  = rumble_.id;
	play.value = 1;

	if (write(fd_, &play, sizeof(play)) < 0) {
		throw Exception("Failed to start rumble effect");
	}
}

/** Stop rumbling. */
void
JoystickForceFeedback::stop_rumble()
{
	if (rumble_.id != -1) {
		if (ioctl(fd_, EVIOCRMFF, rumble_.id) < 0) {
			throw Exception("Failed to stop rumble effect");
		}
		rumble_.id = -1;
	}
}

/** Stop all current effects. */
void
JoystickForceFeedback::stop_all()
{
	stop_rumble();
}
