
/***************************************************************************
 *  dp_ptu.cpp - Controller for Directed Perception, Inc. Pan-Tilt Unit on B21
 *
 *  Created: Wed Nov 29 23:05:49 2006
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

#include "dp_ptu.h"

#include <core/exceptions/system.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <utils/math/angle.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace fawkes;

/** @class DirectedPerceptionPTU "dp_ptu.h"
 * DirectedPerception PTU implementation.
 * Control object to use the DirectedPerception PTU Pan/Tilt unit mounted
 * on carl.
 *
 * @author Tim Niemueller
 */

const char *DirectedPerceptionPTU::DPPTU_PAN_ABSPOS             = "PP";
const char *DirectedPerceptionPTU::DPPTU_TILT_ABSPOS            = "TP";
const char *DirectedPerceptionPTU::DPPTU_PAN_RELPOS             = "PO";
const char *DirectedPerceptionPTU::DPPTU_TILT_RELPOS            = "TO";
const char *DirectedPerceptionPTU::DPPTU_PAN_RESOLUTION         = "PR";
const char *DirectedPerceptionPTU::DPPTU_TILT_RESOLUTION        = "TR";
const char *DirectedPerceptionPTU::DPPTU_PAN_MIN                = "PN";
const char *DirectedPerceptionPTU::DPPTU_PAN_MAX                = "PX";
const char *DirectedPerceptionPTU::DPPTU_TILT_MIN               = "TN";
const char *DirectedPerceptionPTU::DPPTU_TILT_MAX               = "TX";
const char *DirectedPerceptionPTU::DPPTU_LIMITENFORCE_QUERY     = "L";
const char *DirectedPerceptionPTU::DPPTU_LIMITENFORCE_ENABLE    = "LE";
const char *DirectedPerceptionPTU::DPPTU_LIMITENFORCE_DISABLE   = "LD";
const char *DirectedPerceptionPTU::DPPTU_IMMEDIATE_EXECUTION    = "I";
const char *DirectedPerceptionPTU::DPPTU_SLAVED_EXECUTION       = "S";
const char *DirectedPerceptionPTU::DPPTU_AWAIT_COMPLETION       = "A";
const char *DirectedPerceptionPTU::DPPTU_HALT_ALL               = "H";
const char *DirectedPerceptionPTU::DPPTU_HALT_PAN               = "HP";
const char *DirectedPerceptionPTU::DPPTU_HALT_TILT              = "HT";
const char *DirectedPerceptionPTU::DPPTU_PAN_SPEED              = "PS";
const char *DirectedPerceptionPTU::DPPTU_TILT_SPEED             = "TS";
const char *DirectedPerceptionPTU::DPPTU_PAN_ACCEL              = "PA";
const char *DirectedPerceptionPTU::DPPTU_TILT_ACCEL             = "TA";
const char *DirectedPerceptionPTU::DPPTU_PAN_BASESPEED          = "PB";
const char *DirectedPerceptionPTU::DPPTU_TILT_BASESPEED         = "TB";
const char *DirectedPerceptionPTU::DPPTU_PAN_UPPER_SPEED_LIMIT  = "PU";
const char *DirectedPerceptionPTU::DPPTU_PAN_LOWER_SPEED_LIMIT  = "PL";
const char *DirectedPerceptionPTU::DPPTU_TILT_UPPER_SPEED_LIMIT = "TU";
const char *DirectedPerceptionPTU::DPPTU_TILT_LOWER_SPEED_LIMIT = "TL";
const char *DirectedPerceptionPTU::DPPTU_RESET                  = "R";
const char *DirectedPerceptionPTU::DPPTU_STORE                  = "DS";
const char *DirectedPerceptionPTU::DPPTU_RESTORE                = "DR";
const char *DirectedPerceptionPTU::DPPTU_FACTORY_RESET          = "DF";
const char *DirectedPerceptionPTU::DPPTU_ECHO_QUERY             = "E";
const char *DirectedPerceptionPTU::DPPTU_ECHO_ENABLE            = "EE";
const char *DirectedPerceptionPTU::DPPTU_ECHO_DISABLE           = "ED";
const char *DirectedPerceptionPTU::DPPTU_ASCII_VERBOSE          = "FV";
const char *DirectedPerceptionPTU::DPPTU_ASCII_TERSE            = "FT";
const char *DirectedPerceptionPTU::DPPTU_ASCII_QUERY            = "F";
const char *DirectedPerceptionPTU::DPPTU_VERSION                = "V";

/** Constructor.
 * @param device_file serial device file (e.g. /dev/ttyS0)
 * @param timeout_ms timeout for read operations in miliseconds
 */
DirectedPerceptionPTU::DirectedPerceptionPTU(const char *device_file, unsigned int timeout_ms)
{
	device_file_ = strdup(device_file);
	opened_      = false;
	timeout_ms_  = timeout_ms;

	open();
}

/** Destructor. */
DirectedPerceptionPTU::~DirectedPerceptionPTU()
{
	close();
	free(device_file_);
}

void
DirectedPerceptionPTU::open()
{
	if (opened_)
		return;

	fd_ = ::open(device_file_, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (!fd_ || !isatty(fd_)) {
		throw Exception("Cannot open device or device is not a TTY");
	}

	struct termios param;

	if (tcgetattr(fd_, &param) != 0) {
		::close(fd_);
		throw Exception("DP PTU: Cannot get parameters");
		;
	}

	if (cfsetspeed(&param, B9600) == -1) {
		::close(fd_);
		throw Exception("DP PTU: Cannot set speed");
		;
	}

	cfsetospeed(&param, B9600);
	cfsetispeed(&param, B9600);

	// set serial line options
	param.c_cflag |= (CLOCAL | CREAD); // set to local and enable the receiver
	param.c_cflag &= ~CSIZE;           // mask character size bits
	param.c_cflag |= CS8;              // select 8 data bits
	param.c_cflag &= ~PARENB;          // no parity
	param.c_cflag &= ~CSTOPB;          // 1 stop bit

	// set input options
	param.c_iflag &= ~(INPCK | ISTRIP);       // no input parity checking
	param.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control

	param.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	param.c_cc[VTIME] = 1; // wait for a tenth of a second for data
	param.c_cc[VMIN]  = 0;

	if (tcsetattr(fd_, TCSANOW, &param) != 0) {
		::close(fd_);
		throw Exception("DP PTU: Cannot set parameters");
		;
	}

	// get initial values
	send(DPPTU_RESTORE);
	send(DPPTU_ECHO_DISABLE);
	send(DPPTU_ASCII_TERSE);

	send(DPPTU_RESET);

	pan_resolution_  = query_int(DPPTU_PAN_RESOLUTION);
	tilt_resolution_ = query_int(DPPTU_TILT_RESOLUTION);

	pan_upper_limit_  = query_int(DPPTU_PAN_MAX);
	pan_lower_limit_  = query_int(DPPTU_PAN_MIN);
	tilt_upper_limit_ = query_int(DPPTU_TILT_MAX);
	tilt_lower_limit_ = query_int(DPPTU_TILT_MIN);

	opened_ = true;
}

void
DirectedPerceptionPTU::close()
{
	if (opened_) {
		::close(fd_);
		opened_ = false;
	}
}

/** Stop currently running motion. */
void
DirectedPerceptionPTU::stop_motion()
{
	send(DPPTU_HALT_ALL);
}

/** Set pan in motor ticks.
 * @param pan pan position in ticks
 */
void
DirectedPerceptionPTU::set_pan(int pan)
{
	send(DPPTU_PAN_ABSPOS, pan);
}

/** Set tilt in motor ticks.
 * @param tilt tilt position in ticks
 */
void
DirectedPerceptionPTU::set_tilt(int tilt)
{
	send(DPPTU_TILT_ABSPOS, tilt);
}

/** Set pan and tilt in motor ticks.
 * @param pan pan position in ticks
 * @param tilt tilt position in ticks
 */
void
DirectedPerceptionPTU::set_pan_tilt(int pan, int tilt)
{
	if (pan > pan_upper_limit_)
		pan = pan_upper_limit_;
	if (pan < pan_lower_limit_)
		pan = pan_lower_limit_;
	if (tilt > tilt_upper_limit_)
		tilt = tilt_upper_limit_;
	if (tilt < tilt_lower_limit_)
		tilt = tilt_lower_limit_;

	send(DPPTU_PAN_ABSPOS, pan);
	send(DPPTU_TILT_ABSPOS, tilt);
}

/** Set pan and tilt in radians.
 * @param pan pan position rad
 * @param tilt tilt position rad
 */
void
DirectedPerceptionPTU::set_pan_tilt_rad(float pan, float tilt)
{
	set_pan_tilt(pan_rad2ticks(pan), tilt_rad2ticks(tilt));
}

/** Get current position in motor ticks.
 * @param pan upon return contains current pan position in motor ticks
 * @param tilt upon return contains current tilt position in motor ticks
 */
void
DirectedPerceptionPTU::get_pan_tilt(int &pan, int &tilt)
{
	pan  = query_int(DPPTU_PAN_ABSPOS);
	tilt = query_int(DPPTU_TILT_ABSPOS);
}

/** Get pan/tilt in radians.
 * @param pan upon return contains current pan position in radians
 * @param tilt upon return contains current tilt position in radians
 */
void
DirectedPerceptionPTU::get_pan_tilt_rad(float &pan, float &tilt)
{
	int tpan = 0, ttilt = 0;

	tpan  = query_int(DPPTU_PAN_ABSPOS);
	ttilt = query_int(DPPTU_TILT_ABSPOS);

	pan  = pan_ticks2rad(tpan);
	tilt = tilt_ticks2rad(ttilt);
}

/** Get current pan in motor ticks.
 * @return current pan in motor ticks
 */
int
DirectedPerceptionPTU::get_pan()
{
	return query_int(DPPTU_PAN_ABSPOS);
}

/** Get current tilt in motor ticks.
 * @return current tilt in motor ticks
 */
int
DirectedPerceptionPTU::get_tilt()
{
	return query_int(DPPTU_TILT_ABSPOS);
}

/** Get maximum pan in motor ticks.
 * @return maximum pan in motor ticks
 */
int
DirectedPerceptionPTU::max_pan()
{
	return pan_upper_limit_;
}

/** Get minimum pan in motor ticks.
 * @return minimum pan in motor ticks
 */
int
DirectedPerceptionPTU::min_pan()
{
	return pan_lower_limit_;
}

/** Get maximum tilt in motor ticks.
 * @return maximum tilt in motor ticks
 */
int
DirectedPerceptionPTU::max_tilt()
{
	return tilt_upper_limit_;
}

/** Get minimum tilt in motor ticks.
 * @return minimum tilt in motor ticks
 */
int
DirectedPerceptionPTU::min_tilt()
{
	return tilt_lower_limit_;
}

/** Get position limits in radians.
 * @param pan_min upon return contains minimum pan in radians
 * @param pan_max upon return contains maximum pan in radians
 * @param tilt_min upon return contains minimum tilt in radians
 * @param tilt_max upon return contains maximum tilt in radians
 */
void
DirectedPerceptionPTU::get_limits(float &pan_min, float &pan_max, float &tilt_min, float &tilt_max)
{
	pan_min  = pan_ticks2rad(pan_lower_limit_);
	pan_max  = pan_ticks2rad(tilt_upper_limit_);
	tilt_min = tilt_ticks2rad(tilt_lower_limit_);
	tilt_max = tilt_ticks2rad(tilt_upper_limit_);
}

/** Reset the PTU. */
void
DirectedPerceptionPTU::reset()
{
	send(DPPTU_RESET);
}

void
DirectedPerceptionPTU::send(const char *command, int value)
{
	snprintf(obuffer_, DPPTU_MAX_OBUFFER_SIZE, "%s%i ", command, value);
	write(obuffer_);
	if (!result_ok()) {
		printf("Writing with value '%s' to PTU failed\n", obuffer_);
	}
}

void
DirectedPerceptionPTU::send(const char *command)
{
	snprintf(obuffer_, DPPTU_MAX_OBUFFER_SIZE, "%s ", command);
	write(obuffer_);
	if (!result_ok()) {
		printf("Writing '%s' to PTU failed\n", obuffer_);
	}
}

void
DirectedPerceptionPTU::write(const char *buffer)
{
	printf("Writing '%s'\n", obuffer_);

	tcflush(fd_, TCIOFLUSH);
	unsigned int buffer_size = strlen(buffer);
	int          written     = ::write(fd_, buffer, buffer_size);
	tcdrain(fd_);

	if (written < 0) {
		printf("Writing '%s' failed: %s\n", buffer, strerror(errno));
	} else if ((unsigned int)written != buffer_size) {
		printf("Writing '%s' failed, only wrote %i of %u bytes\n", buffer, written, buffer_size);
	}
}

bool
DirectedPerceptionPTU::read(char *buffer, unsigned int buffer_size)
{
	// wait for message
	timeval      start, now;
	unsigned int diff_msec = 0;
	gettimeofday(&start, NULL);

	int num_bytes = 0;
	ioctl(fd_, FIONREAD, &num_bytes);
	while (((timeout_ms_ == 0) || (diff_msec < timeout_ms_)) && (num_bytes == 0)) {
		ioctl(fd_, FIONREAD, &num_bytes);

		gettimeofday(&now, NULL);
		diff_msec = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
		usleep(timeout_ms_ * 100);
	}
	if (num_bytes == 0) {
		return false;
	}
	ssize_t bytes_read = ::read(fd_, buffer, buffer_size);
	if (bytes_read < 0) {
		return false;
	} else {
		return (bytes_read > 0);
	}
}

bool
DirectedPerceptionPTU::result_ok()
{
	if (read(ibuffer_, 1)) {
		if (ibuffer_[0] == '*') {
			return true;
		}
	}

	return false;
}

bool
DirectedPerceptionPTU::data_available()
{
	int num_bytes = 0;
	ioctl(fd_, FIONREAD, &num_bytes);
	return (num_bytes > 0);
}

int
DirectedPerceptionPTU::query_int(const char *query_command)
{
	send(query_command);
	bool ok = read(ibuffer_, DPPTU_MAX_IBUFFER_SIZE);
	if (!ok) {
		throw Exception("DP PTU: failed to query integer");
	}
	int intrv = 0;
	if (sscanf(ibuffer_, "* %i", &intrv) <= 0) {
		throw Exception(errno, "DP PTU: failed to query int");
	}
	return intrv;
}

int
DirectedPerceptionPTU::pan_rad2ticks(float r)
{
	if (pan_resolution_ == 0)
		return 0;
	return (int)rint(rad2deg(r) * 3600 / pan_resolution_);
}

int
DirectedPerceptionPTU::tilt_rad2ticks(float r)
{
	if (tilt_resolution_ == 0)
		return 0;
	return (int)rint(rad2deg(r) * 3600 / tilt_resolution_);
}

float
DirectedPerceptionPTU::pan_ticks2rad(int ticks)
{
	if (pan_resolution_ == 0)
		return 0;
	return deg2rad(ticks * pan_resolution_ / 3600);
}

float
DirectedPerceptionPTU::tilt_ticks2rad(int ticks)
{
	if (tilt_resolution_ == 0)
		return 0;
	return deg2rad(ticks * tilt_resolution_ / 3600);
}
