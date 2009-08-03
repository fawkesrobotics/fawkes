
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
#include <utils/math/angle.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

using namespace std;
using namespace fawkes;

/** @class DirectedPerceptionPTU "dp_ptu.h"
 * DirectedPerception PTU implementation.
 * Control object to use the DirectedPerception PTU Pan/Tilt unit mounted
 * on carl.
 *
 * @author Tim Niemueller
 */

const char * DirectedPerceptionPTU::DPPTU_PAN_ABSPOS             = "PP";
const char * DirectedPerceptionPTU::DPPTU_TILT_ABSPOS            = "TP";
const char * DirectedPerceptionPTU::DPPTU_PAN_RELPOS             = "PO";
const char * DirectedPerceptionPTU::DPPTU_TILT_RELPOS            = "TO";
const char * DirectedPerceptionPTU::DPPTU_PAN_RESOLUTION         = "PR";
const char * DirectedPerceptionPTU::DPPTU_TILT_RESOLUTION        = "TR";
const char * DirectedPerceptionPTU::DPPTU_PAN_MIN                = "PN";
const char * DirectedPerceptionPTU::DPPTU_PAN_MAX                = "PX";
const char * DirectedPerceptionPTU::DPPTU_TILT_MIN               = "TN";
const char * DirectedPerceptionPTU::DPPTU_TILT_MAX               = "TX";
const char * DirectedPerceptionPTU::DPPTU_LIMITENFORCE_QUERY     = "L";
const char * DirectedPerceptionPTU::DPPTU_LIMITENFORCE_ENABLE    = "LE";
const char * DirectedPerceptionPTU::DPPTU_LIMITENFORCE_DISABLE   = "LD";
const char * DirectedPerceptionPTU::DPPTU_IMMEDIATE_EXECUTION    = "I";
const char * DirectedPerceptionPTU::DPPTU_SLAVED_EXECUTION       = "S";
const char * DirectedPerceptionPTU::DPPTU_AWAIT_COMPLETION       = "A";
const char * DirectedPerceptionPTU::DPPTU_HALT_ALL               = "H";
const char * DirectedPerceptionPTU::DPPTU_HALT_PAN               = "HP";
const char * DirectedPerceptionPTU::DPPTU_HALT_TILT              = "HT";
const char * DirectedPerceptionPTU::DPPTU_PAN_SPEED              = "PS";
const char * DirectedPerceptionPTU::DPPTU_TILT_SPEED             = "TS";
const char * DirectedPerceptionPTU::DPPTU_PAN_ACCEL              = "PA";
const char * DirectedPerceptionPTU::DPPTU_TILT_ACCEL             = "TA";
const char * DirectedPerceptionPTU::DPPTU_PAN_BASESPEED          = "PB";
const char * DirectedPerceptionPTU::DPPTU_TILT_BASESPEED         = "TB";
const char * DirectedPerceptionPTU::DPPTU_PAN_UPPER_SPEED_LIMIT  = "PU";
const char * DirectedPerceptionPTU::DPPTU_PAN_LOWER_SPEED_LIMIT  = "PL";
const char * DirectedPerceptionPTU::DPPTU_TILT_UPPER_SPEED_LIMIT = "TU";
const char * DirectedPerceptionPTU::DPPTU_TILT_LOWER_SPEED_LIMIT = "TL";
const char * DirectedPerceptionPTU::DPPTU_RESET                  = "R";
const char * DirectedPerceptionPTU::DPPTU_STORE                  = "DS";
const char * DirectedPerceptionPTU::DPPTU_RESTORE                = "DR";
const char * DirectedPerceptionPTU::DPPTU_FACTORY_RESET          = "DF";
const char * DirectedPerceptionPTU::DPPTU_ECHO_QUERY             = "E";
const char * DirectedPerceptionPTU::DPPTU_ECHO_ENABLE            = "EE";
const char * DirectedPerceptionPTU::DPPTU_ECHO_DISABLE           = "ED";
const char * DirectedPerceptionPTU::DPPTU_ASCII_VERBOSE          = "FV";
const char * DirectedPerceptionPTU::DPPTU_ASCII_TERSE            = "FT";
const char * DirectedPerceptionPTU::DPPTU_ASCII_QUERY            = "F";
const char * DirectedPerceptionPTU::DPPTU_VERSION                = "V";


/** Constructor.
 * @param device_file serial device file (e.g. /dev/ttyS0)
 * @param timeout_ms timeout for read operations in miliseconds
 */
DirectedPerceptionPTU::DirectedPerceptionPTU(const char *device_file,
					     unsigned int timeout_ms)
{
  __device_file = strdup(device_file);
  __opened      = false;
  __timeout_ms  = timeout_ms;

  open();
}



/** Destructor. */
DirectedPerceptionPTU::~DirectedPerceptionPTU()
{
  close();
  free(__device_file);
}


void
DirectedPerceptionPTU::open()
{
  if (__opened) return;

  __fd = ::open(__device_file, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if ( ! __fd || ! isatty(__fd)) {
    throw Exception("Cannot open device or device is not a TTY");
  }

  struct termios param;

  if (tcgetattr(__fd, &param) != 0) {
    ::close(__fd);
    throw Exception("DP PTU: Cannot get parameters");;
  }

  if ( cfsetspeed( &param, B9600 ) == -1 ) {
    ::close( __fd );
    throw Exception("DP PTU: Cannot set speed");;
  }

  cfsetospeed(&param, B9600);
  cfsetispeed(&param, B9600);

  // set serial line options
  param.c_cflag |= ( CLOCAL | CREAD );  // set to local and enable the receiver
  param.c_cflag &= ~CSIZE;              // mask character size bits
  param.c_cflag |= CS8;                 // select 8 data bits
  param.c_cflag &= ~PARENB;             // no parity
  param.c_cflag &= ~CSTOPB;             // 1 stop bit

  // set input options
  param.c_iflag &= ~( INPCK | ISTRIP ); // no input parity checking
  param.c_iflag &= ~( IXON | IXOFF | IXANY ); // no software flow control

  param.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );

  param.c_cc[ VTIME ] = 1;  // wait for a tenth of a second for data
  param.c_cc[ VMIN ] = 0;

  if (tcsetattr(__fd, TCSANOW, &param) != 0) {
    ::close(__fd);
    throw Exception("DP PTU: Cannot set parameters");;
  }

  // get initial values
  send(DPPTU_RESTORE);
  send(DPPTU_ECHO_DISABLE);
  send(DPPTU_ASCII_TERSE);

  send(DPPTU_RESET);

  __pan_resolution   = query_int(DPPTU_PAN_RESOLUTION);
  __tilt_resolution  = query_int(DPPTU_TILT_RESOLUTION);

  __pan_upper_limit  = query_int(DPPTU_PAN_MAX);
  __pan_lower_limit  = query_int(DPPTU_PAN_MIN);
  __tilt_upper_limit = query_int(DPPTU_TILT_MAX);
  __tilt_lower_limit = query_int(DPPTU_TILT_MIN);

  __opened = true;
}


void
DirectedPerceptionPTU::close()
{
  if (__opened) {
    ::close(__fd);
    __opened = false;
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
  if ( pan  > __pan_upper_limit  )  pan  = __pan_upper_limit;
  if ( pan  < __pan_lower_limit  )  pan  = __pan_lower_limit;
  if ( tilt > __tilt_upper_limit )  tilt = __tilt_upper_limit;
  if ( tilt < __tilt_lower_limit )  tilt = __tilt_lower_limit;

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
  return __pan_upper_limit;
}


/** Get minimum pan in motor ticks.
 * @return minimum pan in motor ticks
 */
int
DirectedPerceptionPTU::min_pan()
{
  return __pan_lower_limit;

}


/** Get maximum tilt in motor ticks.
 * @return maximum tilt in motor ticks
 */
int
DirectedPerceptionPTU::max_tilt()
{
  return __tilt_upper_limit;
}


/** Get minimum tilt in motor ticks.
 * @return minimum tilt in motor ticks
 */
int
DirectedPerceptionPTU::min_tilt()
{
  return __tilt_lower_limit;
}


/** Get position limits in radians.
 * @param pan_min upon return contains minimum pan in radians
 * @param pan_max upon return contains maximum pan in radians
 * @param tilt_min upon return contains minimum tilt in radians
 * @param tilt_max upon return contains maximum tilt in radians
 */
void
DirectedPerceptionPTU::get_limits(float &pan_min, float &pan_max,
				  float &tilt_min, float &tilt_max)
{
  pan_min  = pan_ticks2rad(__pan_lower_limit);
  pan_max  = pan_ticks2rad(__tilt_upper_limit);
  tilt_min = tilt_ticks2rad(__tilt_lower_limit);
  tilt_max = tilt_ticks2rad(__tilt_upper_limit);
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
  snprintf(__obuffer, DPPTU_MAX_OBUFFER_SIZE, "%s%i ", command, value);
  write(__obuffer);
  if ( ! result_ok() ) {
    printf("Writing with value '%s' to PTU failed\n", __obuffer);
  }
}


void
DirectedPerceptionPTU::send(const char *command)
{
  snprintf(__obuffer, DPPTU_MAX_OBUFFER_SIZE, "%s ", command);
  write(__obuffer);
  if ( ! result_ok() ) {
    printf("Writing '%s' to PTU failed\n", __obuffer);
  }
}


void
DirectedPerceptionPTU::write(const char *buffer)
{
  printf("Writing '%s'\n", __obuffer);

  tcflush( __fd, TCIOFLUSH );
  unsigned int buffer_size = strlen(buffer);
  int written = ::write(__fd, buffer, buffer_size);
  tcdrain(__fd);

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
  timeval start, now;
  unsigned int diff_msec = 0;
  gettimeofday(&start, NULL);

  int num_bytes = 0;
  ioctl(__fd, FIONREAD, &num_bytes);
  while ( ((__timeout_ms == 0) || (diff_msec < __timeout_ms)) && (num_bytes == 0)) {
    ioctl(__fd, FIONREAD, &num_bytes);

    gettimeofday(&now, NULL);
    diff_msec  = (now.tv_sec  - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
    usleep(__timeout_ms * 100);
  }
  if (num_bytes == 0) {
    return false;
  }
  int bytes_read = ::read(__fd, buffer, buffer_size);
  if ( bytes_read < 0 ) {
    return false;
  } else {
    if ((unsigned int)bytes_read == buffer_size) {
      return true;
    } else {
      return false;
    }
  }
}


bool
DirectedPerceptionPTU::result_ok()
{
  if ( read(__ibuffer, 1) ) {
    if ( __ibuffer[0] == '*' ) {
      return true;
    }
  }

  return false;
}


bool
DirectedPerceptionPTU::data_available()
{
  int num_bytes = 0;
  ioctl(__fd, FIONREAD, &num_bytes);
  return (num_bytes > 0);
}


int
DirectedPerceptionPTU::query_int(const char *query_command)
{
  send(query_command);
  ssize_t read_bytes = read(__ibuffer, DPPTU_MAX_OBUFFER_SIZE);
  if ( read_bytes == -1 ) {
    throw FileReadException(__device_file, errno, "Querying integer from PTU failed");
  } else if (read_bytes == 0) {
    return 0;
  }
  int rv = 0;
  sscanf(__ibuffer, "* %i", &rv);
  return rv;
}


int
DirectedPerceptionPTU::pan_rad2ticks(float r)
{
  if ( __pan_resolution == 0 )  return 0;
  return (int)rint(rad2deg(r) * 3600 / __pan_resolution);
}


int
DirectedPerceptionPTU::tilt_rad2ticks(float r)
{
  if ( __tilt_resolution == 0 )  return 0;
  return (int)rint(rad2deg(r) * 3600 / __tilt_resolution);
}


float
DirectedPerceptionPTU::pan_ticks2rad(int ticks)
{
  if ( __pan_resolution == 0 )  return 0;
  return deg2rad(ticks * __pan_resolution / 3600);
}


float
DirectedPerceptionPTU::tilt_ticks2rad(int ticks)
{
  if ( __tilt_resolution == 0 )  return 0;
  return deg2rad(ticks * __tilt_resolution / 3600);
}
