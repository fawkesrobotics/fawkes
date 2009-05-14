
/***************************************************************************
 *  dp_ptu.cpp - Controller for Directed Perception, Inc. Pan-Tilt Unit on B21
 *
 *  Created: Wed Nov 29 23:05:49 2006
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <core/exception.h>

#include <cams/control/dp_ptu.h>
#include <fvutils/system/camargp.h>

#include <utils/math/angle.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;
using namespace fawkes;

/** @class DPPTUControl <cams/control/dp_ptu.h>
 * DP PTU Control implementation.
 * Control object to use the DP PTU Pan/Tilt unit mounted
 * on carl.
 *
 * @author Tim Niemueller
 */

const char * DPPTUControl::DPPTU_PAN_ABSPOS             = "PP";
const char * DPPTUControl::DPPTU_TILT_ABSPOS            = "TP";
const char * DPPTUControl::DPPTU_PAN_RELPOS             = "PO";
const char * DPPTUControl::DPPTU_TILT_RELPOS            = "TO";
const char * DPPTUControl::DPPTU_PAN_RESOLUTION         = "PR";
const char * DPPTUControl::DPPTU_TILT_RESOLUTION        = "TR";
const char * DPPTUControl::DPPTU_PAN_MIN                = "PN";
const char * DPPTUControl::DPPTU_PAN_MAX                = "PX";
const char * DPPTUControl::DPPTU_TILT_MIN               = "TN";
const char * DPPTUControl::DPPTU_TILT_MAX               = "TX";
const char * DPPTUControl::DPPTU_LIMITENFORCE_QUERY     = "L";
const char * DPPTUControl::DPPTU_LIMITENFORCE_ENABLE    = "LE";
const char * DPPTUControl::DPPTU_LIMITENFORCE_DISABLE   = "LD";
const char * DPPTUControl::DPPTU_IMMEDIATE_EXECUTION    = "I";
const char * DPPTUControl::DPPTU_SLAVED_EXECUTION       = "S";
const char * DPPTUControl::DPPTU_AWAIT_COMPLETION       = "A";
const char * DPPTUControl::DPPTU_HALT_ALL               = "H";
const char * DPPTUControl::DPPTU_HALT_PAN               = "HP";
const char * DPPTUControl::DPPTU_HALT_TILT              = "HT";
const char * DPPTUControl::DPPTU_PAN_SPEED              = "PS";
const char * DPPTUControl::DPPTU_TILT_SPEED             = "TS";
const char * DPPTUControl::DPPTU_PAN_ACCEL              = "PA";
const char * DPPTUControl::DPPTU_TILT_ACCEL             = "TA";
const char * DPPTUControl::DPPTU_PAN_BASESPEED          = "PB";
const char * DPPTUControl::DPPTU_TILT_BASESPEED         = "TB";
const char * DPPTUControl::DPPTU_PAN_UPPER_SPEED_LIMIT  = "PU";
const char * DPPTUControl::DPPTU_PAN_LOWER_SPEED_LIMIT  = "PL";
const char * DPPTUControl::DPPTU_TILT_UPPER_SPEED_LIMIT = "TU";
const char * DPPTUControl::DPPTU_TILT_LOWER_SPEED_LIMIT = "TL";
const char * DPPTUControl::DPPTU_RESET                  = "R";
const char * DPPTUControl::DPPTU_STORE                  = "DS";
const char * DPPTUControl::DPPTU_RESTORE                = "DR";
const char * DPPTUControl::DPPTU_FACTORY_RESET          = "DF";
const char * DPPTUControl::DPPTU_ECHO_QUERY             = "E";
const char * DPPTUControl::DPPTU_ECHO_ENABLE            = "EE";
const char * DPPTUControl::DPPTU_ECHO_DISABLE           = "ED";
const char * DPPTUControl::DPPTU_ASCII_VERBOSE          = "FV";
const char * DPPTUControl::DPPTU_ASCII_TERSE            = "FT";
const char * DPPTUControl::DPPTU_ASCII_QUERY            = "F";
const char * DPPTUControl::DPPTU_VERSION                = "V";


/** Constructor.
 * @param tty_port port device file (e.g. /dev/ttyS0)
 */
DPPTUControl::DPPTUControl(const char *tty_port)
{
  this->tty_port = strdup(tty_port);
  opened = false;
  max_wait_ms = 10;

  open();
}


/** Constructor.
 * Uses camera argument parser to gather arguments. The ID that the camera argument
 * parser returns is used as the serial port (like /dev/ttyS0).
 * @param cap camera argument parser
 */
DPPTUControl::DPPTUControl(const CameraArgumentParser *cap)
{
  tty_port = strdup(cap->cam_id().c_str());
  opened = false;
  max_wait_ms = 10;

  open();
}


/** Destructor. */
DPPTUControl::~DPPTUControl()
{
  free(tty_port);
}


void
DPPTUControl::open()
{
  if (opened) return;

  dev = ::open(tty_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if ( ! dev || ! isatty(dev)) {
    throw Exception("Cannot open device or device is not a TTY");
  }

  struct termios param;

  if (tcgetattr(dev, &param) != 0) {
    ::close(dev);
    throw Exception("DP PTU: Cannot get parameters");;
  }

  if ( cfsetspeed( &param, B9600 ) == -1 ) {
    ::close( dev );
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

  if (tcsetattr(dev, TCSANOW, &param) != 0) {
    ::close(dev);
    throw Exception("DP PTU: Cannot set parameters");;
  }

  // get initial values
  send(DPPTU_RESTORE);
  send(DPPTU_ECHO_DISABLE);
  send(DPPTU_ASCII_TERSE);

  send(DPPTU_RESET);

  pan_resolution   = query_int(DPPTU_PAN_RESOLUTION);
  tilt_resolution  = query_int(DPPTU_TILT_RESOLUTION);

  pan_upper_limit  = query_int(DPPTU_PAN_MAX);
  pan_lower_limit  = query_int(DPPTU_PAN_MIN);
  tilt_upper_limit = query_int(DPPTU_TILT_MAX);
  tilt_lower_limit = query_int(DPPTU_TILT_MIN);

  opened = true;
}

void
DPPTUControl::process_pantilt()
{
}

bool
DPPTUControl::supports_pan()
{
  return true;
}


bool
DPPTUControl::supports_tilt()
{
  return true;
}


void
DPPTUControl::set_pan(int pan)
{
  send(DPPTU_PAN_ABSPOS, pan);
}


void
DPPTUControl::set_tilt(int tilt)
{
  send(DPPTU_TILT_ABSPOS, tilt);
}


void
DPPTUControl::set_pan_tilt(int pan, int tilt)
{
  if ( pan  > pan_upper_limit  )  pan  = pan_upper_limit;
  if ( pan  < pan_lower_limit  )  pan  = pan_lower_limit;
  if ( tilt > tilt_upper_limit )  tilt = tilt_upper_limit;
  if ( tilt < tilt_lower_limit )  tilt = tilt_lower_limit;

  send(DPPTU_PAN_ABSPOS, pan);
  send(DPPTU_TILT_ABSPOS, tilt);
}


void
DPPTUControl::set_pan_tilt_rad(float pan, float tilt)
{
  set_pan_tilt(pan_rad2ticks(pan), tilt_rad2ticks(tilt));
}


void
DPPTUControl::start_get_pan_tilt()
{
}


void
DPPTUControl::pan_tilt(int &pan, int &tilt)
{
  pan  = query_int(DPPTU_PAN_ABSPOS);
  tilt = query_int(DPPTU_TILT_ABSPOS);
}


void
DPPTUControl::pan_tilt_rad(float &pan, float &tilt)
{
  int tpan = 0, ttilt = 0;

  tpan  = query_int(DPPTU_PAN_ABSPOS);
  ttilt = query_int(DPPTU_TILT_ABSPOS);

  pan  = pan_ticks2rad(tpan);
  tilt = tilt_ticks2rad(ttilt);
}


int
DPPTUControl::pan()
{
  return query_int(DPPTU_PAN_ABSPOS);
}


int
DPPTUControl::tilt()
{
  return query_int(DPPTU_TILT_ABSPOS);
}


int
DPPTUControl::max_pan()
{
  return pan_upper_limit;
}


int
DPPTUControl::min_pan()
{
  return pan_lower_limit;

}


int
DPPTUControl::max_tilt()
{
  return tilt_upper_limit;
}


int
DPPTUControl::min_tilt()
{
  return tilt_lower_limit;
}


void
DPPTUControl::reset_pan_tilt()
{
  send(DPPTU_RESET);
}


void
DPPTUControl::set_pan_tilt_limit(int pan_left, int pan_right,
				 int tilt_up, int tilt_down)
{
}


void
DPPTUControl::reset_pan_tilt_limit()
{
}


void
DPPTUControl::send(const char *command, int value)
{
  snprintf(out_buffer, DPPTU_MAX_OBUFFER_SIZE, "%s%i ", command, value);
  write(out_buffer);
  if ( ! result_ok() ) {
    printf("Writing with value '%s' to PTU failed\n", out_buffer);
  }
}


void
DPPTUControl::send(const char *command)
{
  snprintf(out_buffer, DPPTU_MAX_OBUFFER_SIZE, "%s ", command);
  write(out_buffer);
  if ( ! result_ok() ) {
    printf("Writing '%s' to PTU failed\n", out_buffer);
  }
}


void
DPPTUControl::write(const char *buffer)
{
  printf("Writing '%s'\n", out_buffer);

  tcflush( dev, TCIOFLUSH );
  unsigned int buffer_size = strlen(buffer);
  int written = ::write(dev, buffer, buffer_size);
  tcdrain(dev);

  if (written < 0) {
    printf("Writing '%s' failed: %s\n", buffer, strerror(errno));
  } else if ((unsigned int)written != buffer_size) {
    printf("Writing '%s' failed, only wrote %i of %u bytes\n", buffer, written, buffer_size);
  }
}


bool
DPPTUControl::read(char *buffer, unsigned int buffer_size)
{
  // wait for message
  timeval start, now;
  unsigned int diff_msec = 0;
  gettimeofday(&start, NULL);

  int num_bytes = 0;
  ioctl(dev, FIONREAD, &num_bytes);
  while ( ((max_wait_ms == 0) || (diff_msec < max_wait_ms)) && (num_bytes == 0)) {
    ioctl(dev, FIONREAD, &num_bytes);

    gettimeofday(&now, NULL);
    diff_msec  = (now.tv_sec  - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
    usleep(max_wait_ms * 100);
  }
  if (num_bytes == 0) {
    return false;
  }
  int bytes_read = ::read(dev, buffer, buffer_size);
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
DPPTUControl::result_ok()
{
  if ( read(in_buffer, 1) ) {
    if ( in_buffer[0] == '*' ) {
      return true;
    }
  }

  return false;
}


bool
DPPTUControl::data_available()
{
  int num_bytes = 0;
  ioctl(dev, FIONREAD, &num_bytes);
  return (num_bytes > 0);
}


int
DPPTUControl::query_int(const char *query_command)
{
  send(query_command);
  if ( ! read(in_buffer, DPPTU_MAX_OBUFFER_SIZE) ) {
    return 0;
  }
  int rv = 0;
  sscanf(in_buffer, "* %i", &rv);
  return rv;
}


int
DPPTUControl::pan_rad2ticks(float r)
{
  if ( pan_resolution == 0 )  return 0;
  return (int)rint(rad2deg(r) * 3600 / pan_resolution);
}


int
DPPTUControl::tilt_rad2ticks(float r)
{
  if ( tilt_resolution == 0 )  return 0;
  return (int)rint(rad2deg(r) * 3600 / tilt_resolution);
}


float
DPPTUControl::pan_ticks2rad(int ticks)
{
  if ( pan_resolution == 0 )  return 0;
  return deg2rad(ticks * pan_resolution / 3600);
}


float
DPPTUControl::tilt_ticks2rad(int ticks)
{
  if ( tilt_resolution == 0 )  return 0;
  return deg2rad(ticks * tilt_resolution / 3600);
}
