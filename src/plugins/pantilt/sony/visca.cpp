
/***************************************************************************
 *  visca.cpp - Controller for Visca cams
 *
 *  Created: Wed Jun 08 12:08:17 2005 (FireVision)
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

#include "visca.h"

#include <core/exceptions/system.h>

#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cstdlib>

/** @class ViscaException "visca.h"
 * Visca exception.
 */

/** Constructor.
 * @param msg message of exception.
 */
ViscaException::ViscaException(const char *msg)
  : Exception(msg)
{}


/** Constructor with errno.
 * @param msg message prefix
 * @param _errno errno for additional error information.
 */
ViscaException::ViscaException(const char *msg, const int _errno)
  : Exception(_errno, msg)
{}

/** @class ViscaInquiryRunningException "visca.h"
 * Visca inquire running exception.
 */

/** Constructor. */
ViscaInquiryRunningException::ViscaInquiryRunningException()
  : ViscaException("Inquiry already running")
{}


/** Automatic white balance. */
const unsigned int Visca::VISCA_WHITEBLANCE_AUTO      = VISCA_WB_AUTO;
/** Indoor white balance preset. */
const unsigned int Visca::VISCA_WHITEBALANCE_INDOOR   = VISCA_WB_INDOOR;
/** Outdoor white balance preset. */
const unsigned int Visca::VISCA_WHITEBALANCE_OUTDOOR  = VISCA_WB_OUTDOOR;
/** One push white balance preset. */
const unsigned int Visca::VISCA_WHITEBALANCE_ONE_PUSH = VISCA_WB_ONE_PUSH;
/** ATW white balance preset. */
const unsigned int Visca::VISCA_WHITEBALANCE_ATW      = VISCA_WB_ATW;
/** Manual white balance. */
const unsigned int Visca::VISCA_WHITEBALANCE_MANUAL   = VISCA_WB_MANUAL;

/** Non-blocking pan/tilt item. */
const unsigned int Visca::NONBLOCKING_PANTILT   = 0;
/** Non-blocking zoom item. */
const unsigned int Visca::NONBLOCKING_ZOOM      = 1;
/** Number of non-blocking items. */
const unsigned int Visca::NONBLOCKING_NUM       = 2;

/** Number of non-blocking items. */
const unsigned int Visca::MAX_PAN_SPEED         = 0x18;

/** Number of non-blocking items. */
const unsigned int Visca::MAX_TILT_SPEED        = 0x14;


/** @class Visca "visca.h"
 * Visca control protocol implementation over a serial line.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param device_file serial device file (e.g. /dev/ttyUSB0)
 * @param def_timeout_ms default timeout for read operations applied if no explicit
 * timeout is given.
 * @param blocking if true, setting the pan/tilt values will only cause sending the
 * request, you need to call process() when there is time to process and handle
 * incoming messages.
 */
Visca::Visca(const char *device_file, unsigned int def_timeout_ms, bool blocking)
{
  __inquire            = VISCA_RUNINQ_NONE;
  __device_file        = strdup(device_file);
  __blocking           = blocking;
  __opened             = false;
  __default_timeout_ms = def_timeout_ms;
  __pan_speed          = MAX_PAN_SPEED;
  __tilt_speed         = MAX_TILT_SPEED;

  for (unsigned int i = 0; i < NONBLOCKING_NUM; ++i) {
    __nonblocking_sockets[i] = 0;
    __nonblocking_running[i] = false;
  }

  open();

  set_address();
  clear();
}


/** Destructor. */
Visca::~Visca()
{
  close();
  free(__device_file);
}


/** Open serial port. */
void
Visca::open() {

  struct termios param;

  __fd = ::open(__device_file, O_RDWR);
  if (! __fd) {
    throw ViscaException("Cannot open device", errno);
  }

  if (tcgetattr(__fd, &param) == -1) {
    ViscaException ve("Getting the port parameters failed", errno);
    ::close(__fd);
    throw ve;
  }

  cfsetospeed(&param, B9600);
  cfsetispeed(&param, B9600);

  param.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  param.c_cflag |= CREAD;
  param.c_cflag |= CLOCAL;
  //param.c_cflag |= CRTSCTS;
  
  param.c_cc[VMIN] = 1;
  param.c_cc[VTIME] = 0;
    
  param.c_iflag |= IGNBRK;
  param.c_iflag &= ~PARMRK;
  param.c_iflag &= ~ISTRIP;
  param.c_iflag &= ~INLCR;
  param.c_iflag &= ~IGNCR;
  param.c_iflag &= ~ICRNL;
  param.c_iflag &= ~IXON;
  param.c_iflag &= ~IXOFF;
    
  param.c_lflag &= ~ECHO;
    
  // hand shake
  param.c_lflag |= IEXTEN;
  param.c_oflag &= ~OPOST;  //enable raw output
    
  //tcflow (__fd, TCOON);
  //tcflow (__fd, TCION);

  // number of data bits: 8
  param.c_cflag &= ~CS5 & ~CS6 & ~CS7 & ~CS8;

  param.c_cflag |= CS8;
    
  // parity: none
  param.c_cflag &=~(PARENB & PARODD);
    
  // stop bits: 1
  param.c_cflag &= ~CSTOPB;

  if (tcsetattr(__fd, TCSANOW, &param) != 0) {
    ViscaException ve("Setting the port parameters failed", errno);
    ::close(__fd);
    throw ve;
  }

  __opened = true;
  // Choose first camera by default
  __sender    = VISCA_BUS_0;
  __recipient = VISCA_BUS_1;

#ifdef TIMETRACKER_VISCA
  __tt = new TimeTracker();
  __ttc_pantilt_get_send = __tt->addClass("getPanTilt: send");
  __ttc_pantilt_get_read = __tt->addClass("getPanTilt: read");
  __ttc_pantilt_get_handle = __tt->addClass("getPanTilt: handling responses");
  __ttc_pantilt_get_interpret = __tt->addClass("getPanTilt: interpreting");
#endif
}


/** Close port. */
void
Visca::close()
{
  if (__opened) {
    __opened = false;
    ::close(__fd);
  }
}


/** Set addresses of cameras. */
void
Visca::set_address()
{
  unsigned char recp_backup = __recipient;
  __recipient = VISCA_BUS_BROADCAST;
  __obuffer[1] = 0x30;
  __obuffer[2] = 0x01;
  __obuffer_length = 2;

  try {
    send();
    recv();
  } catch (ViscaException &e) {
    __recipient = recp_backup;
    throw;
  }

  __recipient = recp_backup;
}


/** Clear command buffers. */
void
Visca::clear()
{
  if (!__opened)  throw ViscaException("Serial port not open");

  __obuffer[1] = 0x01;
  __obuffer[2] = 0x00;
  __obuffer[3] = 0x01;
  __obuffer_length = 3;

  try {
    send();
    recv();
  } catch (ViscaException &e) {
    e.append("clear() failed");
    throw;
  }
}


/** Send outbound queue. */
void
Visca::send()
{
  if (!__opened)  throw ViscaException("Serial port not open");

  // Set first bit to 1
  __obuffer[0] =  0x80;
  __obuffer[0] |= (__sender << 4);
  __obuffer[0] |= __recipient;

  __obuffer[++__obuffer_length] = VISCA_TERMINATOR;
  ++__obuffer_length;

  int written = write(__fd, __obuffer, __obuffer_length);
  //printf("Visca sent: ");
  //for (int i = 0; i < __obuffer_length; ++i) {
  //  printf("%02X", __obuffer[i]);
  //}
  //printf("\n");
  if (written < __obuffer_length) {
    throw ViscaException("Not all bytes send");
  }
}


/** Check data availability.
 * @return true if data is available, false otherwise
 */
bool
Visca::data_available()
{
  int num_bytes = 0;
  ioctl(__fd, FIONREAD, &num_bytes);
  return (num_bytes > 0);
}


/** Receive data.
 * @param timeout_ms read timeout in miliseconds
 */
void
Visca::recv(unsigned int timeout_ms)
{
  if (timeout_ms == 0xFFFFFFFF) timeout_ms = __default_timeout_ms;
  try {
    recv_packet(timeout_ms);
  } catch (ViscaException &e) {
    e.append("Receiving failed, recv_packet() call failed");
    throw;
  }

  // Get type of message
  unsigned char type = __ibuffer[1] & 0xF0;
  while (type == VISCA_RESPONSE_ACK) {
    try {
      recv_packet(timeout_ms);
    } catch (ViscaException &e) {
      e.append("Receiving failed, recv_packet() call 2 failed");
      throw;
    }
    type = __ibuffer[1] & 0xF0;
  }

  switch (type) {
  case VISCA_RESPONSE_CLEAR:
  case VISCA_RESPONSE_ADDRESS:
  case VISCA_RESPONSE_COMPLETED:
  case VISCA_RESPONSE_ERROR:
    break;
  default:
    throw fawkes::Exception("Receiving failed, unexpected packet type %u received",
			    type);
  }
}


/** Receive ACK packet.
 * @param socket contains the socket that the ACK was received on upon return
 */
void
Visca::recv_ack(unsigned int *socket)
{
  try {
    recv_packet(__default_timeout_ms);
  } catch (ViscaException &e) {
    throw ViscaException("recv_ack(): recv_packet() failed");
  }

  // Get type of message
  unsigned char type = __ibuffer[1] & 0xF0;
  while (type != VISCA_RESPONSE_ACK) {

    try {
      handle_response();
      recv_packet(__default_timeout_ms);
    } catch (ViscaException &e) {
      e.append("Handling message of type %u failed", type);
      throw;
    }
    type = __ibuffer[1] & 0xF0;
  }

  // Got an ack now
  if (socket != NULL) {
    *socket = __ibuffer[1] & 0x0F;
  }

}


/** Send non-blocking.
 * Does a non-blocking send.
 * @param socket the socket that was used to send the request.
 */
void
Visca::send_nonblocking(unsigned int *socket)
{
  try {
    send();
    recv_ack(socket);
  } catch (ViscaException &e) {
    e.append("Non-blocking send failed!");
    throw;
  }
}


/** Finish a non-blocking operation.
 * @param socket socket that the non-blocking operation was sent to
 */
void
Visca::finish_nonblocking( unsigned int socket )
{
  for (unsigned int i = 0; i < NONBLOCKING_NUM; ++i) {
    if (__nonblocking_sockets[i] == socket) {
      __nonblocking_sockets[i] = 0;
      __nonblocking_running[i] = false;
      return;
    }
  }

  throw ViscaException("finish_nonblocking() failed: socket not found");
}


/** Check if a non-blocking operation has been finished.
 * @param item the non-blocking item to check
 * @return true if the non-blocking operation has been finished, false otherwise
 */
bool
Visca::is_nonblocking_finished(unsigned int item) const
{
  if (item >= NONBLOCKING_NUM) {
    throw ViscaException("Invalid item number");
  }
  return ! __nonblocking_running[item];
}


/** Send and wait for reply, blocking. */
void
Visca::send_with_reply()
{
  try {
    send();

    if (__obuffer[1] == VISCA_COMMAND) {
      // do not catch timeouts here, we expect them to be on time
      recv_ack();
      bool rcvd = false;

      while (! rcvd) {
	try {
	  recv();
	  rcvd = true;
	} catch (fawkes::TimeoutException &e) {} // ignored
      }
    } else {
      // timeout applies to inquiries
      recv();
    }
  } catch (ViscaException &e) {
    e.append("Sending with reply failed");
    throw;
  }
}


/** Receive a packet.
 * @param timeout_ms read timeout in miliseconds
 */
void
Visca::recv_packet(unsigned int timeout_ms)
{
  // wait for message
  timeval timeout = {0, (suseconds_t)timeout_ms * 1000};

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(__fd, &read_fds);

  int rv = 0;
  rv = select(__fd + 1, &read_fds, NULL, NULL, &timeout);

  if ( rv == -1 ) {
    throw fawkes::Exception(errno, "Select on FD failed");
  } else if ( rv == 0 ) {
    throw fawkes::TimeoutException("Timeout reached while waiting for incoming data");
  }

  // get octets one by one
  if (read(__fd, __ibuffer, 1) != 1) {
    throw fawkes::Exception(errno, "Visca reading packet byte failed (1)");
  }

  size_t pos = 0;
  while (__ibuffer[pos] != VISCA_TERMINATOR && (pos < sizeof(__ibuffer)-1)) {
    if (read(__fd, &__ibuffer[++pos], 1) != 1) {
      

      throw fawkes::Exception(errno, "Visca reading packet byte failed (2)");
    }
    usleep(0);
  }
  __ibuffer_length = pos + 1;
  //printf("Visca read: ");
  //for (int i = 0; i < __ibuffer_length; ++i) {
  //  printf("%02X", __ibuffer[i]);
  //}
  //printf("\n");
}



/** Handle incoming response.  */
void
Visca::handle_response()
{
  unsigned int type = __ibuffer[1] & 0xF0;
  unsigned int socket = __ibuffer[1] & 0x0F;

  if (socket == 0) {
    // This is an inquire response, do NOT handle!
    //throw ViscaException("handle_response(): Received an inquire response, can't handle");
    return;
  }

  if ( type == VISCA_RESPONSE_COMPLETED ) {
    // Command has been finished
    try {
      finish_nonblocking( __ibuffer[1] & 0x0F );
    } catch (ViscaException &e) {
      // Ignore, happens sometimes without effect
      // e.append("handle_response() failed, could not finish non-blocking");
      // throw;
    }
  } else if ( type == VISCA_RESPONSE_ERROR ) {
    finish_nonblocking( __ibuffer[1] & 0x0F );
    //throw ViscaException("handle_response(): got an error message from camera");
  } else {
    // ignore
    //ViscaException ve("Got unknown/unhandled response type");
    //ve.append("Received message of type %u", type);
    //throw ve;
  }

}


/** Cancel a running command.
 * @param socket socket that the command was send on
 */
void
Visca::cancel_command( unsigned int socket )
{
  unsigned char cancel_socket = socket & 0x0000000F;

  __obuffer[1] = VISCA_CANCEL | cancel_socket;
  __obuffer_length = 1;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("cancel_command() failed");
    throw;
  }

  if (  ((__ibuffer[1] & 0xF0) == VISCA_RESPONSE_ERROR) &&
	((__ibuffer[1] & 0x0F) == cancel_socket) &&
	((__ibuffer[2] == VISCA_ERROR_CANCELLED)) ) {
    return;
  } else {
    throw ViscaException("Command could not be cancelled");
  }
}


/** Process incoming data. */
void
Visca::process()
{

  __inquire = VISCA_RUNINQ_NONE;

  while (data_available()) {
    try {
      recv();
      handle_response();
    } catch (ViscaException &e) {
      // Ignore this error
      return;
    }
  }
}


/** Set power state.
 * @param powered true to power on, false to power off
 */
void
Visca::set_power(bool powered)
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_POWER;
  __obuffer[4] = powered ? VISCA_POWER_ON : VISCA_POWER_OFF;
  __obuffer_length = 4;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("set_power() failed");
    throw;
  }
}


/** Check if camera is powered
 * @return true if camera is powered, false otherwise
 */
bool
Visca::is_powered()
{
  __obuffer[1] = VISCA_INQUIRY;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_POWER;
  __obuffer_length = 3;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("Failed to get power data");
    throw;
  }

  // Extract information from __ibuffer
  if ( __ibuffer[1] == VISCA_RESPONSE_COMPLETED ) {
    return (__ibuffer[2] == VISCA_POWER_ON);
  } else {
    throw ViscaException("is_powered(): inquiry failed, response code not VISCA_RESPONSE_COMPLETED");
  }
}


/** Set pan tilt.
 * @param pan pan
 * @param tilt tilt
 */
void
Visca::set_pan_tilt(int pan, int tilt)
{
  
  // we do not to check for blocking, could not be called at
  // the same time if blocking...
  /*
  if ( __nonblocking_running[ NONBLOCKING_PANTILT] ) {
    cout << "Cancelling old setPanTilt" << endl;
    if (cancel_command( __nonblocking_sockets[ NONBLOCKING_PANTILT ] ) != VISCA_SUCCESS) {
      cout << "Visca: Could not cancel old non-blocking pan/tilt command. Not setting new pan/tilt." << endl;
      return VISCA_E_CANCEL;
    }
    __nonblocking_running[ NONBLOCKING_PANTILT ] = false;
  }
  */

  unsigned short int tilt_val = 0 + tilt;
  unsigned short int pan_val  = 0 + pan;

  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
  __obuffer[3] = VISCA_PT_ABSOLUTE_POSITION;
  __obuffer[4] = __pan_speed;
  __obuffer[5] = __tilt_speed;

  // pan
  __obuffer[6] = (pan_val & 0xf000) >> 12;
  __obuffer[7] = (pan_val & 0x0f00) >>  8;
  __obuffer[8] = (pan_val & 0x00f0) >>  4;
  __obuffer[9] = (pan_val & 0x000f);
  // tilt
  __obuffer[10] = (tilt_val & 0xf000) >> 12;
  __obuffer[11] = (tilt_val & 0x0f00) >> 8;
  __obuffer[12] = (tilt_val & 0x00f0) >> 4;
  __obuffer[13] = (tilt_val & 0x000f);

  __obuffer_length = 13;

  try {
    if (! __blocking) {
      __nonblocking_running[ NONBLOCKING_PANTILT ] = true;
      send_nonblocking( &(__nonblocking_sockets[ NONBLOCKING_PANTILT ]) );
    } else {
      send_with_reply();
    }
  } catch (ViscaException &e) {
    e.append("setPanTilt() failed");
    throw;
  }
}


/** Set pan/tilt speed.
 * @param pan_speed a value between 0 and MAX_PAN_SPEED
 * @param tilt_speed a value between 0 and MAX_TILT_SPEED
 * @exception Exception thrown if desired pan or tilt speed is too high
 */
void
Visca::set_pan_tilt_speed(unsigned char pan_speed, unsigned char tilt_speed)
{
  if (pan_speed > MAX_PAN_SPEED) {
    throw fawkes::Exception("Pan speed too hight, max: %u  des: %u", MAX_PAN_SPEED, pan_speed);
  }
  if (tilt_speed > MAX_TILT_SPEED) {
    throw fawkes::Exception("Tilt speed too hight, max: %u  des: %u", MAX_TILT_SPEED, tilt_speed);
  }

  __pan_speed  = pan_speed;
  __tilt_speed = tilt_speed;
}


/** Get pan/tilt speed.
 * @param pan_speed upon return contains pan speed index
 * @param tilt_speed upon return contains tilt speed index
 */
void
Visca::get_pan_tilt_speed(unsigned char &pan_speed, unsigned char &tilt_speed)
{
  pan_speed  = __pan_speed;
  tilt_speed = __tilt_speed;
}

/** Initiate a pan/tilt request, but do not wait for the reply. */
void
Visca::start_get_pan_tilt()
{

  if ( __inquire )  throw ViscaInquiryRunningException();

  __inquire = VISCA_RUNINQ_PANTILT;

  __obuffer[1] = VISCA_INQUIRY;
  __obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
  __obuffer[3] = VISCA_PT_POSITION_INQ;
  __obuffer_length = 3;

  try {
    send();
  } catch (ViscaException &e) {
    e.append("startGetPanTilt() failed");
    throw;
  }
}


/** Get pan and tilt values.
 * If you used startGetPanTilt() to initiate the query the result is
 * received and returned, otherwise a request is sent and the method blocks
 * until the answer has been received.
 * @param pan contains pan upon return
 * @param tilt contains tilt upon return
 */
void
Visca::get_pan_tilt(int &pan, int &tilt)
{

  if ( __inquire ) {
    if ( __inquire != VISCA_RUNINQ_PANTILT ) {
      throw ViscaException("Inquiry running, but it is not a pan/tilt inquiry");
    } else {
#ifdef TIMETRACKER_VISCA
      __tt->ping_start( __ttc_pantilt_get_read );
#endif
      try {
	recv();
      } catch (ViscaException &e) {
      } catch (fawkes::TimeoutException &e) {
	// Ignore
      }
#ifdef TIMETRACKER_VISCA
      __tt->ping_end( __ttc_pantilt_get_read );
#endif
    }
  } else {

    __obuffer[1] = VISCA_INQUIRY;
    __obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
    __obuffer[3] = VISCA_PT_POSITION_INQ;
    __obuffer_length = 3;

    try {
#ifdef TIMETRACKER_VISCA
      __tt->ping_start( __ttc_pantilt_get_send );
      send();
      __tt->ping_end( __ttc_pantilt_get_send );
      __tt->ping_start( __ttc_pantilt_get_read );
      recv();
      __tt->ping_end( __ttc_pantilt_get_read );
#else
      send_with_reply();
#endif
    } catch (ViscaException &e) {
      // Ignore
    }
  }

#ifdef TIMETRACKER_VISCA
  __tt->ping_start( __ttc_pantilt_get_handle );
#endif

  while (__ibuffer[1] != VISCA_RESPONSE_COMPLETED) {
    // inquire return from socket 0, so this may occur if there
    // are other responses waiting, handle them...
    try {
      handle_response();
      recv();
    } catch (ViscaException &e) {
      // Ignore
    }
  }

#ifdef TIMETRACKER_VISCA
  __tt->ping_end( __ttc_pantilt_get_handle );
  __tt->ping_start( __ttc_pantilt_get_interpret );
#endif


  // Extract information from __ibuffer
  if ( __ibuffer[1] == VISCA_RESPONSE_COMPLETED ) {
    unsigned short int pan_val = 0;
    unsigned short int tilt_val = 0;

    pan_val |= (__ibuffer[2] & 0x0F) << 12;
    pan_val |= (__ibuffer[3] & 0x0F) << 8;
    pan_val |= (__ibuffer[4] & 0x0F) << 4;
    pan_val |= (__ibuffer[5] & 0x0F);

    tilt_val |= (__ibuffer[6] & 0x0F) << 12;
    tilt_val |= (__ibuffer[7] & 0x0F) << 8;
    tilt_val |= (__ibuffer[8] & 0x0F) << 4;
    tilt_val |= (__ibuffer[9] & 0x0F);

    if (pan_val < 0x8000) {
      // The value must be positive
      pan = pan_val;
    } else {
      // negative value
      pan = pan_val - 0xFFFF;
    }

    if (tilt_val < 0x8000) {
      // The value must be positive
      tilt = tilt_val;
    } else {
      // negative value
      tilt = tilt_val - 0xFFFF;
    }

  } else {
    throw ViscaException("getPanTilt(): Wrong response received");
  }
#ifdef TIMETRACKER_VISCA
  __tt->ping_end( __ttc_pantilt_get_interpret );
  __tt->print_to_stdout();
#endif

  __inquire = VISCA_RUNINQ_NONE;
}


/** Reset pan/tilt limit. */
void
Visca::reset_pan_tilt_limit()
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
  __obuffer[3] = VISCA_PT_LIMITSET;
  __obuffer[3] = VISCA_PT_LIMITSET_CLEAR;
  __obuffer[4] = VISCA_PT_LIMITSET_SET_UR;
  __obuffer[5] = 0x07;
  __obuffer[6] = 0x0F;
  __obuffer[7] = 0x0F;
  __obuffer[8] = 0x0F;
  __obuffer[9] = 0x07;
  __obuffer[10] = 0x0F;
  __obuffer[11] = 0x0F;
  __obuffer[12] = 0x0F;
  __obuffer_length = 12;

  try {
    send_with_reply();

    __obuffer[4] = VISCA_PT_LIMITSET_SET_DL;

    send_with_reply();
  } catch (ViscaException &e) {
    e.append("resetPanTiltLimit() failed");
    throw;
  }
}


/** Set pan tilt limit.
 * @param pan_left most left pan value
 * @param pan_right most right pan value
 * @param tilt_up most up tilt value
 * @param tilt_down most down tilt value
 */
void
Visca::set_pan_tilt_limit(int pan_left, int pan_right, int tilt_up, int tilt_down)
{
  try {
    __obuffer[1] = VISCA_COMMAND;
    __obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
    __obuffer[3] = VISCA_PT_LIMITSET;
    __obuffer[3] = VISCA_PT_LIMITSET_SET;
    __obuffer[4] = VISCA_PT_LIMITSET_SET_UR;
    // pan
    __obuffer[5] = (pan_right & 0xf000) >> 12;
    __obuffer[6] = (pan_right & 0x0f00) >>  8;
    __obuffer[7] = (pan_right & 0x00f0) >>  4;
    __obuffer[8] = (pan_right & 0x000f);
    // tilt
    __obuffer[9] = (tilt_up & 0xf000) >> 12;
    __obuffer[10] = (tilt_up & 0x0f00) >>  8;
    __obuffer[11] = (tilt_up & 0x00f0) >>  4;
    __obuffer[12] = (tilt_up & 0x000f);

    __obuffer_length = 12;

    send_with_reply();

    __obuffer[4] = VISCA_PT_LIMITSET_SET_DL;
    // pan
    __obuffer[5] = (pan_left & 0xf000) >> 12;
    __obuffer[6] = (pan_left & 0x0f00) >>  8;
    __obuffer[7] = (pan_left & 0x00f0) >>  4;
    __obuffer[8] = (pan_left & 0x000f);
    // tilt
    __obuffer[9] = (tilt_down & 0xf000) >> 12;
    __obuffer[10] = (tilt_down & 0x0f00) >>  8;
    __obuffer[11] = (tilt_down & 0x00f0) >>  4;
    __obuffer[12] = (tilt_down & 0x000f);

    send_with_reply();
  } catch (ViscaException &e) {
    e.append("setPanTiltLimit() failed");
    throw;
  }
}


/** Reset pan/tilt. */
void
Visca::reset_pan_tilt()
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
  __obuffer[3] = VISCA_PT_HOME;
  __obuffer_length = 3;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("resetPanTilt() failed");
    throw;
  }
}


/** Reset zoom. */
void
Visca::reset_zoom()
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_ZOOM;
  __obuffer[4] = VISCA_ZOOM_STOP;
  __obuffer_length = 4;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("resetZoom() failed");
    throw;
  }
}


/** Set zoom speed in tele.
 * @param speed speed
 */
void
Visca::set_zoom_speed_tele(unsigned int speed)
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_ZOOM;
  __obuffer[4] = VISCA_ZOOM_TELE_SPEED;
  // zoom speed
  __obuffer[5] = (speed & 0x000f) | 0x0020;
  __obuffer_length = 5;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("setZoomSpeedTele() failed");
    throw;
  }
}


/** Set zoom speed in wide angle.
 * @param speed speed
 */
void
Visca::set_zoom_speed_wide(unsigned int speed)
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_ZOOM;
  __obuffer[4] = VISCA_ZOOM_WIDE_SPEED;
  // zoom speed
  __obuffer[5] = (speed & 0x000f) | 0x0020;
  __obuffer_length = 5;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("setZoomSpeedWide() failed");
    throw;
  }
}


/** Set zoom.
 * @param zoom zoom value
 */
void
Visca::set_zoom(unsigned int zoom)
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_ZOOM_VALUE;
  // zoom
  __obuffer[4] = (zoom & 0xf000) >> 12;
  __obuffer[5] = (zoom & 0x0f00) >>  8;
  __obuffer[6] = (zoom & 0x00f0) >>  4;
  __obuffer[7] = (zoom & 0x000f);

  __obuffer_length = 7;

  try {
    if (! __blocking) {
      __nonblocking_running[ NONBLOCKING_ZOOM ] = true;
      send_nonblocking( &(__nonblocking_sockets[ NONBLOCKING_ZOOM ]) );
    } else {
      send_with_reply();
    }
  } catch (ViscaException &e) {
    e.append("setZoom() failed");
    throw;
  }
}


/** Get zoom.
 * @param zoom contains zoom upon return.
 */
void
Visca::get_zoom(unsigned int &zoom)
{
  __obuffer[1] = VISCA_INQUIRY;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_ZOOM_VALUE;
  __obuffer_length = 3;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("Failed to get zoom data");
    throw;
  }

  // Extract information from __ibuffer
  if ( __ibuffer[1] == VISCA_RESPONSE_COMPLETED ) {
    unsigned short int zoom_val = 0;

    zoom_val |= (__ibuffer[2] & 0x0F) << 12;
    zoom_val |= (__ibuffer[3] & 0x0F) << 8;
    zoom_val |= (__ibuffer[4] & 0x0F) << 4;
    zoom_val |= (__ibuffer[5] & 0x0F);

    zoom = zoom_val;
  } else {
    throw ViscaException("Failed to get zoom data failed, response code not VISCA_RESPONSE_COMPLETED");
  }

}


/** Enable or disable digital zoome.
 * @param enabled true to enable digital zoom, false to disable
 */
void
Visca::set_zoom_digital_enabled(bool enabled)
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_DZOOM;
  if (enabled) {
    __obuffer[4] = VISCA_DZOOM_ON;
  } else {
    __obuffer[4] = VISCA_DZOOM_OFF;
  }
  __obuffer_length = 4;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("setZoomDigitalEnabled() failed");
    throw;
  }
}


/** Apply effect.
 * @param filter filter
 */
void
Visca::apply_effect(unsigned char filter)
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_PICTURE_EFFECT;
  __obuffer[4] = filter;
  __obuffer_length = 4;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("applyEffect() failed");
    throw;
  }
}


/** Reset effects. */
void
Visca::reset_effect()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_OFF);
  } catch (ViscaException &e) {
    e.append("resetEffect() failed");
    throw;
  }
}


/** Apply pastel effect. */
void
Visca::apply_effect_pastel()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_PASTEL);
  } catch (ViscaException &e) {
    e.append("applyEffectPastel() failed");
    throw;
  }
}


/** Apply negative art effect. */
void
Visca::apply_effect_neg_art()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_NEGATIVE);
  } catch (ViscaException &e) {
    e.append("applyEffectNegArt() failed");
    throw;
  }
}


/** Apply sepia effect. */
void
Visca::apply_effect_sepia()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_SEPIA);
  } catch (ViscaException &e) {
    e.append("applyEffectSepia() failed");
    throw;
  }
}


/**Apply B/W effect */
void
Visca::apply_effect_bnw()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_BW);
  } catch (ViscaException &e) {
    e.append("applyEffectBnW() failed");
    throw;
  }
}


/** Apply solarize effect. */
void
Visca::apply_effect_solarize()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_SOLARIZE);
  } catch (ViscaException &e) {
    e.append("applyEffectSolarize() failed");
    throw;
  }
}


/** Apply mosaic effect. */
void
Visca::apply_effect_mosaic()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_MOSAIC);
  } catch (ViscaException &e) {
    e.append("applyEffectMosaic() failed");
    throw;
  }
}


/** Apply slim effect. */
void
Visca::apply_effect_slim()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_SLIM);
  } catch (ViscaException &e) {
    e.append("applyEffectSlim() failed");
    throw;
  }
}


/** Apply stretch effect. */
void
Visca::apply_effect_stretch()
{
  try {
    apply_effect(VISCA_PICTURE_EFFECT_STRETCH);
  } catch (ViscaException &e) {
    e.append("applyEffectStretch() failed");
    throw;
  }
}


/** Get white balance mode.
 * @return white balance mode
 */
unsigned int
Visca::get_white_balance_mode()
{
  __obuffer[1] = VISCA_INQUIRY;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_WB;
  __obuffer_length = 3;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("getWhiteBalanceMode() failed");
    throw;
  }

  while (__ibuffer[1] != VISCA_RESPONSE_COMPLETED) {
    // inquire return from socket 0, so this may occur if there
    // are other responses waiting, handle them...
    try {
      handle_response();
      recv();
    } catch (ViscaException &e) {
      e.append("getWhiteBalanceMode() failed");
      throw;
    }
  }

  // Extract information from __ibuffer
  if ( __ibuffer[1] == VISCA_RESPONSE_COMPLETED ) {
    return __ibuffer[2];
  } else {
    throw ViscaException("Did not get 'request completed' response");
  }

}


/** Sett mirror sate
 * @param mirror true to enable mirroring, false to disable
 */
void
Visca::set_mirror(bool mirror)
{
  __obuffer[1] = VISCA_COMMAND;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_MIRROR;
  __obuffer[4] = mirror ? VISCA_MIRROR_ON : VISCA_MIRROR_OFF;
  __obuffer_length = 4;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("set_mirror() failed");
    throw;
  }
}

/** Get mirror sate.
 * @return true if image is mirrored, false otherwise
 */
bool
Visca::get_mirror()
{
  __obuffer[1] = VISCA_INQUIRY;
  __obuffer[2] = VISCA_CATEGORY_CAMERA1;
  __obuffer[3] = VISCA_MIRROR;
  __obuffer_length = 3;

  try {
    send_with_reply();
  } catch (ViscaException &e) {
    e.append("Failed to get mirror data");
    throw;
  }

  // Extract information from __ibuffer
  if ( __ibuffer[1] == VISCA_RESPONSE_COMPLETED ) {
    return (__ibuffer[2] != 0);
  } else {
    throw ViscaException("Failed to get mirror data: zoom inquiry failed, "
			 "response code not VISCA_RESPONSE_COMPLETED");
  }
}
