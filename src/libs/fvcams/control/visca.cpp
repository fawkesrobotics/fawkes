
/***************************************************************************
 *  visca.cpp - Controller for Visca cams
 *
 *  Generated: Wed Jun 08 12:08:17 2005
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

#include <fvcams/control/visca.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <sys/time.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include <utils/system/console_colors.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ViscaControlException <fvcams/control/visca.h>
 * Visca exception.
 */

/** Constructor.
 * @param msg message of exception.
 */
ViscaControlException::ViscaControlException(const char *msg)
  : Exception(msg)
{
}


/** Constructor with errno.
 * @param msg message prefix
 * @param _errno errno for additional error information.
 */
ViscaControlException::ViscaControlException(const char *msg, const int _errno)
  : Exception(msg, _errno)
{
}

/** @class ViscaControlInquiryRunningException <fvcams/control/visca.h>
 * Visca inquire running exception.
 */

/** Constructor. */
ViscaControlInquiryRunningException::ViscaControlInquiryRunningException()
  : ViscaControlException("Inquiry already running")
{
}

/** Automatic white balance. */
const unsigned int ViscaControl::VISCA_WHITEBLANCE_AUTO      = VISCA_WB_AUTO;
/** Indoor white balance preset. */
const unsigned int ViscaControl::VISCA_WHITEBALANCE_INDOOR   = VISCA_WB_INDOOR;
/** Outdoor white balance preset. */
const unsigned int ViscaControl::VISCA_WHITEBALANCE_OUTDOOR  = VISCA_WB_OUTDOOR;
/** One push white balance preset. */
const unsigned int ViscaControl::VISCA_WHITEBALANCE_ONE_PUSH = VISCA_WB_ONE_PUSH;
/** ATW white balance preset. */
const unsigned int ViscaControl::VISCA_WHITEBALANCE_ATW      = VISCA_WB_ATW;
/** Manual white balance. */
const unsigned int ViscaControl::VISCA_WHITEBALANCE_MANUAL   = VISCA_WB_MANUAL;

/** @class ViscaControl <fvcams/control/visca.h>
 * Visca control protocol implementation over a serial line.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param blocking if true, operate in blocking mode, false to operate in non-blocking mode.
 */
ViscaControl::ViscaControl(bool blocking)
{
  opened = false;
  inquire = VISCA_RUNINQ_NONE;
  this->blocking = blocking;

  for (unsigned int i = 0; i < VISCA_NONBLOCKING_NUM; ++i) {
    nonblocking_sockets[i] = 0;
    nonblocking_running[i] = false;
  }
}


/** Open serial port.
 * @param port port to open.
 */
void
ViscaControl::open(const char *port) {

  struct termios param;

  dev = ::open(port, O_CREAT | O_RDWR | O_NONBLOCK);
  if (! dev) {
    throw ViscaControlException("Cannot open device", errno);
  }

  if (tcgetattr(dev, &param) == -1) {
    ViscaControlException ve("Getting the port parameters failed", errno);
    ::close(dev);
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
    
  tcflow (dev, TCOON);
  tcflow (dev, TCION);
    
  // number of data bits: 8
  param.c_cflag &= ~CS5 & ~CS6 & ~CS7 & ~CS8;

  param.c_cflag |= CS8;
    
  // parity: none
  param.c_cflag &=~(PARENB & PARODD);
    
  // stop bits: 1
  param.c_cflag &= ~CSTOPB;

  if (tcsetattr(dev, TCSANOW, &param) != 0) {
    ViscaControlException ve("Setting the port parameters failed", errno);
    ::close(dev);
    throw ve;
  }

  opened = true;
  // Choose first camera by default
  sender    = VISCA_BUS_0;
  recipient = VISCA_BUS_1;

#ifdef TIMETRACKER_VISCA
  tracker = new TimeTracker();
  track_file.open("tracker_visca.txt");
  ttcls_pantilt_get_send = tracker->addClass("getPanTilt: send");
  ttcls_pantilt_get_read = tracker->addClass("getPanTilt: read");
  ttcls_pantilt_get_handle = tracker->addClass("getPanTilt: handling responses");
  ttcls_pantilt_get_interpret = tracker->addClass("getPanTilt: interpreting");
#endif

  // success
}


/** Close port. */
void
ViscaControl::close()
{
  if (opened) {
    opened = false;
    ::close(dev);
  }
}


/** Set addresses of cameras.
 * @param num_cameras number of cameras on bus
 */
void
ViscaControl::set_address(unsigned int num_cameras)
{
  unsigned char recp_backup = recipient;
  recipient = VISCA_BUS_BROADCAST;
  obuffer[1] = 0x30;
  obuffer[2] = 0x01;
  obuffer_length = 2;

  try {
    send();
    recv(0);
  } catch (ViscaControlException &e) {
    e.append("set_address(%u) failed", num_cameras);
    throw;
  }

  recipient = recp_backup;
}


/** Clear */
void
ViscaControl::clear()
{
  if (!opened)  throw ViscaControlException("Serial port not open");

  obuffer[1] = 0x01;
  obuffer[2] = 0x00;
  obuffer[3] = 0x01;
  obuffer_length = 3;

  try {
    send();
    recv(0);
  } catch (ViscaControlException &e) {
    e.append("clear() failed");
    throw;
  }
}


/** Send outbound queue. */
void
ViscaControl::send()
{
  if (!opened)  throw ViscaControlException("Serial port not open");

  // Set first bit to 1
  obuffer[0] =  0x80;
  obuffer[0] |= (sender << 4);
  obuffer[0] |= recipient;

  obuffer[++obuffer_length] = VISCA_TERMINATOR;
  ++obuffer_length;

  int written = write(dev, obuffer, obuffer_length);
  //printf("ViscaControl sent: ");
  //for (int i = 0; i < obuffer_length; ++i) {
  //  printf("%02X", obuffer[i]);
  //}
  //printf("\n");
  if (written < obuffer_length) {
    throw ViscaControlException("Not all bytes send");
  }
}


/** Check data availability.
 * @return true if data is available, false otherwise
 */
bool
ViscaControl::data_available()
{
  int num_bytes = 0;
  ioctl(dev, FIONREAD, &num_bytes);
  return (num_bytes > 0);
}


/** Receive data.
 * @param max_wait_ms maximum wait time in miliseconds
 */
void
ViscaControl::recv(unsigned int max_wait_ms)
{
  try {
    recv_packet(max_wait_ms);
  } catch (ViscaControlException &e) {
    e.append("Receiving failed, recv_packet() call failed");
    throw;
  }

  // Get type of message
  unsigned char type = ibuffer[1] & 0xF0;
  while (type == VISCA_RESPONSE_ACK) {
    try {
      recv_packet(max_wait_ms);
    } catch (ViscaControlException &e) {
      e.append("Receiving failed, recv_packet() call 2 failed");
      throw;
    }
    type = ibuffer[1] & 0xF0;
  }

  switch (type) {
  case VISCA_RESPONSE_CLEAR:
  case VISCA_RESPONSE_ADDRESS:
  case VISCA_RESPONSE_COMPLETED:
  case VISCA_RESPONSE_ERROR:
    break;
  default:
    throw ViscaControlException("Receiving failed, unexpected packet type received");
  }
}


/** Receive ACK packet.
 * @param socket contains the socket that the ACK was received on upon return
 */
void
ViscaControl::recv_ack(unsigned int *socket)
{
  try {
    recv_packet(0);
  } catch (ViscaControlException &e) {
    throw ViscaControlException("recv_ack(): recv_packet() failed");
  }

  // Get type of message
  unsigned char type = ibuffer[1] & 0xF0;
  while (type != VISCA_RESPONSE_ACK) {

    try {
      handle_response();
      recv_packet();
    } catch (ViscaControlException &e) {
      e.append("Handling message of type %u failed", type);
      throw;
    }
    type = ibuffer[1] & 0xF0;
  }

  // Got an ack now
  if (socket != NULL) {
    *socket = ibuffer[1] & 0x0F;
  }

}


/** Send non-blocking.
 * Does a non-blocking send.
 * @param socket the socket that was used to send the request.
 */
void
ViscaControl::send_nonblocking(unsigned int *socket)
{
  try {
    send();
    recv_ack(socket);
  } catch (ViscaControlException &e) {
    e.append("Non-blocking send failed!");
    throw;
  }
}


/** Send and wait for reply, blocking.
 */
void
ViscaControl::send_with_reply()
{
  try {
    send();
    recv();
  } catch (ViscaControlException &e) {
    e.append("Sending with reply failed");
    throw;
  }
}


/** Receive a packet.
 * @param max_wait_ms maximum wait time in miliseconds
 */
void
ViscaControl::recv_packet(unsigned int max_wait_ms)
{
  // wait for message
  timeval start, now;
  unsigned int diff_msec = 0;
  gettimeofday(&start, NULL);

  int num_bytes = 0;
  ioctl(dev, FIONREAD, &num_bytes);
  while ( ((max_wait_ms == 0) || (diff_msec < max_wait_ms)) && (num_bytes == 0)) {
    usleep(max_wait_ms / 100);
    ioctl(dev, FIONREAD, &num_bytes);

    gettimeofday(&now, NULL);
    diff_msec  = (now.tv_sec  - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
  }
  if (num_bytes == 0) {
    throw ViscaControlException("recv_packet() failed: no bytes to read");
  }

  // get octets one by one
  int bytes_read = read(dev, ibuffer, 1);
  int pos = 0;
  while (ibuffer[pos] != VISCA_TERMINATOR) {
    bytes_read = read(dev, &ibuffer[++pos], 1);
    usleep(0);
  }
  ibuffer_length = pos + 1;
  //printf("ViscaControl read: ");
  //for (int i = 0; i < ibuffer_length; ++i) {
  //  printf("%02X", ibuffer[i]);
  //}
  //printf("\n");
}


/** Finish a non-blocking operation.
 * @param socket socket that the non-blocking operation was sent to
 */
void
ViscaControl::finish_nonblocking( unsigned int socket )
{
  for (unsigned int i = 0; i < VISCA_NONBLOCKING_NUM; ++i) {
    if (nonblocking_sockets[i] == socket) {
      nonblocking_sockets[i] = 0;
      nonblocking_running[i] = false;
      return;
    }
  }

  throw ViscaControlException("finish_nonblocking() failed: socket not found");
}


/** Handle incoming response.  */
void
ViscaControl::handle_response()
{
  unsigned int type = ibuffer[1] & 0xF0;
  unsigned int socket = ibuffer[1] & 0x0F;

  if (socket == 0) {
    // This is an inquire response, do NOT handle!
    throw ViscaControlException("handle_response(): Received an inquire response, can't handle");
  }

  if ( type == VISCA_RESPONSE_COMPLETED ) {
    // Command has been finished
    try {
      finish_nonblocking( ibuffer[1] & 0x0F );
    } catch (ViscaControlException &e) {
      // Ignore, happens sometimes without effect
      // e.append("handle_response() failed, could not finish non-blocking");
      // throw;
    }
  } else if ( type == VISCA_RESPONSE_ERROR ) {
    finish_nonblocking( ibuffer[1] & 0x0F );
    throw ViscaControlException("handle_response(): got an error message from camera");
  } else {
    ViscaControlException ve("Got unknown/unhandled response type");
    ve.append("Received message of type %u", type);
    throw ve;
  }

}


/** Cancel a running command.
 * @param socket socket that the command was send on
 */
void
ViscaControl::cancel_command( unsigned int socket )
{
  unsigned char cancel_socket = socket & 0x0000000F;

  obuffer[1] = VISCA_CANCEL | cancel_socket;
  obuffer_length = 1;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("cancel_command() failed");
    throw;
  }

  if (  ((ibuffer[1] & 0xF0) == VISCA_RESPONSE_ERROR) &&
	((ibuffer[1] & 0x0F) == cancel_socket) &&
	((ibuffer[2] == VISCA_ERROR_CANCELLED)) ) {
    return;
  } else {
    throw ViscaControlException("Command could not be cancelled");
  }
}


/** Process incoming data. */
void
ViscaControl::process()
{

  inquire = VISCA_RUNINQ_NONE;

  while (data_available()) {
    try {
      recv();
      handle_response();
    } catch (ViscaControlException &e) {
      // Ignore this error
      return;
    }
  }
}


/** Set pan tilt.
 * @param pan pan
 * @param tilt tilt
 */
void
ViscaControl::setPanTilt(int pan, int tilt)
{
  
  // we do not to check for blocking, could not be called at
  // the same time if blocking...
  /*
  if ( nonblocking_running[ VISCA_NONBLOCKING_PANTILT] ) {
    cout << "Cancelling old setPanTilt" << endl;
    if (cancel_command( nonblocking_sockets[ VISCA_NONBLOCKING_PANTILT ] ) != VISCA_SUCCESS) {
      cout << "ViscaControl: Could not cancel old non-blocking pan/tilt command. Not setting new pan/tilt." << endl;
      return VISCA_E_CANCEL;
    }
    nonblocking_running[ VISCA_NONBLOCKING_PANTILT ] = false;
  }
  */

  unsigned short int tilt_val = 0 + tilt;
  unsigned short int pan_val  = 0 + pan;

  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
  obuffer[3] = VISCA_PT_ABSOLUTE_POSITION;
  // pan speed
  obuffer[4] = 0x18; // max speed
  // tilt speed
  obuffer[5] = 0x14; // max speed

  // pan
  obuffer[6] = (pan_val & 0xf000) >> 12;
  obuffer[7] = (pan_val & 0x0f00) >>  8;
  obuffer[8] = (pan_val & 0x00f0) >>  4;
  obuffer[9] = (pan_val & 0x000f);
  // tilt
  obuffer[10] = (tilt_val & 0xf000) >> 12;
  obuffer[11] = (tilt_val & 0x0f00) >> 8;
  obuffer[12] = (tilt_val & 0x00f0) >> 4;
  obuffer[13] = (tilt_val & 0x000f);

  obuffer_length = 13;

  try {
    if (! blocking) {
      nonblocking_running[ VISCA_NONBLOCKING_PANTILT ] = true;
      send_nonblocking( &(nonblocking_sockets[ VISCA_NONBLOCKING_PANTILT ]) );
    } else {
      send_with_reply();
    }
  } catch (ViscaControlException &e) {
    e.append("setPanTilt() failed");
    throw;
  }
}


/** Initiate a pan/tilt request, but do not wait for the reply. */
void
ViscaControl::startGetPanTilt()
{

  if ( inquire )  throw ViscaControlInquiryRunningException();

  inquire = VISCA_RUNINQ_PANTILT;

  obuffer[1] = VISCA_INQUIRY;
  obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
  obuffer[3] = VISCA_PT_POSITION_INQ;
  obuffer_length = 3;

  try {
    send();
  } catch (ViscaControlException &e) {
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
ViscaControl::getPanTilt(int *pan, int *tilt)
{

  if ( inquire ) {
    if ( inquire != VISCA_RUNINQ_PANTILT ) {
      throw ViscaControlException("Inquiry running, but it is not a pan/tilt inquiry");
    } else {
#ifdef TIMETRACKER_VISCA
      tracker->pingStart( ttcls_pantilt_get_read );
#endif
      try {
	recv();
      } catch (ViscaControlException &e) {
	// Ignore
      }
#ifdef TIMETRACKER_VISCA
      tracker->pingEnd( ttcls_pantilt_get_read );
#endif
    }
  } else {

    obuffer[1] = VISCA_INQUIRY;
    obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
    obuffer[3] = VISCA_PT_POSITION_INQ;
    obuffer_length = 3;

    try {
#ifdef TIMETRACKER_VISCA
      tracker->pingStart( ttcls_pantilt_get_send );
      send();
      tracker->pingEnd( ttcls_pantilt_get_send );
      tracker->pingStart( ttcls_pantilt_get_read );
      recv();
      tracker->pingEnd( ttcls_pantilt_get_read );
#else
      send_with_reply();
#endif
    } catch (ViscaControlException &e) {
      // Ignore
    }
  }

#ifdef TIMETRACKER_VISCA
  tracker->pingStart( ttcls_pantilt_get_handle );
#endif

  while (ibuffer[1] != VISCA_RESPONSE_COMPLETED) {
    // inquire return from socket 0, so this may occur if there
    // are other responses waiting, handle them...
    try {
      handle_response();
      recv();
    } catch (ViscaControlException &e) {
      // Ignore
    }
  }

#ifdef TIMETRACKER_VISCA
  tracker->pingEnd( ttcls_pantilt_get_handle );
  tracker->pingStart( ttcls_pantilt_get_interpret );
#endif


  // Extract information from ibuffer
  if ( ibuffer[1] == VISCA_RESPONSE_COMPLETED ) {
    unsigned short int pan_val = 0;
    unsigned short int tilt_val = 0;

    pan_val |= (ibuffer[2] & 0x0F) << 12;
    pan_val |= (ibuffer[3] & 0x0F) << 8;
    pan_val |= (ibuffer[4] & 0x0F) << 4;
    pan_val |= (ibuffer[5] & 0x0F);

    tilt_val |= (ibuffer[6] & 0x0F) << 12;
    tilt_val |= (ibuffer[7] & 0x0F) << 8;
    tilt_val |= (ibuffer[8] & 0x0F) << 4;
    tilt_val |= (ibuffer[9] & 0x0F);

    if (pan_val < 0x8000) {
      // The value must be positive
      *pan = pan_val;
    } else {
      // negative value
      *pan = pan_val - 0xFFFF;
    }

    if (tilt_val < 0x8000) {
      // The value must be positive
      *tilt = tilt_val;
    } else {
      // negative value
      *tilt = tilt_val - 0xFFFF;
    }

  } else {
    throw ViscaControlException("getPanTilt(): Wrong response received");
  }
#ifdef TIMETRACKER_VISCA
  tracker->pingEnd( ttcls_pantilt_get_interpret );
  tracker->printToStream( track_file );
#endif

  inquire = VISCA_RUNINQ_NONE;
}


/** Reset pan/tilt limit. */
void
ViscaControl::resetPanTiltLimit()
{
  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
  obuffer[3] = VISCA_PT_LIMITSET;
  obuffer[3] = VISCA_PT_LIMITSET_CLEAR;
  obuffer[4] = VISCA_PT_LIMITSET_SET_UR;
  obuffer[5] = 0x07;
  obuffer[6] = 0x0F;
  obuffer[7] = 0x0F;
  obuffer[8] = 0x0F;
  obuffer[9] = 0x07;
  obuffer[10] = 0x0F;
  obuffer[11] = 0x0F;
  obuffer[12] = 0x0F;
  obuffer_length = 12;

  try {
    send_with_reply();

    obuffer[4] = VISCA_PT_LIMITSET_SET_DL;

    send_with_reply();
  } catch (ViscaControlException &e) {
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
ViscaControl::setPanTiltLimit(int pan_left, int pan_right, int tilt_up, int tilt_down)
{
  try {
    obuffer[1] = VISCA_COMMAND;
    obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
    obuffer[3] = VISCA_PT_LIMITSET;
    obuffer[3] = VISCA_PT_LIMITSET_SET;
    obuffer[4] = VISCA_PT_LIMITSET_SET_UR;
    // pan
    obuffer[5] = (pan_right & 0xf000) >> 12;
    obuffer[6] = (pan_right & 0x0f00) >>  8;
    obuffer[7] = (pan_right & 0x00f0) >>  4;
    obuffer[8] = (pan_right & 0x000f);
    // tilt
    obuffer[9] = (tilt_up & 0xf000) >> 12;
    obuffer[10] = (tilt_up & 0x0f00) >>  8;
    obuffer[11] = (tilt_up & 0x00f0) >>  4;
    obuffer[12] = (tilt_up & 0x000f);

    obuffer_length = 12;

    send_with_reply();

    obuffer[4] = VISCA_PT_LIMITSET_SET_DL;
    // pan
    obuffer[5] = (pan_left & 0xf000) >> 12;
    obuffer[6] = (pan_left & 0x0f00) >>  8;
    obuffer[7] = (pan_left & 0x00f0) >>  4;
    obuffer[8] = (pan_left & 0x000f);
    // tilt
    obuffer[9] = (tilt_down & 0xf000) >> 12;
    obuffer[10] = (tilt_down & 0x0f00) >>  8;
    obuffer[11] = (tilt_down & 0x00f0) >>  4;
    obuffer[12] = (tilt_down & 0x000f);

    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("setPanTiltLimit() failed");
    throw;
  }
}


/** Reset pan/tilt. */
void
ViscaControl::resetPanTilt()
{
  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_PAN_TILTER;
  obuffer[3] = VISCA_PT_HOME;
  obuffer_length = 3;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("resetPanTilt() failed");
    throw;
  }
}


/** Reset zoom. */
void
ViscaControl::resetZoom()
{
  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_CAMERA1;
  obuffer[3] = VISCA_ZOOM;
  obuffer[4] = VISCA_ZOOM_STOP;
  obuffer_length = 4;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("resetZoom() failed");
    throw;
  }
}


/** Set zoom speed in tele.
 * @param speed speed
 */
void
ViscaControl::setZoomSpeedTele(unsigned int speed)
{
  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_CAMERA1;
  obuffer[3] = VISCA_ZOOM;
  obuffer[4] = VISCA_ZOOM_TELE_SPEED;
  // zoom speed
  obuffer[5] = (speed & 0x000f) | 0x0020;
  obuffer_length = 5;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("setZoomSpeedTele() failed");
    throw;
  }
}


/** Set zoom speed in wide angle.
 * @param speed speed
 */
void
ViscaControl::setZoomSpeedWide(unsigned int speed)
{
  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_CAMERA1;
  obuffer[3] = VISCA_ZOOM;
  obuffer[4] = VISCA_ZOOM_WIDE_SPEED;
  // zoom speed
  obuffer[5] = (speed & 0x000f) | 0x0020;
  obuffer_length = 5;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("setZoomSpeedWide() failed");
    throw;
  }
}


/** Set zoom.
 * @param zoom zoom value
 */
void
ViscaControl::setZoom(unsigned int zoom)
{
  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_CAMERA1;
  obuffer[3] = VISCA_ZOOM_VALUE;
  // zoom
  obuffer[4] = (zoom & 0xf000) >> 12;
  obuffer[5] = (zoom & 0x0f00) >>  8;
  obuffer[6] = (zoom & 0x00f0) >>  4;
  obuffer[7] = (zoom & 0x000f);

  obuffer_length = 7;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("setZoom() failed");
    throw;
  }
}


/** Get zoom.
 * @param zoom contains zoom upon return.
 */
void
ViscaControl::getZoom(unsigned int *zoom)
{
  obuffer[1] = VISCA_INQUIRY;
  obuffer[2] = VISCA_CATEGORY_CAMERA1;
  obuffer[3] = VISCA_ZOOM_VALUE;
  obuffer_length = 3;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("getZoom() failed");
    throw;
  }

  // Extract information from ibuffer
  if ( ibuffer[1] == VISCA_RESPONSE_COMPLETED ) {
    unsigned short int zoom_val = 0;

    zoom_val |= (ibuffer[2] & 0x0F) << 12;
    zoom_val |= (ibuffer[3] & 0x0F) << 8;
    zoom_val |= (ibuffer[4] & 0x0F) << 4;
    zoom_val |= (ibuffer[5] & 0x0F);

    *zoom = zoom_val;
  } else {
    throw ViscaControlException("getZoom(): zoom inquiry failed, response code not VISCA_RESPONSE_COMPLETED");
  }

}


/** Enable or disable digital zoome.
 * @param enabled true to enable digital zoom, false to disable
 */
void
ViscaControl::setZoomDigitalEnabled(bool enabled)
{
  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_CAMERA1;
  obuffer[3] = VISCA_DZOOM;
  if (enabled) {
    obuffer[4] = VISCA_DZOOM_ON;
  } else {
    obuffer[4] = VISCA_DZOOM_OFF;
  }
  obuffer_length = 4;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("setZoomDigitalEnabled() failed");
    throw;
  }
}


/** Apply effect.
 * @param filter filter
 */
void
ViscaControl::applyEffect(unsigned char filter)
{
  obuffer[1] = VISCA_COMMAND;
  obuffer[2] = VISCA_CATEGORY_CAMERA1;
  obuffer[3] = VISCA_PICTURE_EFFECT;
  obuffer[4] = filter;
  obuffer_length = 4;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("applyEffect() failed");
    throw;
  }
}


/** Reset effects. */
void
ViscaControl::resetEffect()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_OFF);
  } catch (ViscaControlException &e) {
    e.append("resetEffect() failed");
    throw;
  }
}


/** Apply pastel effect. */
void
ViscaControl::applyEffectPastel()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_PASTEL);
  } catch (ViscaControlException &e) {
    e.append("applyEffectPastel() failed");
    throw;
  }
}


/** Apply negative art effect. */
void
ViscaControl::applyEffectNegArt()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_NEGATIVE);
  } catch (ViscaControlException &e) {
    e.append("applyEffectNegArt() failed");
    throw;
  }
}


/** Apply sepia effect. */
void
ViscaControl::applyEffectSepia()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_SEPIA);
  } catch (ViscaControlException &e) {
    e.append("applyEffectSepia() failed");
    throw;
  }
}


/**Apply B/W effect */
void
ViscaControl::applyEffectBnW()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_BW);
  } catch (ViscaControlException &e) {
    e.append("applyEffectBnW() failed");
    throw;
  }
}


/** Apply solarize effect. */
void
ViscaControl::applyEffectSolarize()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_SOLARIZE);
  } catch (ViscaControlException &e) {
    e.append("applyEffectSolarize() failed");
    throw;
  }
}


/** Apply mosaic effect. */
void
ViscaControl::applyEffectMosaic()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_MOSAIC);
  } catch (ViscaControlException &e) {
    e.append("applyEffectMosaic() failed");
    throw;
  }
}


/** Apply slim effect. */
void
ViscaControl::applyEffectSlim()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_SLIM);
  } catch (ViscaControlException &e) {
    e.append("applyEffectSlim() failed");
    throw;
  }
}


/** Apply stretch effect. */
void
ViscaControl::applyEffectStretch()
{
  try {
    applyEffect(VISCA_PICTURE_EFFECT_STRETCH);
  } catch (ViscaControlException &e) {
    e.append("applyEffectStretch() failed");
    throw;
  }
}


/** Get white balance mode.
 * @return white balance mode
 */
unsigned int
ViscaControl::getWhiteBalanceMode()
{
  obuffer[1] = VISCA_INQUIRY;
  obuffer[2] = VISCA_CATEGORY_CAMERA1;
  obuffer[3] = VISCA_WB;
  obuffer_length = 3;

  try {
    send_with_reply();
  } catch (ViscaControlException &e) {
    e.append("getWhiteBalanceMode() failed");
    throw;
  }

  while (ibuffer[1] != VISCA_RESPONSE_COMPLETED) {
    // inquire return from socket 0, so this may occur if there
    // are other responses waiting, handle them...
    try {
      handle_response();
      recv();
    } catch (ViscaControlException &e) {
      e.append("getWhiteBalanceMode() failed");
      throw;
    }
  }

  // Extract information from ibuffer
  if ( ibuffer[1] == VISCA_RESPONSE_COMPLETED ) {
    return ibuffer[2];
  } else {
    throw ViscaControlException("Did not get 'request completed' response");
  }

}

} // end namespace firevision
