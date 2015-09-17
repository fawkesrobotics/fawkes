
/***************************************************************************
 *  servo_chain.cpp - Robotis dynamixel servo chain driver
 *
 *  Created: Mon Mar 23 20:44:28 2015 (based on pantilt plugin)
 *  Copyright  2005-2015  Tim Niemueller [www.niemueller.de]
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

#include "servo_chain.h"

#include <core/exceptions/system.h>
#include <core/exceptions/software.h>

#include <utils/math/angle.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <sys/time.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <cstdarg>

const unsigned char DynamixelChain::BROADCAST_ID             = 0xfe; /**< BROADCAST_ID */
const unsigned int  DynamixelChain::MAX_POSITION             = 0x3ff; /**< MAX_POSITION */
const unsigned int  DynamixelChain::CENTER_POSITION          = 0x1ff; /**< CENTER_POSITION */
const unsigned int  DynamixelChain::MAX_SPEED                = 0x7ff; /**< MAX_SPEED */
const float         DynamixelChain::MAX_ANGLE_DEG            = 300; /**< MAX_ANGLE_DEG */
const float         DynamixelChain::MAX_ANGLE_RAD            = fawkes::deg2rad(DynamixelChain::MAX_ANGLE_DEG); /**< MAX_ANGLE_RAD */
const float         DynamixelChain::RAD_PER_POS_TICK         = DynamixelChain::MAX_ANGLE_RAD / (float)(DynamixelChain::MAX_POSITION); /**< RAD_PER_POS_TICK */
const float         DynamixelChain::POS_TICKS_PER_RAD        = (float)(DynamixelChain::MAX_POSITION) / DynamixelChain::MAX_ANGLE_RAD; /**< POS_TICKS_PER_RAD */
const float         DynamixelChain::SEC_PER_60DEG_12V        = 0.167; /**< SEC_PER_60DEG_12V */
const float         DynamixelChain::SEC_PER_60DEG_16V        = 0.126; /**< SEC_PER_60DEG_16V */

const unsigned char DynamixelChain::SRL_RESPOND_NONE         =  0; /**< SRL_RESPOND_NONE */
const unsigned char DynamixelChain::SRL_RESPOND_READ         =  1; /**< SRL_RESPOND_READ */
const unsigned char DynamixelChain::SRL_RESPOND_ALL          =  2; /**< SRL_RESPOND_ALL */

const unsigned char DynamixelChain::P_MODEL_NUMBER_L         =  0; /**< P_MODEL_NUMBER_L */
const unsigned char DynamixelChain::P_MODEL_NUMBER_H         =  1; /**< P_MODEL_NUMBER_H */
const unsigned char DynamixelChain::P_VERSION                =  2; /**< P_VERSION */
const unsigned char DynamixelChain::P_ID                     =  3; /**< P_ID */
const unsigned char DynamixelChain::P_BAUD_RATE              =  4; /**< P_BAUD_RATE */
const unsigned char DynamixelChain::P_RETURN_DELAY_TIME      =  5; /**< P_RETURN_DELAY_TIME */
const unsigned char DynamixelChain::P_CW_ANGLE_LIMIT_L       =  6; /**< P_CW_ANGLE_LIMIT_L */
const unsigned char DynamixelChain::P_CW_ANGLE_LIMIT_H       =  7; /**< P_CW_ANGLE_LIMIT_H */
const unsigned char DynamixelChain::P_CCW_ANGLE_LIMIT_L      =  8; /**< P_CCW_ANGLE_LIMIT_L */
const unsigned char DynamixelChain::P_CCW_ANGLE_LIMIT_H      =  9; /**< P_CCW_ANGLE_LIMIT_H */
const unsigned char DynamixelChain::P_SYSTEM_DATA2           = 10; /**< P_SYSTEM_DATA2 */
const unsigned char DynamixelChain::P_LIMIT_TEMPERATURE      = 11; /**< P_LIMIT_TEMPERATURE */
const unsigned char DynamixelChain::P_DOWN_LIMIT_VOLTAGE     = 12; /**< P_DOWN_LIMIT_VOLTAGE */
const unsigned char DynamixelChain::P_UP_LIMIT_VOLTAGE       = 13; /**< P_UP_LIMIT_VOLTAGE */
const unsigned char DynamixelChain::P_MAX_TORQUE_L           = 14; /**< P_MAX_TORQUE_L */
const unsigned char DynamixelChain::P_MAX_TORQUE_H           = 15; /**< P_MAX_TORQUE_H */
const unsigned char DynamixelChain::P_RETURN_LEVEL           = 16; /**< P_RETURN_LEVEL */
const unsigned char DynamixelChain::P_ALARM_LED              = 17; /**< P_ALARM_LED */
const unsigned char DynamixelChain::P_ALARM_SHUTDOWN         = 18; /**< P_ALARM_SHUTDOWN */
const unsigned char DynamixelChain::P_OPERATING_MODE         = 19; /**< P_OPERATING_MODE */
const unsigned char DynamixelChain::P_DOWN_CALIBRATION_L     = 20; /**< P_DOWN_CALIBRATION_L */
const unsigned char DynamixelChain::P_DOWN_CALIBRATION_H     = 21; /**< P_DOWN_CALIBRATION_H */
const unsigned char DynamixelChain::P_UP_CALIBRATION_L       = 22; /**< P_UP_CALIBRATION_L */
const unsigned char DynamixelChain::P_UP_CALIBRATION_H       = 23; /**< P_UP_CALIBRATION_H */

const unsigned char DynamixelChain::P_TORQUE_ENABLE          = 24; /**< P_TORQUE_ENABLE */
const unsigned char DynamixelChain::P_LED                    = 25; /**< P_LED */
const unsigned char DynamixelChain::P_CW_COMPLIANCE_MARGIN   = 26; /**< P_CW_COMPLIANCE_MARGIN */
const unsigned char DynamixelChain::P_CCW_COMPLIANCE_MARGIN  = 27; /**< P_CCW_COMPLIANCE_MARGIN */
const unsigned char DynamixelChain::P_CW_COMPLIANCE_SLOPE    = 28; /**< P_CW_COMPLIANCE_SLOPE */
const unsigned char DynamixelChain::P_CCW_COMPLIANCE_SLOPE   = 29; /**< P_CCW_COMPLIANCE_SLOPE */
const unsigned char DynamixelChain::P_GOAL_POSITION_L        = 30; /**< P_GOAL_POSITION_L */
const unsigned char DynamixelChain::P_GOAL_POSITION_H        = 31; /**< P_GOAL_POSITION_H */
const unsigned char DynamixelChain::P_GOAL_SPEED_L           = 32; /**< P_GOAL_SPEED_L */
const unsigned char DynamixelChain::P_GOAL_SPEED_H           = 33; /**< P_GOAL_SPEED_H */
const unsigned char DynamixelChain::P_TORQUE_LIMIT_L         = 34; /**< P_TORQUE_LIMIT_L */
const unsigned char DynamixelChain::P_TORQUE_LIMIT_H         = 35; /**< P_TORQUE_LIMIT_H */
const unsigned char DynamixelChain::P_PRESENT_POSITION_L     = 36; /**< P_PRESENT_POSITION_L */
const unsigned char DynamixelChain::P_PRESENT_POSITION_H     = 37; /**< P_PRESENT_POSITION_H */
const unsigned char DynamixelChain::P_PRESENT_SPEED_L        = 38; /**< P_PRESENT_SPEED_L */
const unsigned char DynamixelChain::P_PRESENT_SPEED_H        = 39; /**< P_PRESENT_SPEED_H */
const unsigned char DynamixelChain::P_PRESENT_LOAD_L         = 40; /**< P_PRESENT_LOAD_L */
const unsigned char DynamixelChain::P_PRESENT_LOAD_H         = 41; /**< P_PRESENT_LOAD_H */
const unsigned char DynamixelChain::P_PRESENT_VOLTAGE        = 42; /**< P_PRESENT_VOLTAGE */
const unsigned char DynamixelChain::P_PRESENT_TEMPERATURE    = 43; /**< P_PRESENT_TEMPERATURE */
const unsigned char DynamixelChain::P_REGISTERED_INSTRUCTION = 44; /**< P_REGISTERED_INSTRUCTION */
const unsigned char DynamixelChain::P_PAUSE_TIME             = 45; /**< P_PAUSE_TIME */
const unsigned char DynamixelChain::P_MOVING                 = 46; /**< P_MOVING */
const unsigned char DynamixelChain::P_LOCK                   = 47; /**< P_LOCK */
const unsigned char DynamixelChain::P_PUNCH_L                = 48; /**< P_PUNCH_L */
const unsigned char DynamixelChain::P_PUNCH_H                = 49; /**< P_PUNCH_H */

//--- Instructions ---
const unsigned char DynamixelChain::INST_PING                = 0x01; /**< INST_PING */
const unsigned char DynamixelChain::INST_READ                = 0x02; /**< INST_READ */
const unsigned char DynamixelChain::INST_WRITE               = 0x03; /**< INST_WRITE */
const unsigned char DynamixelChain::INST_REG_WRITE           = 0x04; /**< INST_REG_WRITE */
const unsigned char DynamixelChain::INST_ACTION              = 0x05; /**< INST_ACTION */
const unsigned char DynamixelChain::INST_RESET               = 0x06; /**< INST_RESET */
const unsigned char DynamixelChain::INST_DIGITAL_RESET       = 0x07; /**< INST_DIGITAL_RESET */
const unsigned char DynamixelChain::INST_SYSTEM_READ         = 0x0C; /**< INST_SYSTEM_READ */
const unsigned char DynamixelChain::INST_SYSTEM_WRITE        = 0x0D; /**< INST_SYSTEM_WRITE */
const unsigned char DynamixelChain::INST_SYNC_WRITE          = 0x83; /**< INST_SYNC_WRITE */
const unsigned char DynamixelChain::INST_SYNC_REG_WRITE      = 0x84; /**< INST_SYNC_REG_WRITE */

const unsigned char DynamixelChain::PACKET_OFFSET_ID         = 2; /**< PACKET_OFFSET_ID */
const unsigned char DynamixelChain::PACKET_OFFSET_LENGTH     = 3; /**< PACKET_OFFSET_LENGTH */
const unsigned char DynamixelChain::PACKET_OFFSET_INST       = 4; /**< PACKET_OFFSET_INST */
const unsigned char DynamixelChain::PACKET_OFFSET_PARAM      = 5; /**< PACKET_OFFSET_PARAM */
const unsigned char DynamixelChain::PACKET_OFFSET_ERROR      = 4; /**< PACKET_OFFSET_ERROR */

using namespace std;
using namespace fawkes;


/** @class DynamixelChain "servo_chain.h"
 * Class to access a chain of Robotis dynamixel servos.
 * One instance of this class communicates with a chain of up to 254 Robotis
 * ServoChain servos, which are uniquely identified with an ID. Before making use of
 * the chain, connect each servo individually and set its ID. See the
 * discover() method for more information about numbering of the servos.
 * To achieve a higher speed, it is recommended to set the status return level
 * to reply only on READ instructions. You can do this for the whole chain with
 * @code
 * servo_chain->set_status_return_level(DynamixelChain::BROADCAST_ID, DynamixelChain::SRL_RESPOND_READ);
 * @endcode
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param device_file device file of the serial port
 * @param default_timeout_ms the timeout to apply by default to reading operations
 * @param enable_echo_fix true to enable a fix for buggy serial adapters
 * @param enable_connection_stability true to enable explicit timeout handling
 * @param min_voltage minimum voltage to assume safe operation
 * @param max_voltage maximum voltage to assume safe operation
 */
DynamixelChain::DynamixelChain(const char *device_file, unsigned int default_timeout_ms, bool enable_echo_fix, bool enable_connection_stability, float min_voltage, float max_voltage)
{
  __default_timeout_ms          = default_timeout_ms;
  __device_file                 = strdup(device_file);
  __fd                          = -1;
  __obuffer_length              = 0;
  __ibuffer_length              = 0;
  __enable_echo_fix             = enable_echo_fix;
  __enable_connection_stability = enable_connection_stability;
  __min_voltage                 = min_voltage;
  __max_voltage                 = max_voltage;
  memset(__control_table, 0, DYNAMIXEL_MAX_NUM_SERVOS * DYNAMIXEL_CONTROL_TABLE_LENGTH);
  try {
    open();
  } catch (Exception &e) {
    free(__device_file);
    throw;
  }
}


/** Destructor. */
DynamixelChain::~DynamixelChain()
{
  free(__device_file);
}

/** Open serial port. */
void
DynamixelChain::open() {

  struct termios param;

  __fd = ::open(__device_file, O_NOCTTY | O_RDWR);
  if (__fd == -1) {
    throw CouldNotOpenFileException(__device_file, errno, "Cannot open device file");
  }
  tcflush(__fd, TCIOFLUSH);

  if (tcgetattr(__fd, &param) == -1) {
    Exception e(errno, "Getting the port parameters failed");
    ::close(__fd);
    __fd = -1;
    throw e;
  }

  cfsetospeed(&param, B57600);
  cfsetispeed(&param, B57600);

  param.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
  param.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK | PARMRK);

  // turn off hardware flow control
  param.c_iflag &= ~(IXON | IXOFF | IXANY);
  param.c_cflag &= ~CRTSCTS;

  param.c_cflag |= (CREAD | CLOCAL);
    
  // number of data bits: 8
  param.c_cflag &= ~CS5 & ~CS6 & ~CS7 & ~CS8;
  param.c_cflag |= CS8;
    
  // parity: none
  param.c_cflag &= ~(PARENB | PARODD);
  param.c_iflag &= ~(INPCK | ISTRIP);
    
  // stop bits: 1
  param.c_cflag &= ~CSTOPB;

  //enable raw output
  param.c_oflag &= ~OPOST;

  param.c_cc[VMIN]  = 1;
  param.c_cc[VTIME] = 0;
    
  tcflush(__fd, TCIOFLUSH);

  if (tcsetattr(__fd, TCSANOW, &param) != 0) {
    Exception e(errno, "Setting the port parameters failed");
    ::close(__fd);
    __fd = -1;
    throw e;
  }

  //char junk[1];
  //read(__fd, junk, 1);

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
DynamixelChain::close()
{
  if (__fd >= 0) {
    ::close(__fd);
    __fd = -1;
  }
}


/** Calculate the checksum for the given packet.
 * @param id servo ID
 * @param instruction instruction to send
 * @param params parameters in the message
 * @param plength length of the params array
 * @return checksum as defined in the ServoChain manual
 */
unsigned char
DynamixelChain::calc_checksum(const unsigned char id, const unsigned char instruction,
			   const unsigned char *params, const unsigned char plength)
{
  unsigned int checksum = id + instruction + plength+2;
  for (unsigned char i = 0; i < plength; ++i) {
    checksum += params[i];
  }

  return ~(checksum & 0xFF);
}


/** Send instruction packet.
 * @param id servo ID
 * @param instruction instruction to send
 * @param params parameters in the message
 * @param plength length of the params array
 */
void
DynamixelChain::send(const unsigned char id, const unsigned char instruction,
		  const unsigned char *params, const unsigned char plength)
{
  // Byte 0 and 1 must be 0xFF
  __obuffer[0] = 0xFF;
  __obuffer[1] = 0xFF;
  __obuffer[PACKET_OFFSET_ID]     = id;
  __obuffer[PACKET_OFFSET_LENGTH] = plength+2;
  __obuffer[PACKET_OFFSET_INST]   = instruction;

  unsigned int checksum = id + plength+2;

  for (unsigned char i = 0; i < plength; ++i) {
    __obuffer[PACKET_OFFSET_PARAM + i] = params[i];
    checksum += params[i];
  }

  // actually +5, but zero-based array, therefore index 4, but fifth value
  __obuffer[3 + plength+2] = calc_checksum(id, instruction, params, plength);
  __obuffer_length = plength+2 + 4 ; // 4 for 0xFF 0xFF ID LENGTH

#ifdef DEBUG_ServoChain_COMM
  printf("Sending: ");
  for (int i = 0; i < __obuffer_length; ++i) {
    printf("%X ", __obuffer[i]);
  }
  printf("\n");
#endif

  int written = write(__fd, __obuffer, __obuffer_length);
  //printf("Wrote %d bytes\n", written);

  if (__enable_echo_fix) {
    // For some reason we have to read the shit immediately, although ECHO is off
    int readd = 0;
    while (readd < __obuffer_length) {
      readd += read(__fd, __ibuffer + readd, __obuffer_length - readd);
    }
  #ifdef DEBUG_ServoChain_COMM
    printf("Read %d junk bytes: ", readd);
    for (int i = 0; i < readd; ++i) {
      printf("%X ", __ibuffer[i]);
    }
    printf("\n");
  #endif
  }
  if ( written < 0 ) {
    throw Exception(errno, "Failed to write ServoChain packet %x for %x", instruction, id);
  } else if (written < __obuffer_length) {
    throw Exception("Failed to write ServoChain packet %x for %x, only %d of %d bytes sent",
		    instruction, id, written, __obuffer_length);
  }
}


/** Receive a packet.
 * @param timeout_ms maximum wait time in miliseconds
 * @param exp_length expected number of bytes
 */
void
DynamixelChain::recv(const unsigned char exp_length, unsigned int timeout_ms)
{
  timeval timeout = {0, (suseconds_t)(timeout_ms == 0xFFFFFFFF ? __default_timeout_ms : timeout_ms)  * 1000};

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(__fd, &read_fds);

  int rv = 0;
  rv = select(__fd + 1, &read_fds, NULL, NULL, &timeout);

  if ( rv == -1 ) {
   throw Exception(errno, "Select on FD failed");
  } else if ( rv == 0 ) {
    //printf("Timeout, no data :-/\n");
    throw TimeoutException("Timeout reached while waiting for incoming ServoChain data");
  }

  __ibuffer_length = 0;

  // get octets one by one
  int bytes_read = 0;
  while (bytes_read < 6) {
#ifdef DEBUG_ServoChain_COMM
    printf("Trying to read %d bytes\n", 6 - bytes_read);
#endif
    if (__enable_connection_stability) {
        // select file descriptor again to make sure data is available
        rv = select(__fd + 1, &read_fds, NULL, NULL, &timeout);

        if ( rv == -1 ) {
         throw Exception(errno, "Select on FD failed");
        } else if ( rv == 0 ) {
          //printf("Timeout, no data :-/\n");
          throw TimeoutException("Timeout reached while waiting for incoming ServoChain data");
        }
    }
    bytes_read += read(__fd, __ibuffer + bytes_read, 6 - bytes_read);
#ifdef DEBUG_ServoChain_COMM
    printf("%d bytes read  ", bytes_read);
    for (int i = 0; i < bytes_read; ++i) {
      printf("%X ", __ibuffer[i]);
    }
    printf("\n");
#endif
  }
  if (bytes_read < 6) {
    throw Exception("Failed to read packet header");
  }
  if ( (__ibuffer[0] != 0xFF) || (__ibuffer[1] != 0xFF) ) {
    throw Exception("Packet does not start with 0xFFFF.");
  }
  if (exp_length != __ibuffer[PACKET_OFFSET_LENGTH] - 2) {
    tcflush(__fd, TCIFLUSH);
    throw Exception("Wrong packet length, expected %u, got %u",
		    exp_length,  __ibuffer[PACKET_OFFSET_LENGTH] - 2);
  }
  const unsigned char plength = __ibuffer[PACKET_OFFSET_LENGTH] - 2;
#ifdef DEBUG_ServoChain_COMM
  printf("header read, params have length %d\n", plength);
#endif
  if (plength > 0) {
    bytes_read = 0;
    while (bytes_read < plength) {
      if (__enable_connection_stability) {
          // select file descriptor again to make sure data is available
          rv = select(__fd + 1, &read_fds, NULL, NULL, &timeout);

          if ( rv == -1 ) {
           throw Exception(errno, "Select on FD failed");
          } else if ( rv == 0 ) {
            //printf("Timeout, no data :-/\n");
            throw TimeoutException("Timeout reached while waiting for incoming ServoChain data");
          }
      }
      bytes_read += read(__fd, &__ibuffer[6] + bytes_read, plength - bytes_read);
    }
    if (bytes_read < plength) {
      throw Exception("Failed to read packet data");
    }
  }

  __ibuffer_length = plength+2 + 4;
#ifdef DEBUG_ServoChain_COMM
  printf("Read: ");
  for (int i = 0; i < __ibuffer_length; ++i) {
    printf("%X ", __ibuffer[i]);
  }
  printf("\n");
#endif

  // verify checksum
  unsigned char checksum = calc_checksum(__ibuffer[PACKET_OFFSET_ID],
					 __ibuffer[PACKET_OFFSET_INST],
					 &__ibuffer[PACKET_OFFSET_PARAM], plength);
  if (checksum != __ibuffer[plength + 5]) {
    throw Exception("Checksum error while receiving packet, expected %d, got %d",
		    checksum, __ibuffer[plength + 5]);
  }

  __ibuffer_length = plength+2 + 4;
}


/** Check data availability.
 * @return true if data is available, false otherwise
 */
bool
DynamixelChain::data_available()
{
  int num_bytes = 0;
  ioctl(__fd, FIONREAD, &num_bytes);
  return (num_bytes > 0);
}


/** Discover devices on the bus.
 * This method will send a PING instruction to either the broadcast ID 
 * (servos.size() == 0) or by iterating the given servos vector and collect responses.
 * This assumes that the return delay time is set appropriately 
 * that all responses can be received without collisions, and that the difference
 * between the time of two consecutive servos is smaller than the given timeout
 * (note that this might be void if you have one servo with ID 1 and one with
 * ID 253). After sending the packet this method will do up to
 * DYNAMIXEL_MAX_NUM_SERVOS receive operations, each with the given timeout. After the
 * first timeout the discovery is aborted assuming that all replies have been
 * received. You can set the timeout really high (several seconds) to be sure
 * that all connected servos are recognized.
 * For this to work best it is recommended to set consecutive servo IDs starting
 * from 1 on the servos.
 * After the servos are found, the control tables of all recognized servos are
 * received to ensure that all other methods return valid data.
 * @param timeout_ms maximum timeout to wait for replies.
 * @param servos vector of desired servos on the bus, can be an empty vector
 * to make use of the broadcast (all available servos).
 * @return list of detected servo IDs
 */
DynamixelChain::DeviceList
DynamixelChain::discover(unsigned int timeout_ms, const std::vector<unsigned int> servos)
{
  DeviceList rv;
  if (servos.size() == 0) {
    // send ping to broadcast address because we want all available servos
    // on the bus
    // simply re-throw exception upwards
    send(BROADCAST_ID, INST_PING, NULL, 0);
    for (unsigned int i = 0; i < DYNAMIXEL_MAX_NUM_SERVOS ; ++i) {
      try {
        recv(0, timeout_ms);
        rv.push_back(__ibuffer[PACKET_OFFSET_ID]);
      } catch (TimeoutException &e) {
        // the first timeout, no more devices can be expected to respond
        break;
      }
    }
  }
  else {
    // discover servos given by the servos-vector considering the defined timeout
    for (std::vector<unsigned int>::const_iterator iS = servos.begin() ; iS != servos.end(); ++iS)
      try {
        send(*iS, INST_PING, NULL, 0);
        recv(0, timeout_ms);
        rv.push_back(__ibuffer[PACKET_OFFSET_ID]);
      } catch (TimeoutException &e) {
        throw Exception("Desired servo with ID %u not found!", *iS);
      }
    }

  // now get data about all servos
  for (DeviceList::iterator i = rv.begin(); i != rv.end(); ++i) {
    try {
      read_table_values(*i);
    } catch (Exception &e) {
      e.append("Failed to receive control table for servo %u", *i);
      throw;
    }
  }
 
  return rv;
}


/** Ping servo.
 * This pings the given servo by sending a PING instruction and
 * reading the reply.
 * @param id servo ID, not the broadcast ID
 * @param timeout_ms maximum wait time in miliseconds
 * @return true if the ping was successful, false otherwise
 */
bool
DynamixelChain::ping(unsigned char id, unsigned int timeout_ms)
{
  assert_valid_id(id);
  try {
    send(id, INST_PING, NULL, 0);
    recv(0, timeout_ms);
    return true;
  } catch (Exception &e) {
    e.print_trace();
    return false;
  }
}


/** Read all table values for given servo.
 * This issues a READ comment for the whole control table and waits for the
 * response.
 * @param id servo ID
 */
void
DynamixelChain::read_table_values(unsigned char id)
{
  start_read_table_values(id);
  finish_read_table_values();
}


/** Start to receive table values.
 * This method sends a READ instruction packet for the whole table, but it does
 * not wait for the reply. This can be used to overlap the receiving with other
 * operations. You have to ensure to call finish_read_table_values() before
 * sending any other data.
 * @param id servo ID, not the broadcast ID
 */
void
DynamixelChain::start_read_table_values(unsigned char id)
{
  assert_valid_id(id);
  unsigned char param[2];
  param[0] = 0x00;
  param[1] = DYNAMIXEL_CONTROL_TABLE_LENGTH;

  send(id, INST_READ, param, 2);
}

/** Finish control table receive operations.
 * This executes the receive operation initiated by start_read_table_values().
 * This will read the values and write the output to the control table
 * (in memory, not in the servo), such that the appropriate get methods will
 * return the new data.
 */
void
DynamixelChain::finish_read_table_values()
{
  recv(DYNAMIXEL_CONTROL_TABLE_LENGTH);

  if (__ibuffer_length != 5 + DYNAMIXEL_CONTROL_TABLE_LENGTH + 1) {
    throw Exception("Input buffer of invalid size: %u vs. %u", __ibuffer_length, 5 + DYNAMIXEL_CONTROL_TABLE_LENGTH + 1);
  }
  memcpy(__control_table[__ibuffer[PACKET_OFFSET_ID]],
	 &__ibuffer[PACKET_OFFSET_PARAM], DYNAMIXEL_CONTROL_TABLE_LENGTH);
}


/** Read a table value.
 * This will read the given value(s) and write the output to the control table
 * (in memory, not in the servo), such that the appropriate get method will return
 * the new value.
 * @param id servo ID, not the broadcast ID
 * @param addr start addr, one of the P_* constants.
 * @param read_length number of bytes to read
 */
void
DynamixelChain::read_table_value(unsigned char id,
			      unsigned char addr, unsigned char read_length)
{
  assert_valid_id(id);

  unsigned char param[2];
  param[0] = addr;
  param[1] = read_length;

  send(id, INST_READ, param, 2);
  recv(read_length);

  if (__ibuffer_length != (5 + read_length + 1)) {
    throw Exception("Input buffer not of expected size, expected %u, got %u",
		    (5 + read_length + 1), __ibuffer_length);
  }

  for (unsigned int i = 0; i < read_length; ++i) {
    __control_table[id][addr + i] = __ibuffer[PACKET_OFFSET_PARAM + i];
  }
}


/** Write a table value.
 * @param id servo ID, may be the broadcast ID
 * @param addr start addr, one of the P_* constants.
 * @param value value to write
 * @param double_byte if true, will assume value to be a two-byte value, otherwise
 * it is considered as a one-byte value.
 */
void
DynamixelChain::write_table_value(unsigned char id, unsigned char addr,
			       unsigned int value, bool double_byte)
{
  unsigned char param[3];
  param[0] = addr;
  param[1] = value & 0xFF;
  param[2] = (value >> 8) & 0xFF;

  try {
    send(id, INST_WRITE, param, double_byte ? 3 : 2);

    if (id == BROADCAST_ID) {
      for (unsigned int i = 0; i < DYNAMIXEL_MAX_NUM_SERVOS; ++i) {
	__control_table[i][addr] = param[1];
	if (double_byte) __control_table[i][addr + 1] = param[2];
      }
    } else {
      __control_table[id][addr] = param[1];
      if (double_byte) __control_table[id][addr + 1] = param[2];
    }

    if ((id != BROADCAST_ID) && responds_all(id))  recv(0);
  } catch (Exception &e) {
    e.print_trace();
    throw;
  }
}


/** Write multiple table values.
 * @param id servo ID, may be the broadcast ID
 * @param start_addr start addr, one of the P_* constants.
 * @param values values to write
 * @param num_values length in bytes of the values array
 */
void
DynamixelChain::write_table_values(unsigned char id, unsigned char start_addr,
				unsigned char *values, unsigned int num_values)
{
  unsigned char param[num_values + 1];
  param[0] = start_addr;
  for (unsigned int i = 0; i < num_values; ++i) {
    param[i + 1] = values[i];
  }

  try {
    send(id, INST_WRITE, param, num_values + 1);

    if (id == BROADCAST_ID) {
      for (unsigned int i = 0; i < DYNAMIXEL_MAX_NUM_SERVOS; ++i) {
	for (unsigned int j = 0; j < num_values; ++j) {
	  __control_table[i][start_addr + j] = values[j];
	}
      }
    } else {
      for (unsigned int j = 0; j < num_values; ++j) {
	__control_table[id][start_addr + j] = values[j];
      }
    }

    if ((id != BROADCAST_ID) && responds_all(id))  recv(0);
  } catch (Exception &e) {
    e.print_trace();
    throw;
  }
}


/** Assert that the ID is valid.
 * @exception Exception thrown if \p id is the broadcast ID
 * @exception OutOfBoundsException thrown if the number is greater than the
 * maximum number of servos.
 */
void
DynamixelChain::assert_valid_id(unsigned char id)
{
  if (id == BROADCAST_ID) {
    throw Exception("Data can only be queried for a specific servo");
  } else if (id >= DYNAMIXEL_MAX_NUM_SERVOS) {
    throw OutOfBoundsException("Servo ID out of bounds", id, 0, DYNAMIXEL_MAX_NUM_SERVOS);
  }
}


/** Merge two values to a two-byte value.
 * @param id servo id, not the broadcast ID
 * @param ind_l low index in control table
 * @param ind_h high index in control table
 */
unsigned int
DynamixelChain::merge_twobyte_value(unsigned int id,
				 unsigned char ind_l, unsigned char ind_h)
{
  unsigned int rv = (__control_table[id][ind_h] & 0xFF) << 8;
  rv |= __control_table[id][ind_l] & 0xFF;
  return rv;
}


/** Get a value from the control table, possibly from servo.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @param ind_l low index in control table
 * @param ind_h high index in control table, only set if value is a
 * two-byte value.
 * @return value
 */
unsigned int
DynamixelChain::get_value(unsigned int id, bool refresh,
		       unsigned int ind_l, unsigned int ind_h)
{
  assert_valid_id(id);

  if (refresh)  read_table_value(id, ind_l, (ind_h == 0xFFFFFFFF ? 1 : 2));

  if (ind_h == 0xFFFFFFFF) {
    return __control_table[id][ind_l];
  } else {
    return merge_twobyte_value(id, ind_l, ind_h);
  }
}

/** Get model string.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return model name
 */
const char *
DynamixelChain::get_model(unsigned char id, bool refresh)
{
  const unsigned int model = get_model_number(id, refresh);
  switch (model) {
  case  12: return "AX-12";
  case  18: return "AX-18";
  case  24: return "RX-24F";
  case  28: return "RX-28";
  case  29: return "MX-28";
  case  54: return "MX-64";
  case  64: return "RX-64";
  case 104: return "MAX-12W";
  case 107: return "EX-106+";
  case 310: return "MX-64AT";
  case 320: return "MX-106";
  default:  return "UNKNOWN";
  }
}

/** Get model.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return model
 */
unsigned int
DynamixelChain::get_model_number(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_MODEL_NUMBER_L, P_MODEL_NUMBER_H);
}


/** Get current position.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return current position
 */
unsigned int
DynamixelChain::get_position(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_PRESENT_POSITION_L, P_PRESENT_POSITION_H);
}


/** Get firmware version.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return firmware version
 */
unsigned char
DynamixelChain::get_firmware_version(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_VERSION);
}


/** Get baud rate.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return baud rate
 */
unsigned char
DynamixelChain::get_baudrate(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_BAUD_RATE);
}


/** Get time of the delay before replies are sent.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return delay time
 */
unsigned char
DynamixelChain::get_delay_time(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_RETURN_DELAY_TIME);
}


/** Get error flags set by the servo
 * @param id servo ID, not the broadcast ID
 * @return error flags
 */
unsigned char
DynamixelChain::get_error(unsigned char id)
{
  return __ibuffer[PACKET_OFFSET_ERROR];
}


/** Get angle limits.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @param cw_limit upon return contains the clockwise angle limit
 * @param ccw_limit upon return contains the counter-clockwise angle limit
 */
void
DynamixelChain::get_angle_limits(unsigned char id,
			      unsigned int &cw_limit, unsigned int &ccw_limit,
			      bool refresh)
{
  cw_limit  = get_value(id, refresh, P_CW_ANGLE_LIMIT_L, P_CW_ANGLE_LIMIT_H);
  ccw_limit = get_value(id, refresh, P_CCW_ANGLE_LIMIT_L, P_CCW_ANGLE_LIMIT_H);
}


/** Get temperature limit.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return temperature limit.
 */
unsigned char
DynamixelChain::get_temperature_limit(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_LIMIT_TEMPERATURE);
}


/** Get voltage limits.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @param low upon return contains low voltage limit
 * @param high upon return contans high voltage limit
 */
void
DynamixelChain::get_voltage_limits(unsigned char id,
			      unsigned char &low, unsigned char &high,
			      bool refresh)
{
  low  = get_value(id, refresh, P_DOWN_LIMIT_VOLTAGE);
  high = get_value(id, refresh, P_UP_LIMIT_VOLTAGE);
}


/** Get maximum torque.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return maximum torque
 */
unsigned int
DynamixelChain::get_max_torque(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_MAX_TORQUE_L, P_MAX_TORQUE_H);
}


/** Get status return level.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return status return level
 */
unsigned char
DynamixelChain::get_status_return_level(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_RETURN_LEVEL);
}


/** Get alarm LED status.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return alarm LED status.
 */
unsigned char
DynamixelChain::get_alarm_led(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_ALARM_LED);
}


/** Get shutdown on alarm state.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return shutdown on alarm state
 */
unsigned char
DynamixelChain::get_alarm_shutdown(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_ALARM_SHUTDOWN);
}


/** Get calibration data.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @param down_calib downward calibration
 * @param up_calib upward calibration
 */
void
DynamixelChain::get_calibration(unsigned char id,
			     unsigned int &down_calib, unsigned int &up_calib,
			     bool refresh)
{
  down_calib = get_value(id, refresh, P_DOWN_CALIBRATION_L, P_DOWN_CALIBRATION_H);
  up_calib   = get_value(id, refresh, P_UP_CALIBRATION_L, P_UP_CALIBRATION_H);
}


/** Check if torque is enabled
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return true if torque is enabled, false otherwise
 */
bool
DynamixelChain::is_torque_enabled(unsigned char id, bool refresh)
{
  return (get_value(id, refresh, P_TORQUE_ENABLE) == 1);
}


/** Check if LED is enabled
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return true if led is enabled, false otherwise.
 */
bool
DynamixelChain::is_led_enabled(unsigned char id, bool refresh)
{
  return (get_value(id, refresh, P_LED) == 1);
}


/** Get compliance values.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @param cw_margin upon return contains clockwise margin
 * @param cw_slope upon return contains clockwise slope
 * @param ccw_margin upon return contains counter-clockwise margin
 * @param ccw_slope upon return contains counter-clockwise slope
 */
void
DynamixelChain::get_compliance_values(unsigned char id,
				   unsigned char &cw_margin, unsigned char &cw_slope,
				   unsigned char &ccw_margin, unsigned char &ccw_slope,
				   bool refresh)
{
  cw_margin  = get_value(id, refresh, P_CW_COMPLIANCE_MARGIN);
  cw_slope   = get_value(id, refresh, P_CW_COMPLIANCE_SLOPE);
  ccw_margin = get_value(id, refresh, P_CCW_COMPLIANCE_MARGIN);
  ccw_slope  = get_value(id, refresh, P_CCW_COMPLIANCE_SLOPE);
}


/** Get goal position.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return goal position
 */
unsigned int
DynamixelChain::get_goal_position(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_GOAL_POSITION_L, P_GOAL_POSITION_H);
}


/** Get goal speed.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return goal speed
 */
unsigned int
DynamixelChain::get_goal_speed(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_GOAL_SPEED_L, P_GOAL_SPEED_H);
}


/** Get maximum supported speed.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return maximum supported speed in rad/s
 */
float
DynamixelChain::get_max_supported_speed(unsigned char id, bool refresh)
{
  float voltage = get_voltage(id, refresh) / 10.0;

  if ((voltage < __min_voltage) || (voltage > __max_voltage)) {
    throw OutOfBoundsException("Voltage is outside of specs", voltage, __min_voltage, __max_voltage);
  }

  float sec_per_deg_12V = SEC_PER_60DEG_12V / 60.0;
  float sec_per_deg_16V = SEC_PER_60DEG_16V / 60.0;

  float range_sec_per_deg = sec_per_deg_12V - sec_per_deg_16V;
  float pos = voltage - 12.0;

  float sec_per_deg = sec_per_deg_16V + pos * range_sec_per_deg;
  float deg_per_sec = 1.0 / sec_per_deg;

  return deg2rad(deg_per_sec);
}


/** Get torque limit.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return torque limit
 */
unsigned int
DynamixelChain::get_torque_limit(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_TORQUE_LIMIT_L, P_TORQUE_LIMIT_H);
}


/** Get current speed.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return current speed
 */
unsigned int
DynamixelChain::get_speed(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_PRESENT_SPEED_L, P_PRESENT_SPEED_H);
}


/** Get current load.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return current load
 */
unsigned int
DynamixelChain::get_load(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_PRESENT_LOAD_L, P_PRESENT_LOAD_H);
}


/** Get current voltage.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return voltage, divide by 10 to get V
 */
unsigned char
DynamixelChain::get_voltage(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_PRESENT_VOLTAGE);
}


/** Get temperature.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return temperature in degrees Celsius
 */
unsigned char
DynamixelChain::get_temperature(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_PRESENT_TEMPERATURE);
}


/** Check if servo is moving.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return true if servo is moving, false otherwise
 */
bool
DynamixelChain::is_moving(unsigned char id, bool refresh)
{
  return (get_value(id, refresh, P_MOVING) == 1);
}


/** Check is servo is locked.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return true if servo config is locked, false otherwise
 */
bool
DynamixelChain::is_locked(unsigned char id, bool refresh)
{
  return (get_value(id, refresh, P_LOCK) == 1);
}


/** Get punch.
 * @param id servo ID, not the broadcast ID
 * @param refresh if true, will issue a read command for the value
 * @return punch
 */
unsigned int
DynamixelChain::get_punch(unsigned char id, bool refresh)
{
  return get_value(id, refresh, P_PUNCH_L, P_PUNCH_H);
}


/** Set ID.
 * @param id servo ID
 * @param new_id new ID to set
 */
void
DynamixelChain::set_id(unsigned char id, unsigned char new_id)
{
  write_table_value(id, P_ID, new_id);
}


/** Set baud rate.
 * @param id servo ID
 * @param baudrate new baudrate
 */
void
DynamixelChain::set_baudrate(unsigned char id, unsigned char baudrate)
{
  write_table_value(id, P_BAUD_RATE, baudrate);
}


/** Set return delay time.
 * @param id servo ID
 * @param return_delay_time new return delay time
 */
void
DynamixelChain::set_return_delay_time(unsigned char id, unsigned char return_delay_time)
{
  write_table_value(id, P_RETURN_DELAY_TIME, return_delay_time);
}


/** Set angle limits.
 * @param id servo ID
 * @param cw_limit new clockwise limit
 * @param ccw_limit new counter-clockwise limit
 */
void
DynamixelChain::set_angle_limits(unsigned char id,
			      unsigned int cw_limit, unsigned int ccw_limit)
{
  write_table_value(id, P_CW_ANGLE_LIMIT_L, cw_limit, true);
  write_table_value(id, P_CCW_ANGLE_LIMIT_L, ccw_limit, true);
}


/** Set temperature limit.
 * @param id servo ID
 * @param temp_limit new temperature limit (in degrees Celsius)
 */
void
DynamixelChain::set_temperature_limit(unsigned char id, unsigned char temp_limit)
{
  write_table_value(id, P_LIMIT_TEMPERATURE, temp_limit);
}


/** Set voltage limits.
 * @param id servo ID
 * @param low lower bound (give Volts * 10)
 * @param high higher bound (give Volts * 10)
 */
void
DynamixelChain::set_voltage_limits(unsigned char id, unsigned char low, unsigned char high)
{
  unsigned char param[2];
  param[0] = low;
  param[1] = high;
  write_table_values(id, P_DOWN_LIMIT_VOLTAGE, param, 2);
}


/** Set maximum torque.
 * @param id servo ID
 * @param max_torque new maximum torque
 */
void
DynamixelChain::set_max_torque(unsigned char id, unsigned int max_torque)
{
  write_table_value(id, P_MAX_TORQUE_L, max_torque, true);
}


/** Set status return level
 * @param id servo ID
 * @param status_return_level status return level, one of SRL_RESPOND_NONE,
 * SRL_RESPOND_READ or SRL_RESPOND_ALL.
 */
void
DynamixelChain::set_status_return_level(unsigned char id, unsigned char status_return_level)
{
  write_table_value(id, P_RETURN_LEVEL, status_return_level);
}


/** Set alarm LED settings.
 * @param id servo ID
 * @param alarm_led new LED alarm value.
 */
void
DynamixelChain::set_alarm_led(unsigned char id, unsigned char alarm_led)
{
  write_table_value(id, P_ALARM_LED, alarm_led);
}


/** Set shutdown on alarm.
 * @param id servo ID
 * @param alarm_shutdown alarm shutdown settings
 */
void
DynamixelChain::set_alarm_shutdown(unsigned char id, unsigned char alarm_shutdown)
{
  write_table_value(id, P_ALARM_SHUTDOWN, alarm_shutdown);
}


/** Enable or disable torque.
 * @param id servo ID
 * @param enabled true to enable (servo is powered) false to disable
 * (servo power disabled, servo can be freely moved manually)
 */
void
DynamixelChain::set_torque_enabled(unsigned char id, bool enabled )
{
  write_table_value(id, P_TORQUE_ENABLE, enabled ? 1 : 0);
}


/** Enable or disable torque for multiple (selected) servos at once.
 * Given the number of servos the same number of variadic arguments must be
 * passed, one for each servo ID that should be enabled/disabled.
 * @param enabled true to enable (servo is powered) false to disable
 * (servo power disabled, servo can be freely moved manually)
 * @param num_servos number of servos to set, maximum is 120
 */
void
DynamixelChain::set_torques_enabled(bool enabled, unsigned char num_servos, ...)
{
  if (num_servos > 120) {
    // not enough space for everything in the parameters..
    throw Exception("You cannot set more than 120 servos at once");
  }

  va_list arg;
  va_start(arg, num_servos);

  unsigned int  plength = 2 * num_servos + 2;
  unsigned char param[plength];
  param[0] = P_TORQUE_ENABLE;
  param[1] = 1;
  for (unsigned int i = 0; i < num_servos; ++i) {
    unsigned char id    = va_arg(arg, unsigned int);
    param[2 + i * 2] = id;
    param[2 + i * 2 + 1] = enabled ? 1 : 0;
  }
  va_end(arg);

  send(BROADCAST_ID, INST_SYNC_WRITE, param, plength);
}


/** Turn LED on or off.
 * @param id servo ID
 * @param led_enabled true to turn LED on, false to turn off
 */
void
DynamixelChain::set_led_enabled(unsigned char id, bool led_enabled )
{
  write_table_value(id, P_LED, led_enabled ? 1 : 0);
}


/** Set compliance values.
 * @param id servo ID
 * @param cw_margin clockwise margin
 * @param cw_slope clockwise slope
 * @param ccw_margin counter-clockwise margin
 * @param ccw_slope counter-clockwise slope
 */
void
DynamixelChain::set_compliance_values(unsigned char id,
				   unsigned char cw_margin, unsigned char cw_slope,
				   unsigned char ccw_margin, unsigned char ccw_slope)
{
  unsigned char param[4];
  param[0] = cw_margin;
  param[1] = ccw_margin;
  param[2] = cw_slope;
  param[3] = ccw_slope;
  write_table_values(id, P_CW_COMPLIANCE_MARGIN, param, 4);
}


/** Set goal speed.
 * @param id servo ID
 * @param goal_speed desired goal speed, 1024 is maximum, 0 means "no velicity
 * control", i.e. move as fast as possible depending on the voltage
 */
void
DynamixelChain::set_goal_speed(unsigned char id, unsigned int goal_speed)
{
  write_table_value(id, P_GOAL_SPEED_L, goal_speed, true);
}


/** Set goal speeds for multiple servos.
 * Given the number of servos the variadic arguments must contain two values
 * for each servo, first is the ID, second the value.
 * @param num_servos number of servos, maximum is 83
 */
void
DynamixelChain::set_goal_speeds(unsigned int num_servos, ...)
{
  if (num_servos > 83) {
    // not enough space for everything in the parameters..
    throw Exception("You cannot set more than 83 speeds at once");
  }

  va_list arg;
  va_start(arg, num_servos);

  unsigned int  plength = 3 * num_servos + 2;
  unsigned char param[plength];
  param[0] = P_GOAL_SPEED_L;
  param[1] = 2;
  for (unsigned int i = 0; i < num_servos; ++i) {
    unsigned char id    = va_arg(arg, unsigned int);
    unsigned int  value = va_arg(arg, unsigned int);
    //printf("Servo speed %u to %u\n", id, value);
    param[2 + i * 3] = id;
    param[2 + i * 3 + 1] = (value & 0xFF);
    param[2 + i * 3 + 2] = (value >> 8) & 0xFF;
  }
  va_end(arg);

  send(BROADCAST_ID, INST_SYNC_WRITE, param, plength);
}


/** Set torque limit.
 * @param id servo ID
 * @param torque_limit new torque limit
 */
void
DynamixelChain::set_torque_limit(unsigned char id, unsigned int torque_limit)
{
  write_table_value(id, P_TORQUE_LIMIT_L, torque_limit, true);
}


/** Set punch.
 * @param id servo ID
 * @param punch new punch value
 */
void
DynamixelChain::set_punch(unsigned char id, unsigned int punch)
{
  write_table_value(id, P_PUNCH_L, punch, true);
}


/** Lock config.
 * Locks the config, configuration values can no longer be modified until the
 * next power cycle.
 * @param id servo ID
 */
void
DynamixelChain::lock_config(unsigned char id)
{
  write_table_value(id, P_LOCK, 1);
}


/** Move servo to specified position.
 * @param id servo ID
 * @param value position, value between 0 and 1023 (inclusive), covering
 * an angle range from 0 to 300 degrees.
 */
void
DynamixelChain::goto_position(unsigned char id, unsigned int value)
{
  write_table_value(id, P_GOAL_POSITION_L, value, true);
}


/** Move several servos to specified positions.
 * Given the number of servos the variadic arguments must contain two values
 * for each servo, first is the ID, second the position (see goto_position() for
 * information on the valid values).
 * @param num_servos number of servos, maximum is 83
 */
void
DynamixelChain::goto_positions(unsigned int num_servos, ...)
{
  if (num_servos > 83) {
    // not enough space for everything in the parameters..
    throw Exception("You cannot set more than 83 servos at once");
  }

  va_list arg;
  va_start(arg, num_servos);

  unsigned int  plength = 3 * num_servos + 2;
  unsigned char param[plength];
  param[0] = P_GOAL_POSITION_L;
  param[1] = 2;
  for (unsigned int i = 0; i < num_servos; ++i) {
    unsigned char id    = va_arg(arg, unsigned int);
    unsigned int  value = va_arg(arg, unsigned int);
    param[2 + i * 3] = id;
    param[2 + i * 3 + 1] = (value & 0xFF);
    param[2 + i * 3 + 2] = (value >> 8) & 0xFF;
  }
  va_end(arg);

  send(BROADCAST_ID, INST_SYNC_WRITE, param, plength);
}

