
/***************************************************************************
 *  roomba_500.cpp - Roomba Open Interface implementation for 500 series
 *
 *  Created: Sat Jan 01 19:37:13 2011
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "roomba_500.h"

#include <core/exceptions/system.h>
#include <core/threading/mutex_locker.h>
#include <netinet/in.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#ifdef HAVE_BLUEZ
#	include <bluetooth/bluetooth.h>
#	include <bluetooth/hci.h>
#	include <bluetooth/hci_lib.h>
#	include <bluetooth/rfcomm.h>
#	include <sys/socket.h>

#	include <fnmatch.h>

static const bdaddr_t _BDADDR_ANY = {{0, 0, 0, 0, 0, 0}};
#endif

using namespace fawkes;

/// @cond SELFEXPLAINING
const unsigned char Roomba500::BUTTON_CLEAN    = 1;
const unsigned char Roomba500::BUTTON_SPOT     = 2;
const unsigned char Roomba500::BUTTON_DOCK     = 4;
const unsigned char Roomba500::BUTTON_MINUTE   = 8;
const unsigned char Roomba500::BUTTON_HOUR     = 16;
const unsigned char Roomba500::BUTTON_DAY      = 32;
const unsigned char Roomba500::BUTTON_SCHEDULE = 64;
const unsigned char Roomba500::BUTTON_CLOCK    = 128;

const unsigned char Roomba500::WHEEL_DROP_LEFT  = 8;
const unsigned char Roomba500::WHEEL_DROP_RIGHT = 4;
const unsigned char Roomba500::BUMP_LEFT        = 2;
const unsigned char Roomba500::BUMP_RIGHT       = 1;

const unsigned char Roomba500::OVERCURRENT_WHEEL_LEFT  = 16;
const unsigned char Roomba500::OVERCURRENT_WHEEL_RIGHT = 8;
const unsigned char Roomba500::OVERCURRENT_MAIN_BRUSH  = 4;
const unsigned char Roomba500::OVERCURRENT_SIDE_BRUSH  = 1;

const unsigned char Roomba500::CHARGING_SOURCE_HOME_BASE = 2;
const unsigned char Roomba500::CHARGING_SOURCE_INTERNAL  = 1;

const unsigned char Roomba500::BUMPER_LEFT         = 1;
const unsigned char Roomba500::BUMPER_FRONT_LEFT   = 2;
const unsigned char Roomba500::BUMPER_CENTER_LEFT  = 4;
const unsigned char Roomba500::BUMPER_CENTER_RIGHT = 8;
const unsigned char Roomba500::BUMPER_FRONT_RIGHT  = 16;
const unsigned char Roomba500::BUMPER_RIGHT        = 32;

const unsigned char Roomba500::LED_DEBRIS      = 1;
const unsigned char Roomba500::LED_SPOT        = 2;
const unsigned char Roomba500::LED_DOCK        = 4;
const unsigned char Roomba500::LED_CHECK_ROBOT = 8;

const unsigned char Roomba500::WEEKDAY_LED_SUN = 1;
const unsigned char Roomba500::WEEKDAY_LED_MON = 2;
const unsigned char Roomba500::WEEKDAY_LED_TUE = 4;
const unsigned char Roomba500::WEEKDAY_LED_WED = 8;
const unsigned char Roomba500::WEEKDAY_LED_THU = 16;
const unsigned char Roomba500::WEEKDAY_LED_FRI = 32;
const unsigned char Roomba500::WEEKDAY_LED_SAT = 64;

const unsigned char Roomba500::SCHEDULING_LED_COLON    = 1;
const unsigned char Roomba500::SCHEDULING_LED_PM       = 2;
const unsigned char Roomba500::SCHEDULING_LED_AM       = 4;
const unsigned char Roomba500::SCHEDULING_LED_CLOCK    = 8;
const unsigned char Roomba500::SCHEDULING_LED_SCHEDULE = 16;

const unsigned char Roomba500::DIGIT_LED_NORTH      = 1;
const unsigned char Roomba500::DIGIT_LED_NORTH_WEST = 32;
const unsigned char Roomba500::DIGIT_LED_NORTH_EAST = 2;
const unsigned char Roomba500::DIGIT_LED_CENTER     = 64;
const unsigned char Roomba500::DIGIT_LED_SOUTH_WEST = 16;
const unsigned char Roomba500::DIGIT_LED_SOUTH_EAST = 4;
const unsigned char Roomba500::DIGIT_LED_SOUTH      = 8;

const unsigned char Roomba500::MOTOR_SIDE_BRUSH            = 1;
const unsigned char Roomba500::MOTOR_VACUUM                = 2;
const unsigned char Roomba500::MOTOR_MAIN_BRUSHES          = 4;
const unsigned char Roomba500::MOTOR_SIDE_BRUSH_BACKWARD   = 8;
const unsigned char Roomba500::MOTOR_MAIN_BRUSHES_BACKWARD = 16;

const unsigned char Roomba500::CHARGER_HOME_BASE = 2;
const unsigned char Roomba500::CHARGER_INTERNAL  = 1;

const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_0                   = 26;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_1                   = 10;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_2                   = 6;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_3                   = 10;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_4                   = 14;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_5                   = 12;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_6                   = 52;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_ALL                 = 80;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_101                 = 28;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_106                 = 12;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_107                 = 9;
const unsigned short int Roomba500::SENSPACK_SIZE_BUMPS_DROPS               = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_WALL                      = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_LEFT                = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_FRONT_LEFT          = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_FRONT_RIGHT         = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_RIGHT               = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_VIRTUAL_WALL              = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_WHEEL_OVERCURRENTS        = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_DIRT_DETECT               = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_IR_CHAR_OMNI              = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_IR_CHAR_LEFT              = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_IR_CHAR_RIGHT             = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_BUTTONS                   = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_DISTANCE                  = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_ANGLE                     = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CHARGING_STATE            = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_VOLTAGE                   = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CURRENT                   = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_TEMPERATURE               = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_BATTERY_CHARGE            = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_BATTERY_CAPACITY          = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_WALL_SIGNAL               = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_LEFT_SIGNAL         = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_FRONT_LEFT_SIGNAL   = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_FRONT_RIGHT_SIGNAL  = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_RIGHT_SIGNAL        = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CHARGE_SOURCES            = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_OI_MODE                   = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_SONG_NUMBER               = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_SONG_PLAYING              = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_STREAM_PACKETS            = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_REQ_VELOCITY              = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_REQ_RADIUS                = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_REQ_RIGHT_VELOCITY        = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_REQ_LEFT_VELOCITY         = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_RIGHT_ENCODER             = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LEFT_ENCODER              = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER              = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_LEFT         = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_FRONT_LEFT   = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_CENTER_LEFT  = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_CENTER_RIGHT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_FRONT_RIGHT  = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_RIGHT        = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LEFT_MOTOR_CURRENT        = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_RIGHT_MOTOR_CURRENT       = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_BRUSH_MOTOR_CURRENT       = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_SIDE_BRUSH_MOTOR_CURRENT  = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_STASIS                    = 1;

const float Roomba500::DIAMETER        = 0.33;
const float Roomba500::BUMPER_X_OFFSET = 0.05;
const float Roomba500::AXLE_LENGTH     = 0.235;

const short int Roomba500::MAX_LIN_VEL_MM_S = 500;
const short int Roomba500::MAX_RADIUS_MM    = 2000;

const unsigned short int Roomba500::MAX_ENCODER_COUNT = 65535;
const short int          Roomba500::MAX_PWM           = 255;

const unsigned short int Roomba500::STREAM_INTERVAL_MS  = 15;
const unsigned short int Roomba500::MODE_CHANGE_WAIT_MS = 20;

const unsigned char Roomba500::CHECKSUM_SIZE = 1;

/// @endcond

/// @cond INTERNALS
#pragma pack(push, 1)

typedef struct
{
	int16_t velocity;
	int16_t radius;
} DriveParams;

typedef struct
{
	int16_t left_velocity;
	int16_t right_velocity;
} DriveDirectParams;

typedef struct
{
	int16_t left_pwm;
	int16_t right_pwm;
} DrivePWMParams;

typedef struct
{
	uint8_t num_packets;
	uint8_t packet_id;
} StreamOnePacketParams;

#pragma pack(pop)
/// @endcond

/** @class Roomba500 "roomba_500.h"
 * Roomba 500 series communication class.
 * This class implements the serial communication with Roomba robots of the
 * 500 series.
 *
 * RFCOMM by reading http://people.csail.mit.edu/albert/bluez-intro/.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param conntype connection type
 * @param device for CONN_SERIAL connection type this is the device file for
 * the serial connection. For CONN_ROOTOOTH this is either a name pattern
 * of a bluetooth device to query or a bluetooth address. The name can be the
 * full name, or a pattern using shell globs (e.g. FireFly-*). The bluetooth
 * address must be given in hexadecimal manner (e.g. 00:11:22:33:44:55).
 * @param flags ConnectionFlags constants, joined with bit-wise "or" (|).
 */
Roomba500::Roomba500(Roomba500::ConnectionType conntype, const char *device, unsigned int flags)
: device_(device)
{
	conntype_   = conntype;
	conn_flags_ = flags;
#ifndef HAVE_BLUEZ
	if (conntype_ == CONNTYPE_ROOTOOTH) {
		throw Exception("Native RooTooth not available at compile time.");
	}
#endif
	mode_            = MODE_OFF;
	fd_              = -1;
	packet_id_       = SENSPACK_GROUP_ALL;
	sensors_enabled_ = false;

	open();
}

/** Destructor. */
Roomba500::~Roomba500()
{
	close();
}

/** Open serial port. */
void
Roomba500::open()
{
	if (conntype_ == CONNTYPE_SERIAL) {
		struct termios param;

		fd_ = ::open(device_.c_str(), O_NOCTTY | O_RDWR);
		if (fd_ == -1) {
			throw CouldNotOpenFileException(device_.c_str(), errno, "Cannot open device file");
		}

		if (tcgetattr(fd_, &param) == -1) {
			Exception e(errno, "Getting the port parameters failed");
			::close(fd_);
			fd_ = -1;
			throw e;
		}

		cfsetospeed(&param, B115200);
		cfsetispeed(&param, B115200);

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

		tcflush(fd_, TCIOFLUSH);

		if (tcsetattr(fd_, TCSANOW, &param) != 0) {
			Exception e(errno, "Setting the port parameters failed");
			::close(fd_);
			fd_ = -1;
			throw e;
		}
	} else {
#ifndef HAVE_BLUEZ
		throw Exception("Native RooTooth support unavailable at compile time.");
#else
		struct hci_dev_info di;
		inquiry_info *      ii = NULL;
		int                 max_rsp, num_rsp;
		int                 dev_id, sock, len, flags;
		char                addrstr[19] = {0};
		char                name[248]   = {0};
		bdaddr_t            baddr       = _BDADDR_ANY;

		if ((dev_id = hci_get_route(NULL)) < 0) {
			throw Exception("RooTooth: local bluetooth device is not available");
		}

		if (hci_devinfo(dev_id, &di) < 0) {
			throw Exception("RooTooth: cannot get local device info.");
		}

		if ((sock = hci_open_dev(dev_id)) < 0) {
			throw Exception("RooTooth: failed to open socket.");
		}

		len     = 8;
		max_rsp = 255;
		flags   = IREQ_CACHE_FLUSH;
		ii      = (inquiry_info *)malloc(max_rsp * sizeof(inquiry_info));

		if (device_.empty()) {
			// we simply guess from the device class

			num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
			if (num_rsp < 0) {
				throw Exception(errno, "HCI inquiry failed");
			}

			for (int i = 0; i < num_rsp; i++) {
				uint8_t b[6];
				baswap((bdaddr_t *)b, &(ii + i)->bdaddr);

				ba2str(&(ii + i)->bdaddr, addrstr);
				/*
	printf("Comparing (0x%2.2x%2.2x%2.2x) (0x%2.2x%2.2x%2.2x) %s %s\n",
	       b[0], b[1], b[2],
	       ii[i].dev_class[0], ii[i].dev_class[1], ii[i].dev_class[2],
	       name, addrstr);
	*/

				// This checks for the RooTooth I have which identifies itself as
				// FireFly-XXXX, where XXXX are the last four digits of the BT addr
				if ( // check OUI for Roving Networks
				  (b[0] == 0x00) && (b[1] == 0x06) && (b[2] == 0x66) &&

				  // check device class
				  (ii[i].dev_class[0] == 0x00) && (ii[i].dev_class[1] == 0x1f)
				  && (ii[i].dev_class[2] == 0x00)) {
					// verify the name
					memset(name, 0, sizeof(name));
					hci_read_remote_name(sock, &(ii + i)->bdaddr, sizeof(name), name, 0);

					if ( // Now check the advertised name
					  (fnmatch("FireFly-*", name, FNM_NOESCAPE) == 0) || (strcmp("RooTooth", name) == 0)) {
						// found a device which is likely a
						ba2str(&(ii + i)->bdaddr, addrstr);
						//printf("found A: %s  %s\n", addrstr, name);
						device_ = addrstr;
						bacpy(&baddr, &(ii + i)->bdaddr);
						break;
					}
				}
			}
		} else {
			bool is_bdaddr = (bachk(device_.c_str()) == 0);

			if (is_bdaddr) {
				//printf("Match by bdaddr\n");

				str2ba(device_.c_str(), &baddr);
				ba2str(&baddr, addrstr);

				//printf("found B: %s  %s\n", addrstr, name);
			} else {
				//printf("Match by btname\n");

				num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
				if (num_rsp < 0) {
					throw Exception(errno, "HCI inquiry failed");
				}

				// we have a name pattern to check
				for (int i = 0; i < num_rsp; i++) {
					ba2str(&(ii + i)->bdaddr, addrstr);
					memset(name, 0, sizeof(name));
					if (hci_read_remote_name(sock, &(ii + i)->bdaddr, sizeof(name), name, 0) < 0) {
						strcpy(name, "[unknown]");
					}
					if (fnmatch(device_.c_str(), name, FNM_NOESCAPE) == 0) {
						// found the device
						//printf("found C: %s  %s\n", addrstr, name);
						device_ = addrstr;
						bacpy(&baddr, &(ii + i)->bdaddr);
						break;
					}
				}
			}
		}

		free(ii);
		::close(sock);

		if (bacmp(&baddr, &_BDADDR_ANY) == 0) {
			throw Exception("Could not find RooTooth device.");
		}
		ba2str(&baddr, addrstr);

		// connect via RFCOMM
		struct sockaddr_rc rcaddr = {0};

		// allocate a socket
		fd_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

		// set the connection parameters (who to connect to)
		rcaddr.rc_family  = AF_BLUETOOTH;
		rcaddr.rc_channel = (uint8_t)1;
		bacpy(&rcaddr.rc_bdaddr, &baddr);

		// connect to server
		if (connect(fd_, (struct sockaddr *)&rcaddr, sizeof(rcaddr)) < 0) {
			throw Exception(errno, "Failed to connect to %s", addrstr);
		}

		try {
			// Set to passive mode to ensure that auto-detection does no harm
			send(OPCODE_START);
			usleep(MODE_CHANGE_WAIT_MS * 1000);
			mode_ = MODE_PASSIVE;
			// disable sensors just in case
			disable_sensors();
		} catch (Exception &e) {
			::close(fd_);
			mode_ = MODE_OFF;
			throw;
		}

		if (flags & FLAG_FIREFLY_FASTMODE) {
			const char *cmd_seq = "$$$";
			if (write(fd_, cmd_seq, 3) != 3) {
				::close(fd_);
				throw Exception(errno,
				                "Roomba500 (RooTooth): Failed to send command "
				                "sequence to enable fast mode");
			}

			timeval timeout = {1, 500000};
			fd_set  read_fds;
			FD_ZERO(&read_fds);
			FD_SET(fd_, &read_fds);

			int rv = 0;
			rv     = select(fd_ + 1, &read_fds, NULL, NULL, &timeout);

			if (rv > 0) {
				char cmd_reply[4];
				if (read(fd_, cmd_reply, 4) == 4) {
					if (strncmp(cmd_reply, "CMD", 3) == 0) {
						// We entered command mode, enable fast mode
						const char *cmd_fastmode = "F,1\r";
						if (write(fd_, cmd_fastmode, 4) != 4) {
							::close(fd_);
							throw Exception(errno,
							                "Roomba500 (RooTooth): Failed to send fast "
							                "mode command sequence.");
						} // else fast mode enabled
					}   // else invalid reply, again assume fast mode
				}     // assume fast mode
			}       // else assume already enabled fast mode
		}         // else do not enable fast mode, assume user knows what he is doing

#endif
	}

	try {
		send(OPCODE_START);
		usleep(MODE_CHANGE_WAIT_MS * 1000);
		mode_ = MODE_PASSIVE;
	} catch (Exception &e) {
		::close(fd_);
		mode_ = MODE_OFF;
		throw;
	}
}

/** Close serial connection. */
void
Roomba500::close()
{
	if (fd_ >= 0) {
		::close(fd_);
		fd_ = -1;
	}
	mode_ = MODE_OFF;
}

/** Send instruction packet.
 * @param opcode Opcode of command
 * @param params parameter bytes of the message
 * @param plength length of the params array
 */
void
Roomba500::send(Roomba500::OpCode opcode, const void *params, const size_t plength)
{
	MutexLocker write_lock(write_mutex_);

	// Byte 0 and 1 must be 0xFF
	obuffer_[0]     = opcode;
	obuffer_length_ = 1;

	if (params && (plength > 0)) {
		if (plength > (sizeof(obuffer_) - obuffer_length_)) {
			throw Exception("Parameters for command %i too long, maximum length is %zu",
			                opcode,
			                (sizeof(obuffer_) - obuffer_length_));
		}
		unsigned char *pbytes = (unsigned char *)params;
		for (size_t i = 0; i < plength; ++i) {
			obuffer_[1 + i] = pbytes[i];
		}
		obuffer_length_ += plength;
	}

	int written = write(fd_, obuffer_, obuffer_length_);

	/*
  printf("Wrote %i of %i bytes:\n", written, obuffer_length_);
  for (int i = 0; i < obuffer_length_; ++i) {
    printf("%2u %s", obuffer_[i], i == written ? "| " : "");
  }
  printf("\n");
  */

	if (written < 0) {
		throw Exception(errno, "Failed to write Roomba 500 packet %i", opcode);
	} else if (written < obuffer_length_) {
		throw Exception("Failed to write Roomba 500 packet %i, "
		                "only %d of %d bytes sent",
		                opcode,
		                written,
		                obuffer_length_);
	}
}

/** Receive a packet.
 * @param index index in ibuffer to fill
 * @param num_bytes number of bytes to read
 * @param timeout_ms maximum wait time in miliseconds, 0 to wait indefinitely.
 */
void
Roomba500::recv(size_t index, size_t num_bytes, unsigned int timeout_ms)
{
	timeval timeout = {0, (suseconds_t)timeout_ms * 1000};

	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(fd_, &read_fds);

	int rv = 0;
	rv     = select(fd_ + 1, &read_fds, NULL, NULL, (timeout_ms > 0) ? &timeout : NULL);

	if (rv == -1) {
		throw Exception(errno, "Roomba500::recv(): select on file descriptor failed");
	} else if (rv == 0) {
		throw TimeoutException("Timeout while waiting for incoming Roomba data");
	}

	ibuffer_length_ = 0;

	// get octets one by one
	int bytes_read = 0;
	while (bytes_read < (int)num_bytes) {
		int rv = read(fd_, &ibuffer_[index] + bytes_read, num_bytes - bytes_read);
		if (rv == -1) {
			throw Exception(errno, "Roomba500::recv(): read failed");
		}
		bytes_read += rv;
	}

	if (bytes_read < (int)num_bytes) {
		throw Exception("Roomba500::recv(): failed to read packet data");
	}

	ibuffer_length_ = index + num_bytes;
}

/** Check if data is available.
 * @return true if data is available and can be read, false otherwise
 */
bool
Roomba500::is_data_available()
{
	if (!sensors_enabled_) {
		throw Exception("Roomba 500 sensors have not been enabled.");
	}

	timeval timeout = {0, 0};

	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(fd_, &read_fds);

	int rv = 0;
	rv     = select(fd_ + 1, &read_fds, NULL, NULL, &timeout);

	return (rv > 0);
}

/** Read sensor values.
 * Enable sensors before using enable_sensors(). This method will block until new
 * data has been read. You can call is_data_available() to check if this method
 * will block or not.
 */
void
Roomba500::read_sensors()
{
	MutexLocker read_lock(read_mutex_);

	if (!sensors_enabled_) {
		throw Exception("Roomba 500 sensors have not been enabled.");
	}

	bool done = false;
	//unsigned int skipped = 0;
	while (!done) {
		ibuffer_length_ = 0;

		recv(ibuffer_length_, 1);
		if (ibuffer_[0] != 19) {
			//++skipped;
			continue;
		}

		recv(ibuffer_length_, 1);
		if (ibuffer_[1] != packet_length_ + 1) {
			//++skipped;
			continue;
		}

		recv(ibuffer_length_, 1);
		if (ibuffer_[2] != packet_id_) {
			//++skipped;
			continue;
		}

		recv(ibuffer_length_, packet_length_);

		recv(ibuffer_length_++, 1);

		unsigned int sum = 0;
		for (int i = 0; i < ibuffer_length_; ++i) {
			sum += ibuffer_[i];
		}

		if ((sum & 0xFF) != 0) {
			sensor_packet_received_ = false;
		} else {
			sensor_mutex_.lock();
			memcpy(&sensor_packet_, &ibuffer_[3], sizeof(SensorPacketGroupAll));
			sensor_packet_received_ = true;
			sensor_mutex_.unlock();
		}

		done = true;
	}

	/*
  printf("Read %u bytes: ", ibuffer_length_);
  for (int i = 0; i < ibuffer_length_; ++i) {
    printf("%2u ", ibuffer_[i]);
  }
  printf(" (skipped %u)\n", skipped);
  */
}

/** Enable sensor data stream.
 * For simplicity and efficiency only the single SENSPACK_GROUP_ALL packet can
 * be streamed at this time. Make sure that the used connection is fast enough.
 * 56700 bit/sec should suffice, but 115200 is strongly recommended. If using
 * RooTooth make sure to use it in fast mode.
 */
void
Roomba500::enable_sensors()
{
	assert_connected();

	StreamOnePacketParams sp;
	sp.num_packets = 1;
	sp.packet_id   = SENSPACK_GROUP_ALL;

	send(OPCODE_STREAM, &sp, sizeof(StreamOnePacketParams));

	packet_id_              = SENSPACK_GROUP_ALL;
	packet_reply_id_        = 19;
	packet_length_          = get_packet_size(SENSPACK_GROUP_ALL);
	sensors_enabled_        = true;
	sensor_packet_received_ = false;
}

/** Disable sensor data stream. */
void
Roomba500::disable_sensors()
{
	assert_connected();

	unsigned char streamstate = STREAM_DISABLE;

	send(OPCODE_PAUSE_RESUME_STREAM, &streamstate, 1);

	sensors_enabled_        = false;
	sensor_packet_received_ = false;
}

/** Query sensor once.
 * For simplicity and efficiency only the single SENSPACK_GROUP_ALL packet can
 * be streamed at this time.
 */
void
Roomba500::query_sensors()
{
	assert_connected();

	unsigned char p = SENSPACK_GROUP_ALL;

	send(OPCODE_QUERY, &p, 1);

	packet_id_              = SENSPACK_GROUP_ALL;
	packet_reply_id_        = 0;
	packet_length_          = get_packet_size(SENSPACK_GROUP_ALL);
	sensor_packet_received_ = true;

	read_mutex_.lock();
	recv(0, packet_length_);
	read_mutex_.unlock();

	sensor_mutex_.lock();
	memcpy(&sensor_packet_, ibuffer_, sizeof(SensorPacketGroupAll));
	sensor_mutex_.unlock();
}

/** Get latest sensor packet.
 * @return sensor packet
 */
const Roomba500::SensorPacketGroupAll
Roomba500::get_sensor_packet() const
{
	MutexLocker lock(sensor_mutex_);
	if (!sensor_packet_received_) {
		throw Exception("No valid data received, yet.");
	}

	return sensor_packet_;
}

/** Set control mode.
 * @param mode the mode can be either MODE_FULL or MODE_SAFE (recommended).
 * MODE_OFF and MODE_PASSIVE cannot be set explicitly. To enter MODE_PASSIVE
 * issue a command which transitions mode, like clean() or seek_dock().
 * @exception Exception thrown if an invalid mode is passed
 */
void
Roomba500::set_mode(Roomba500::Mode mode)
{
	if (mode == MODE_PASSIVE) {
		send(OPCODE_START);
	}
	if (mode == MODE_SAFE) {
		send(OPCODE_SAFE);
	} else if (mode == MODE_FULL) {
		send(OPCODE_FULL);
	} else if (mode == MODE_OFF) {
		throw Exception("Mode OFF cannot be set, use power_down() instead");
	}

	usleep(MODE_CHANGE_WAIT_MS * 1000);
	mode_ = mode;
}

/** Start normal cleaning operation.
 * Transitions to passive mode.
 */
void
Roomba500::clean()
{
	send(OPCODE_CLEAN);

	mode_ = MODE_PASSIVE;
}

/** Start spot cleaning operation.
 * Transitions to passive mode.
 */
void
Roomba500::clean_spot()
{
	send(OPCODE_SPOT);

	mode_ = MODE_PASSIVE;
}

/** Seek for the home base and dock.
 * Transitions to passive mode.
 */
void
Roomba500::seek_dock()
{
	assert_connected();

	send(OPCODE_SEEK_DOCK);
	mode_ = MODE_PASSIVE;
}

/** Powers down the Roomba.
 * Transitions to passive mode.
 */
void
Roomba500::power_down()
{
	assert_connected();

	send(OPCODE_POWER);
	mode_ = MODE_PASSIVE;
}

/** Stop moption of the Roomba.
 * Available only in safe or full mode.
 */
void
Roomba500::stop()
{
	assert_control();
	drive_pwm(0, 0);
}

/** Drive Roomba straight.
 * Available only in safe or full mode.
 * @param velo_mm_per_sec desired velocity in m/sec
 */
void
Roomba500::drive_straight(short int velo_mm_per_sec)
{
	assert_control();

	if (velo_mm_per_sec < -MAX_LIN_VEL_MM_S)
		velo_mm_per_sec = -MAX_LIN_VEL_MM_S;
	if (velo_mm_per_sec > MAX_LIN_VEL_MM_S)
		velo_mm_per_sec = MAX_LIN_VEL_MM_S;

	DriveParams dp;
	dp.velocity = htons(velo_mm_per_sec);
	dp.radius   = htons(0x8000);

	send(OPCODE_DRIVE, &dp, sizeof(DriveParams));
}

/** Turn robot on the spot.
 * Available only in safe or full mode.
 * @param direction turning direction
 */
void
Roomba500::drive_turn(Roomba500::TurnDirection direction)
{
	assert_control();

	DriveParams dp;
	dp.velocity = htons(0);
	dp.radius   = (direction == TURN_CLOCKWISE) ? -1 : 1;

	send(OPCODE_DRIVE, &dp, sizeof(DriveParams));
}

/** Drive Roomba on an arc.
 * Available only in safe or full mode.
 * @param velo_mm_per_sec desired velocity in m/sec
 * @param radius_mm desired radius of arc in m
 */
void
Roomba500::drive_arc(short int velo_mm_per_sec, short int radius_mm)
{
	assert_control();

	if (velo_mm_per_sec < -MAX_LIN_VEL_MM_S)
		velo_mm_per_sec = -MAX_LIN_VEL_MM_S;
	if (velo_mm_per_sec > MAX_LIN_VEL_MM_S)
		velo_mm_per_sec = MAX_LIN_VEL_MM_S;

	if (radius_mm < -MAX_RADIUS_MM)
		radius_mm = -MAX_RADIUS_MM;
	if (radius_mm > MAX_RADIUS_MM)
		radius_mm = MAX_RADIUS_MM;

	DriveParams dp;
	dp.velocity = htons(velo_mm_per_sec);
	dp.radius   = htons(radius_mm);

	send(OPCODE_DRIVE, &dp, sizeof(DriveParams));
}

/** Drive Roomba.
 * Available only in safe or full mode.
 * @param velo_mm_per_sec desired velocity in m/sec
 * @param radius_mm desired radius of arc in m
 */
void
Roomba500::drive(short int velo_mm_per_sec, short int radius_mm)
{
	assert_control();

	if (velo_mm_per_sec < -MAX_LIN_VEL_MM_S)
		velo_mm_per_sec = -MAX_LIN_VEL_MM_S;
	if (velo_mm_per_sec > MAX_LIN_VEL_MM_S)
		velo_mm_per_sec = MAX_LIN_VEL_MM_S;

	if (radius_mm < -MAX_RADIUS_MM)
		radius_mm = -MAX_RADIUS_MM;
	if (radius_mm > MAX_RADIUS_MM)
		radius_mm = 0x8000; // drive straight

	DriveParams dp;
	dp.velocity = htons(velo_mm_per_sec);
	dp.radius   = htons(radius_mm);

	send(OPCODE_DRIVE, &dp, sizeof(DriveParams));
}

/** Directly control wheel velocities.
 * Available only in safe or full mode.
 * @param left_mm_per_sec velocity of left wheel in m/sec
 * @param right_mm_per_sec velocity of right wheel in m/sec
 */
void
Roomba500::drive_direct(short int left_mm_per_sec, short int right_mm_per_sec)
{
	assert_control();

	if (left_mm_per_sec < -MAX_LIN_VEL_MM_S)
		left_mm_per_sec = -MAX_LIN_VEL_MM_S;
	if (left_mm_per_sec > MAX_LIN_VEL_MM_S)
		left_mm_per_sec = MAX_LIN_VEL_MM_S;

	if (right_mm_per_sec < -MAX_LIN_VEL_MM_S)
		right_mm_per_sec = -MAX_LIN_VEL_MM_S;
	if (right_mm_per_sec > MAX_LIN_VEL_MM_S)
		right_mm_per_sec = MAX_LIN_VEL_MM_S;

	DriveDirectParams dp;
	dp.left_velocity  = htons(left_mm_per_sec);
	dp.right_velocity = htons(right_mm_per_sec);

	send(OPCODE_DRIVE, &dp, sizeof(DriveDirectParams));
}

/** Directly control wheel velocities via PWM.
 * Available only in safe or full mode.
 * @param left_wheel_pwm PWM parameter for left wheel
 * @param right_wheel_pwm PWM parameter for right wheel
 */
void
Roomba500::drive_pwm(short int left_wheel_pwm, short int right_wheel_pwm)
{
	assert_control();

	if (left_wheel_pwm < -MAX_PWM)
		left_wheel_pwm = -MAX_PWM;
	if (left_wheel_pwm > MAX_PWM)
		left_wheel_pwm = MAX_PWM;

	if (right_wheel_pwm < -MAX_PWM)
		right_wheel_pwm = -MAX_PWM;
	if (right_wheel_pwm > MAX_PWM)
		right_wheel_pwm = MAX_PWM;

	DrivePWMParams dp;
	dp.left_pwm  = htons(left_wheel_pwm);
	dp.right_pwm = htons(right_wheel_pwm);

	send(OPCODE_DRIVE, &dp, sizeof(DrivePWMParams));
}

/** Set motor states (brushes and vacuum).
 * Available only in safe or full mode.
 * @param main true to enable main brushes
 * @param side true to enable side brush
 * @param vacuum true to enable vacuuming
 * @param main_backward true to enable backward operation of main brushes
 * @param side_backward true to enable backward operation of side brush
 */
void
Roomba500::set_motors(bool main, bool side, bool vacuum, bool main_backward, bool side_backward)
{
	assert_control();

	unsigned char param = 0;
	if (main)
		param |= MOTOR_MAIN_BRUSHES;
	if (side)
		param |= MOTOR_SIDE_BRUSH;
	if (vacuum)
		param |= MOTOR_VACUUM;
	if (main_backward)
		param |= MOTOR_MAIN_BRUSHES_BACKWARD;
	if (side_backward)
		param |= MOTOR_SIDE_BRUSH_BACKWARD;

	send(OPCODE_MOTORS, &param, 1);
}

/** Set LED status of main LEDs.
 * Available only in safe or full mode.
 * @param debris true to enable debris LED, false to disable
 * @param spot true to enable spot LED, false to disable
 * @param dock true to enable dock LED, false to disable
 * @param check_robot true to enable check_robot LED, false to disable
 * @param clean_color color of clean button LED from green (0) to red (255)
 * @param clean_intensity intensity of clean button LED from off (0) to
 * full intensity (255)
 */
void
Roomba500::set_leds(bool          debris,
                    bool          spot,
                    bool          dock,
                    bool          check_robot,
                    unsigned char clean_color,
                    unsigned char clean_intensity)
{
	assert_control();

	unsigned char param[3] = {0, clean_color, clean_intensity};
	if (debris)
		param[0] |= LED_DEBRIS;
	if (spot)
		param[0] |= LED_SPOT;
	if (dock)
		param[0] |= LED_DOCK;
	if (check_robot)
		param[0] |= LED_CHECK_ROBOT;

	send(OPCODE_LEDS, param, 3);
}

/** Set digit LEDs.
 * Available only in safe or full mode.
 * Note, that not all characters are availabe. You can use ASCII table entries
 * 32-39, 44-63, 65-96, and 123-126.
 * @param digits array of digit values
 */
void
Roomba500::set_digit_leds(const char digits[4])
{
	assert_control();

	send(OPCODE_DIGIT_LEDS_ASCII, digits, 4);
}

/** Play a simple fanfare.
 * You can play this for example upon connection to inform the user.
 */
void
Roomba500::play_fanfare()
{
	unsigned char p[14];
	p[0] = 0;
	p[1] = 6;

	// C,E,G,G,E,G
	p[2] = 72;
	p[3] = 6;

	p[4] = 76;
	p[5] = 6;

	p[6] = 79;
	p[7] = 8;

	p[8] = 79;
	p[9] = 10;

	p[10] = 76;
	p[11] = 8;

	p[12] = 79;
	p[13] = 8;

	unsigned char play;
	play = 0;

	send(OPCODE_SONG, p, sizeof(p));
	send(OPCODE_PLAY, &play, 1);
}

/** Get size of packet.
 * @param packet ID of packet to query size for
 * @return size of packet
 * @exception Exception thrown for unknown packet IDs.
 */
unsigned short int
Roomba500::get_packet_size(Roomba500::SensorPacketID packet)
{
	switch (packet) {
	case SENSPACK_BUMPS_DROPS: return SENSPACK_SIZE_BUMPS_DROPS;
	case SENSPACK_WALL: return SENSPACK_SIZE_WALL;
	case SENSPACK_CLIFF_LEFT: return SENSPACK_SIZE_CLIFF_LEFT;
	case SENSPACK_CLIFF_FRONT_LEFT: return SENSPACK_SIZE_CLIFF_FRONT_LEFT;
	case SENSPACK_CLIFF_FRONT_RIGHT: return SENSPACK_SIZE_CLIFF_FRONT_RIGHT;
	case SENSPACK_CLIFF_RIGHT: return SENSPACK_SIZE_CLIFF_RIGHT;
	case SENSPACK_VIRTUAL_WALL: return SENSPACK_SIZE_VIRTUAL_WALL;
	case SENSPACK_WHEEL_OVERCURRENTS: return SENSPACK_SIZE_WHEEL_OVERCURRENTS;
	case SENSPACK_DIRT_DETECT: return SENSPACK_SIZE_DIRT_DETECT;
	case SENSPACK_IR_CHAR_OMNI: return SENSPACK_SIZE_IR_CHAR_OMNI;
	case SENSPACK_BUTTONS: return SENSPACK_SIZE_BUTTONS;
	case SENSPACK_DISTANCE: return SENSPACK_SIZE_DISTANCE;
	case SENSPACK_ANGLE: return SENSPACK_SIZE_ANGLE;
	case SENSPACK_CHARGING_STATE: return SENSPACK_SIZE_CHARGING_STATE;
	case SENSPACK_VOLTAGE: return SENSPACK_SIZE_VOLTAGE;
	case SENSPACK_CURRENT: return SENSPACK_SIZE_CURRENT;
	case SENSPACK_TEMPERATURE: return SENSPACK_SIZE_TEMPERATURE;
	case SENSPACK_BATTERY_CHARGE: return SENSPACK_SIZE_BATTERY_CHARGE;
	case SENSPACK_BATTERY_CAPACITY: return SENSPACK_SIZE_BATTERY_CAPACITY;
	case SENSPACK_WALL_SIGNAL: return SENSPACK_SIZE_WALL_SIGNAL;
	case SENSPACK_CLIFF_LEFT_SIGNAL: return SENSPACK_SIZE_CLIFF_LEFT_SIGNAL;
	case SENSPACK_CLIFF_FRONT_LEFT_SIGNAL: return SENSPACK_SIZE_CLIFF_FRONT_LEFT_SIGNAL;
	case SENSPACK_CLIFF_FRONT_RIGHT_SIGNAL: return SENSPACK_SIZE_CLIFF_FRONT_RIGHT_SIGNAL;
	case SENSPACK_CLIFF_RIGHT_SIGNAL: return SENSPACK_SIZE_CLIFF_RIGHT_SIGNAL;
	case SENSPACK_CHARGE_SOURCES: return SENSPACK_SIZE_CHARGE_SOURCES;
	case SENSPACK_OI_MODE: return SENSPACK_SIZE_OI_MODE;
	case SENSPACK_SONG_NUMBER: return SENSPACK_SIZE_SONG_NUMBER;
	case SENSPACK_SONG_PLAYING: return SENSPACK_SIZE_SONG_PLAYING;
	case SENSPACK_STREAM_PACKETS: return SENSPACK_SIZE_STREAM_PACKETS;
	case SENSPACK_REQ_VELOCITY: return SENSPACK_SIZE_REQ_VELOCITY;
	case SENSPACK_REQ_RADIUS: return SENSPACK_SIZE_REQ_RADIUS;
	case SENSPACK_REQ_RIGHT_VELOCITY: return SENSPACK_SIZE_REQ_RIGHT_VELOCITY;
	case SENSPACK_REQ_LEFT_VELOCITY: return SENSPACK_SIZE_REQ_LEFT_VELOCITY;
	case SENSPACK_RIGHT_ENCODER: return SENSPACK_SIZE_RIGHT_ENCODER;
	case SENSPACK_LEFT_ENCODER: return SENSPACK_SIZE_LEFT_ENCODER;
	case SENSPACK_LIGHT_BUMPER: return SENSPACK_SIZE_LIGHT_BUMPER;
	case SENSPACK_LIGHT_BUMPER_LEFT: return SENSPACK_SIZE_LIGHT_BUMPER_LEFT;
	case SENSPACK_LIGHT_BUMPER_FRONT_LEFT: return SENSPACK_SIZE_LIGHT_BUMPER_FRONT_LEFT;
	case SENSPACK_LIGHT_BUMPER_CENTER_LEFT: return SENSPACK_SIZE_LIGHT_BUMPER_CENTER_LEFT;
	case SENSPACK_LIGHT_BUMPER_CENTER_RIGHT: return SENSPACK_SIZE_LIGHT_BUMPER_CENTER_RIGHT;
	case SENSPACK_LIGHT_BUMPER_FRONT_RIGHT: return SENSPACK_SIZE_LIGHT_BUMPER_FRONT_RIGHT;
	case SENSPACK_LIGHT_BUMPER_RIGHT: return SENSPACK_SIZE_LIGHT_BUMPER_RIGHT;
	case SENSPACK_IR_CHAR_LEFT: return SENSPACK_SIZE_IR_CHAR_LEFT;
	case SENSPACK_IR_CHAR_RIGHT: return SENSPACK_SIZE_IR_CHAR_RIGHT;
	case SENSPACK_LEFT_MOTOR_CURRENT: return SENSPACK_SIZE_LEFT_MOTOR_CURRENT;
	case SENSPACK_RIGHT_MOTOR_CURRENT: return SENSPACK_SIZE_RIGHT_MOTOR_CURRENT;
	case SENSPACK_BRUSH_MOTOR_CURRENT: return SENSPACK_SIZE_BRUSH_MOTOR_CURRENT;
	case SENSPACK_SIDE_BRUSH_MOTOR_CURRENT: return SENSPACK_SIZE_SIDE_BRUSH_MOTOR_CURRENT;
	case SENSPACK_STASIS: return SENSPACK_SIZE_STASIS;
	case SENSPACK_GROUP_0: return SENSPACK_SIZE_GROUP_0;
	case SENSPACK_GROUP_1: return SENSPACK_SIZE_GROUP_1;
	case SENSPACK_GROUP_2: return SENSPACK_SIZE_GROUP_2;
	case SENSPACK_GROUP_3: return SENSPACK_SIZE_GROUP_3;
	case SENSPACK_GROUP_4: return SENSPACK_SIZE_GROUP_4;
	case SENSPACK_GROUP_5: return SENSPACK_SIZE_GROUP_5;
	case SENSPACK_GROUP_6: return SENSPACK_SIZE_GROUP_6;
	case SENSPACK_GROUP_ALL: return SENSPACK_SIZE_GROUP_ALL;
	case SENSPACK_GROUP_101: return SENSPACK_SIZE_GROUP_101;
	case SENSPACK_GROUP_106: return SENSPACK_SIZE_GROUP_106;
	case SENSPACK_GROUP_107: return SENSPACK_SIZE_GROUP_107;
	default: throw Exception("Roomba500:get_packet_size(): unknown packet ID %i", packet);
	}
}
