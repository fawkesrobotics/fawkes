/***************************************************************************
 *  direct_com_message.h - Message for RobotinoDirectThread
 *
 *  Created: Mon Apr 04 15:40:32 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROBOTINO_DIRECT_COM_MESSAGE_H_
#define __PLUGINS_ROBOTINO_DIRECT_COM_MESSAGE_H_

#include <core/exception.h>

#include <cstdint>
#include <boost/asio.hpp>

class DirectRobotinoComMessage
{
 public:
	/// shared pointer to direct com message
	typedef std::shared_ptr<DirectRobotinoComMessage> pointer;

	/// @cond INTERNAL
	typedef enum {
		CMDID_NONE  = 0,
		CMDID_GET_HW_VERSION						=	1,
		CMDID_HW_VERSION							=	2,
		CMDID_GET_SW_VERSION						=	3,
		CMDID_SW_VERSION							=	4,
		CMDID_GET_DISTANCE_SENSOR_READINGS	=	5,
		CMDID_DISTANCE_SENSOR_READINGS			=	6,
		CMDID_GET_ADC_PARAMETERS            = 7,
		CMDID_ADC_PARAMETERS						=	8,
		CMDID_SET_MOTOR_SPEED						=	9,
		CMDID_GET_ALL_MOTOR_SPEEDS				=	10,
		CMDID_ALL_MOTOR_SPEEDS					=	11,
		CMDID_SET_MOTOR_POSITION					=	12,
		CMDID_GET_ALL_MOTOR_POSITIONS				=	13,
		CMDID_ALL_MOTOR_POSITIONS					=	14,
		CMDID_SET_MOTOR_PID_PARAMETERS			=	15,
		CMDID_GET_ALL_MOTOR_PID_PARAMETERS		=	16,
		CMDID_ALL_MOTOR_PID_PARAMETERS			=	17,
		CMDID_SET_ALL_DIGITAL_OUTPUTS				=	18,
		CMDID_SET_ALL_RELAYS						=	19,
		CMDID_SET_ODOMETRY						=	20,
		CMDID_SET_ODOMETRY_ROTATION				=	21,
		CMDID_GET_ODOMETRY						=	22,
		CMDID_ODOMETRY							=	23,
		CMDID_GET_ALL_MOTOR_CURRENT_READINGS		=	26,
		CMDID_ALL_MOTOR_CURRENT_READINGS			=	27,
		CMDID_GET_ALL_ANALOG_INPUTS				=	32,
		CMDID_ALL_ANALOG_INPUTS					=	33,
		CMDID_GET_ALL_DIGITAL_INPUTS				=	34,
		CMDID_ALL_DIGITAL_INPUTS					=	35,
		CMDID_GET_BUMPER							=	36,
		CMDID_BUMPER								=	37,
		CMDID_GET_POWER_BUTTON					=	38,
		CMDID_POWER_BUTTON						=	39,
		CMDID_SET_FPGA_POWER						=	40,
		CMDID_GET_FPGA_POWER						=	41,
		CMDID_FPGA_POWER							=	42,
		CMDID_GET_PWR_OK_STATE					=	43,
		CMDID_PWR_OK_STATE						=	44,
		CMDID_SET_PWR_OK_STATE					=	45,
		CMDID_SET_PWM								=	46,
		CMDID_SET_MOTOR_ON						=	47,
		CMDID_SET_PWRBTN							=	48,
		CMDID_SET_SYS_RESET						=	49,
		CMDID_GET_COM_EXPRESS_STATES				=	50,
		CMDID_COM_EXPRESS_STATES					=	51,
		CMDID_GET_ALL_MOTOR_READINGS				=	52,
		CMDID_ALL_MOTOR_READINGS					=	53,
		CMDID_GET_IP_ADDRESS						=	54,
		CMDID_IP_ADDRESS							=	55,
		CMDID_SET_IP_ADDRESS						=	56,
		CMDID_SET_EMERGENCY_BUMPER				=	57,
		CMDID_SET_MOTOR_MODE						=	58,
		CMDID_RESET_LPC							=	59,
		CMDID_POWER_OFF							=	60,
		CMDID_SET_SAFTEY_VELOCITY					=	61,	
		CMDID_GET_SAFTEY_VELOCITY					=	62,	
		CMDID_SAFTEY_VELOCITY						=	63,	
		CMDID_POWER_SOURCE_READINGS				=	65,
		CMDID_SET_MOTOR_ACCEL_LIMITS				=	66,
		CMDID_MOTOR_ACCEL_LIMITS					=	67,
		CMDID_GET_MOTOR_ACCEL_LIMITS				=	68,
		CMDID_GET_GYRO_Z_ANGLE					=	69,
		CMDID_GYRO_Z_ANGLE						=	70,
		CMDID_GET_CAN_MSG							=	71,
		CMDID_CAN_MSG								=	72,
		CMDID_SET_NRST							=	73,
		CMDID_GET_NRST							=	74,
		CMDID_NRST								=	75,
		CMDID_SET_BOOT							=	76,
		CMDID_GET_BOOT							=	77,
		CMDID_BOOT								=	78,
		CMDID_CONFIG_RESET						=	79,
		CMDID_CHARGER_INFO						=	80,
		CMDID_CHARGER_EVENT						=	81,
		CMDID_CHARGER_VERSION						=	82,
		CMDID_CHARGER_GET_VERSION					=	83,
		CMDID_CHARGER_CLEAR_ERROR					=	84,
		CMDID_CHARGER_ERROR						=	85,
		CMDID_SET_BATTERY_MIN						=	86,
		CMDID_GET_BATTERY_MIN						=	87,
		CMDID_BATTERY_MIN							=	88,
		CMDID_GET_GYRODATA						=	89,
		CMDID_GYRODATA							=	90,
		CMDID_GET_GPAIN							= 91,
		CMDID_GPAIN									= 92,
		CMDID_GET_VERSIONBITS				= 93,
		CMDID_VERSIONBITS						= 94,
		CMDID_GYRO_SET_PARAM				= 95,
		CMDID_GYRO_GET_PARAM				= 96,
		CMDID_GYRO_PARAM						= 97,
		CMDID_INFO								=	250,
		CMDID_WARNING								=	251,
		CMDID_ERROR								=	252,
		CMDID_LINGO_A								=	253,
		CMDID_LINGO_B								=	254,
		CMDID_LINGO_C								=	255
	} command_id_t;

	typedef enum {
		READ,
		WRITE
	} mode_t;

	static const unsigned char MSG_HEAD;
	static const unsigned char MSG_DATA_ESCAPE;
	static const unsigned char MSG_DATA_MANGLE;
	static const unsigned int  MSG_METADATA_SIZE;
	/// @endcond INTERNAL

	class ChecksumError : public fawkes::Exception
	{
	public:
		ChecksumError(unsigned int expected, unsigned int received,
		              unsigned char byte1, unsigned char byte2);
	};

	DirectRobotinoComMessage();
	DirectRobotinoComMessage(command_id_t cmdid);
	DirectRobotinoComMessage(const unsigned char *msg, size_t msg_size);
	virtual ~DirectRobotinoComMessage();

	void add_command(command_id_t cmdid);
	void add_int8(int8_t value);
	void add_uint8(uint8_t value);
	void add_int16(int16_t value);
	void add_uint16(uint16_t value);
	void add_int32(int32_t value);
	void add_uint32(uint32_t value);
	void add_float(float value);

	void rewind();
	command_id_t  next_command();
	uint8_t       command_length() const;
	command_id_t  command_id() const;
	int8_t        get_int8();
	uint8_t       get_uint8();
	int16_t       get_int16();
	uint16_t      get_uint16();
	int32_t       get_int32();
	uint32_t      get_uint32();
	float         get_float();
	std::string   get_string();

	void          skip_int8();
	void          skip_uint8();
	void          skip_int16();
	void          skip_uint16();
	void          skip_int32();
	void          skip_uint32();
	void          skip_float();

	boost::asio::const_buffer buffer();
	
	void     pack();
	uint16_t checksum() const;

	std::string to_string(bool escaped = false);

	static size_t unescape(unsigned char *unescaped, size_t unescaped_size,
	                       const unsigned char *escaped, size_t escaped_size);

	static uint16_t parse_uint16(const unsigned char *buf);

	size_t escaped_data_size();
	size_t payload_size();
	size_t data_size();
	
 private:
	void ctor();
	void assert_mode(mode_t mode) const;
	void assert_command() const;
	void assert_command_data(uint8_t size) const;

	void inc_payload_by(uint16_t count);
	void escape();
	size_t unescape_data();
	void check_checksum() const;
  
 private:
	mode_t mode_;

	unsigned char *data_;
	unsigned short data_size_;
	unsigned short payload_size_;
	unsigned char *escaped_data_;
	unsigned short escaped_data_size_;

	unsigned char *cur_cmd_;
	unsigned char *cur_data_;

};


#endif

