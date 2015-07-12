
/***************************************************************************
 *  servo_chain.h - Class for accessing Robotis dynamixel servos
 *
 *  Created: Mon Mar 23 20:42:42 2015 (based on pantilt plugin)
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

#ifndef __PLUGINS_DYNAMIXEL_SERVO_CHAIN_H_
#define __PLUGINS_DYNAMIXEL_SERVO_CHAIN_H_

#include <core/exception.h>

#include <list>
#include <vector>
#include <cstddef>
#include <cstdio>

#define DYNAMIXEL_CONTROL_TABLE_LENGTH 0x32
#define DYNAMIXEL_MAX_NUM_SERVOS       254

class DynamixelChain
{
 public:
  /** List of servo IDs. */
  typedef std::list<unsigned char> DeviceList;

  DynamixelChain(const char *device_file, unsigned int default_timeout_ms = 30, bool enable_echo_fix = false, bool enable_connection_stability = false, float min_voltage = 12.0, float max_voltage = 16.0);
  ~DynamixelChain();

  void         open();
  void         close();

  bool         ping(unsigned char id, unsigned int timeout_ms = 100);
  DeviceList   discover(unsigned int total_timeout_ms = 50, const std::vector<unsigned int> servos = std::vector<unsigned int>());


  void write_table_value(unsigned char id, unsigned char addr,
			 unsigned int value, bool double_byte = false);
  void write_table_values(unsigned char id, unsigned char start_addr,
			  unsigned char *values, unsigned int num_values);
  void read_table_values(unsigned char id);
  void read_table_value(unsigned char id,
			unsigned char addr, unsigned char read_length);
  void          start_read_table_values(unsigned char id);
  void          finish_read_table_values();

  void          goto_position(unsigned char id, unsigned int value);
  void          goto_positions(unsigned int num_positions, ...);

  const char *  get_model(unsigned char id, bool refresh = false);
  unsigned int  get_model_number(unsigned char id, bool refresh = false);
  unsigned int  get_position(unsigned char id, bool refresh = false);
  unsigned char get_firmware_version(unsigned char id, bool refresh = false);
  unsigned char get_baudrate(unsigned char id, bool refresh = false);
  unsigned char get_delay_time(unsigned char id, bool refresh = false);
  unsigned char get_error(unsigned char id);
  void          get_angle_limits(unsigned char id,
				 unsigned int &cw_limit, unsigned int &ccw_limit,
				 bool refresh = false);
  unsigned char get_temperature_limit(unsigned char id, bool refresh = false);
  void          get_voltage_limits(unsigned char id,
				   unsigned char &low, unsigned char &high,
				   bool refresh = false);
  unsigned int  get_max_torque(unsigned char id, bool refresh = false);
  unsigned char get_status_return_level(unsigned char id, bool refresh = false);
  unsigned char get_alarm_led(unsigned char id, bool refresh = false);
  unsigned char get_alarm_shutdown(unsigned char id, bool refresh = false);
  void          get_calibration(unsigned char id,
				unsigned int &down_calib, unsigned int &up_calib,
				bool refresh = false);
  bool          is_torque_enabled(unsigned char id, bool refresh = false);
  bool          is_led_enabled(unsigned char id, bool refresh = false);
  void          get_compliance_values(unsigned char id,
				      unsigned char &cw_margin, unsigned char &cw_slope,
				      unsigned char &ccw_margin, unsigned char &ccw_slope,
				      bool refresh = false);
  unsigned int  get_goal_position(unsigned char id, bool refresh = false);
  unsigned int  get_goal_speed(unsigned char id, bool refresh = false);
  float         get_max_supported_speed(unsigned char id, bool refresh = false);
  unsigned int  get_torque_limit(unsigned char id, bool refresh = false);
  unsigned int  get_speed(unsigned char id, bool refresh = false);
  unsigned int  get_load(unsigned char id, bool refresh = false);
  unsigned char get_voltage(unsigned char id, bool refresh = false);
  unsigned char get_temperature(unsigned char id, bool refresh = false);
  bool          is_moving(unsigned char id, bool refresh = false);
  bool          is_locked(unsigned char id, bool refresh = false);
  unsigned int  get_punch(unsigned char id, bool refresh = false);

  void          set_id(unsigned char id, unsigned char new_id);
  void          set_baudrate(unsigned char id, unsigned char baudrate);
  void          set_return_delay_time(unsigned char id, unsigned char return_delay_time);
  void          set_angle_limits(unsigned char id,
				 unsigned int cw_limit, unsigned int ccw_limit);
  void          set_temperature_limit(unsigned char id, unsigned char temp_limit);
  void          set_voltage_limits(unsigned char id, unsigned char low, unsigned char high);
  void          set_max_torque(unsigned char id, unsigned int max_torque);
  void          set_status_return_level(unsigned char id, unsigned char status_return_level);
  void          set_alarm_led(unsigned char id, unsigned char alarm_led);
  void          set_alarm_shutdown(unsigned char id, unsigned char alarm_shutdown);
  void          set_torque_enabled(unsigned char id, bool enabled);
  void          set_torques_enabled(bool enabled, unsigned char num_servos, ...);
  void          set_led_enabled(unsigned char id, bool enabled);
  void          set_compliance_values(unsigned char id,
				      unsigned char cw_margin, unsigned char cw_slope,
				      unsigned char ccw_margin, unsigned char ccw_slope);
  void          set_goal_speed(unsigned char id, unsigned int goal_speed);
  void          set_goal_speeds(unsigned int num_servos, ...);
  void          set_torque_limit(unsigned char id, unsigned int torque_limit);
  void          lock_config(unsigned char id);
  void          set_punch(unsigned char id, unsigned int punch);

  bool          data_available();

  // Status return level
  static const unsigned char SRL_RESPOND_NONE;
  static const unsigned char SRL_RESPOND_READ;
  static const unsigned char SRL_RESPOND_ALL;

  static const unsigned char BROADCAST_ID;
  static const unsigned int  MAX_POSITION;
  static const unsigned int  CENTER_POSITION;
  static const float         MAX_ANGLE_DEG;
  static const float         MAX_ANGLE_RAD;
  static const float         RAD_PER_POS_TICK;
  static const float         POS_TICKS_PER_RAD;
  static const float         SEC_PER_60DEG_12V;
  static const float         SEC_PER_60DEG_16V;
  static const unsigned int  MAX_SPEED;

  // Parameter entry offsets
  static const unsigned char P_MODEL_NUMBER_L;
  static const unsigned char P_MODEL_NUMBER_H;
  static const unsigned char P_VERSION;
  static const unsigned char P_ID;
  static const unsigned char P_BAUD_RATE;
  static const unsigned char P_RETURN_DELAY_TIME;
  static const unsigned char P_CW_ANGLE_LIMIT_L;
  static const unsigned char P_CW_ANGLE_LIMIT_H;
  static const unsigned char P_CCW_ANGLE_LIMIT_L;
  static const unsigned char P_CCW_ANGLE_LIMIT_H;
  static const unsigned char P_SYSTEM_DATA2;
  static const unsigned char P_LIMIT_TEMPERATURE;
  static const unsigned char P_DOWN_LIMIT_VOLTAGE;
  static const unsigned char P_UP_LIMIT_VOLTAGE;
  static const unsigned char P_MAX_TORQUE_L;
  static const unsigned char P_MAX_TORQUE_H;
  static const unsigned char P_RETURN_LEVEL;
  static const unsigned char P_ALARM_LED;
  static const unsigned char P_ALARM_SHUTDOWN;
  static const unsigned char P_OPERATING_MODE;
  static const unsigned char P_DOWN_CALIBRATION_L;
  static const unsigned char P_DOWN_CALIBRATION_H;
  static const unsigned char P_UP_CALIBRATION_L;
  static const unsigned char P_UP_CALIBRATION_H;

  static const unsigned char P_TORQUE_ENABLE;
  static const unsigned char P_LED;
  static const unsigned char P_CW_COMPLIANCE_MARGIN;
  static const unsigned char P_CCW_COMPLIANCE_MARGIN;
  static const unsigned char P_CW_COMPLIANCE_SLOPE;
  static const unsigned char P_CCW_COMPLIANCE_SLOPE;
  static const unsigned char P_GOAL_POSITION_L;
  static const unsigned char P_GOAL_POSITION_H;
  static const unsigned char P_GOAL_SPEED_L;
  static const unsigned char P_GOAL_SPEED_H;
  static const unsigned char P_TORQUE_LIMIT_L;
  static const unsigned char P_TORQUE_LIMIT_H;
  static const unsigned char P_PRESENT_POSITION_L;
  static const unsigned char P_PRESENT_POSITION_H;
  static const unsigned char P_PRESENT_SPEED_L;
  static const unsigned char P_PRESENT_SPEED_H;
  static const unsigned char P_PRESENT_LOAD_L;
  static const unsigned char P_PRESENT_LOAD_H;
  static const unsigned char P_PRESENT_VOLTAGE;
  static const unsigned char P_PRESENT_TEMPERATURE;
  static const unsigned char P_REGISTERED_INSTRUCTION;
  static const unsigned char P_PAUSE_TIME;
  static const unsigned char P_MOVING;
  static const unsigned char P_LOCK;
  static const unsigned char P_PUNCH_L;
  static const unsigned char P_PUNCH_H;

 private:
  // Instructions
  static const unsigned char INST_PING;
  static const unsigned char INST_READ;
  static const unsigned char INST_WRITE;
  static const unsigned char INST_REG_WRITE;
  static const unsigned char INST_ACTION;
  static const unsigned char INST_RESET;
  static const unsigned char INST_DIGITAL_RESET;
  static const unsigned char INST_SYSTEM_READ;
  static const unsigned char INST_SYSTEM_WRITE;
  static const unsigned char INST_SYNC_WRITE;
  static const unsigned char INST_SYNC_REG_WRITE;

  // Packet offsets
  static const unsigned char PACKET_OFFSET_ID;
  static const unsigned char PACKET_OFFSET_LENGTH;
  static const unsigned char PACKET_OFFSET_INST;
  static const unsigned char PACKET_OFFSET_PARAM;
  static const unsigned char PACKET_OFFSET_ERROR;

 private:
  unsigned char calc_checksum(const unsigned char id, const unsigned char instruction,
			      const unsigned char *params, const unsigned char plength);
  void send(const unsigned char id, const unsigned char instruction,
	    const unsigned char *params, const unsigned char plength);
  void recv(const unsigned char exp_length, unsigned int timeout_ms = 0xFFFFFFFF);
  void assert_valid_id(unsigned char id);
  unsigned int merge_twobyte_value(unsigned int id,
				   unsigned char ind_l, unsigned char ind_h);
  unsigned int get_value(unsigned int id, bool refresh,
			 unsigned int ind_l, unsigned int ind_h = 0xFFFFFFFF);
  bool inline responds_read(unsigned int id)
  {
    
    return ((__control_table[id][P_RETURN_LEVEL] == SRL_RESPOND_READ) ||
	    (__control_table[id][P_RETURN_LEVEL] == SRL_RESPOND_ALL));
  }

  bool inline responds_all(unsigned int id)
  {
    return (__control_table[id][P_RETURN_LEVEL] == SRL_RESPOND_ALL);
  }

  int           __fd;
  char         *__device_file;
  unsigned int  __default_timeout_ms;
  bool          __enable_echo_fix;
  bool          __enable_connection_stability;
  float         __min_voltage;
  float         __max_voltage;

  unsigned char __obuffer[260];
  unsigned char __ibuffer[260];

  int           __obuffer_length;
  int           __ibuffer_length;

  char          __control_table[DYNAMIXEL_MAX_NUM_SERVOS][DYNAMIXEL_CONTROL_TABLE_LENGTH];

};



#endif
