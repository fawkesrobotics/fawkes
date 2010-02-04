
/***************************************************************************
 * lase_edl_aqt.cpp - Thread to retrieves laser data from Lase LD A OEM
 *
 *  Created: Wed Oct 08 13:41:02 2008
 *  Copyright  2002       Christian Fritz
 *             2008-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef HAVE_LIBPCAN
#  error "Cannot use Lase EDL driver without libpcan"
#endif

#ifndef __PLUGINS_LASER_LASE_EDL_AQT_H_
#define __PLUGINS_LASER_LASE_EDL_AQT_H_

#include "acquisition_thread.h"

#include <libpcan.h>
#include <string>

class LaseEdlAcquisitionThread : public LaserAcquisitionThread
{
 public:
  LaseEdlAcquisitionThread(std::string &cfg_name, std::string &cfg_prefix);

  // from LaserAcquisitionThread
  virtual void pre_init(fawkes::Configuration *config, fawkes::Logger *logger);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  static const WORD  RESETLEVEL_RESET;
  static const WORD  RESETLEVEL_RESTART;
  static const WORD  RESETLEVEL_HALT_IDLE;
  static const WORD  RESETLEVEL_RELOAD_VOLTSET;
  static const WORD  CONFIGITEM_ARCNET_HISTORIC;
  static const WORD  CONFIGITEM_RS232_RS422;
  static const WORD  CONFIGITEM_CAN;
  static const WORD  CONFIGITEM_SPI;
  static const WORD  CONFIGITEM_ARCNET;
  static const WORD  CONFIGITEM_GLOBAL;
  static const WORD  CONFIGDATA_LENGTH_RS232_RS422;
  static const WORD  CONFIGDATA_LENGTH_CAN;
  static const WORD  CONFIGDATA_LENGTH_ARCNET;
  static const WORD  CONFIGDATA_LENGTH_GLOBAL;
  static const WORD  SECTOR_0;
  static const WORD  SECTOR_1;
  static const WORD  SECTOR_2;
  static const WORD  SECTOR_3;
  static const WORD  SECTOR_4;
  static const WORD  SECTOR_5;
  static const WORD  SECTOR_6;
  static const WORD  SECTOR_7;
  static const WORD  SECTORFUNC_NOT_INITIALIZED;
  static const WORD  SECTORFUNC_NO_MEASUREMENT;
  static const WORD  SECTORFUNC_DUMMY_MEASUREMENT;
  static const WORD  SECTORFUNC_NORMAL_MEASUREMENT;
  static const WORD  SECTORFUNC_REFERENCE_TARGET;
  static const WORD  FLASH_YES;
  static const WORD  FLASH_NO;
  static const WORD  PROFILENUM_CONTINUOUS;
  static const WORD  PROFILEFORMAT_NUMBER;
  static const WORD  PROFILEFORMAT_COUNTER;
  static const WORD  PROFILEFORMAT_LAYER;
  static const WORD  PROFILEFORMAT_SECTOR;
  static const WORD  PROFILEFORMAT_ANGLE_STEP;
  static const WORD  PROFILEFORMAT_NUM_SECT_POINTS;
  static const WORD  PROFILEFORMAT_TIMESTAMP_START;
  static const WORD  PROFILEFORMAT_START_DIRECTION;
  static const WORD  PROFILEFORMAT_DISTANCE;
  static const WORD  PROFILEFORMAT_DIRECTION;
  static const WORD  PROFILEFORMAT_ECHO_AMPLITUDE;
  static const WORD  PROFILEFORMAT_TIMESTAMP_END;
  static const WORD  PROFILEFORMAT_END_DIRECTION;
  static const WORD  PROFILEFORMAT_SENSOR_MODE;
  static const WORD  SERVICEGROUP_STATUS;
  static const WORD  CMD_GET_IDENTIFICATION;
  static const WORD  CMD_GET_STATUS;
  static const WORD  CMD_GET_ERROR;
  static const WORD  CMD_GET_SIGNAL;
  static const WORD  CMD_SET_SIGNAL;
  static const WORD  CMD_REGISTER_APPLICATION;
  static const WORD  SERVICEGROUP_CONFIG;
  static const WORD  CMD_SET_CONFIG;
  static const WORD  CMD_GET_CONFIG;
  static const WORD  CMD_SET_SYNC_ABS;
  static const WORD  CMD_SET_SYNC_REL;
  static const WORD  CMD_SET_SYNC_CLOCK;
  static const WORD  CMD_SET_ZONE;
  static const WORD  CMD_GET_ZONE;
  static const WORD  CMD_RELEASE_ZONE;
  static const WORD  CMD_SET_FILTER;
  static const WORD  CMD_SET_FUNCTION;
  static const WORD  CMD_GET_FUNCTION;
  static const WORD  SERVICEGROUP_MEASUREMENT;
  static const WORD  CMD_GET_PROFILE;
  static const WORD  CMD_CANCEL_PROFILE;
  static const WORD  SERVICEGROUP_WORKING;
  static const WORD  CMD_DO_RESET;
  static const WORD  CMD_TRANS_IDLE;
  static const WORD  CMD_TRANS_ROTATE;
  static const WORD  CMD_TRANS_MEASURE;
  static const WORD  SERVICEGROUP_MAINTENANCE;
  static const WORD  CMD_DO_ADJUST;
  static const WORD  CMD_DO_TEST;
  static const WORD  SERVICEGROUP_INTERFACE_ROUTING;
  static const WORD  CMD_COM_ATTACH;
  static const WORD  CMD_COM_DETACH;
  static const WORD  CMD_COM_INIT;
  static const WORD  CMD_COM_OUTPUT;
  static const WORD  CMD_COM_DATA;
  static const WORD  SERVICEGROUP_FILE;
  static const WORD  CMD_DIR;
  static const WORD  CMD_SAVE;
  static const WORD  CMD_LOAD;
  static const WORD  CMD_DELETE;
  static const WORD  SERVICEGROUP_MONITOR;
  static const WORD  CMD_MONITOR_ENABLE_LOG;
  static const WORD  CMD_MONITOR_DISABLE_LOG;
  //static const WORD  SERVICEGROUP_APPLICATION; 0x1000 to 0x3F00
  static const WORD  SERVICEGROUP_ADJUST;
  static const WORD  SERVICEGROUP_SPECIAL;
  static const WORD  CMD_SERVICE_FAILURE;
  static const WORD  RESPONSE_BIT;
  static const float DISTANCE_FACTOR;

 private:
  unsigned int calc_angle_step(unsigned int rotation_freq, unsigned int max_pulse_freq);
  inline WORD respcode(WORD cmd) { return cmd | RESPONSE_BIT; }
  void init_bus();
  void send(WORD *data, int n);
  int  recv(WORD **data, bool allocate = true);

  void send_and_check(WORD *command_data, int command_length,
		      WORD *expected_response, int n,
		      WORD **real_response = NULL, int *response_size = NULL);

  inline void append_to_msg(WORD word, TPCANMsg *msg);
  inline void append_to_msg(BYTE byte, TPCANMsg *msg);
  inline WORD get_word_from_msg(TPCANMsg *msg, int *index);
  WORD * make_word_array(int count, ...);
  int compare_word_arrays(int count, WORD* a, WORD* b);
  void print_word_array(int count, WORD* a);
  void print_message(TPCANMsg *m);

  void process_profiles();

  void SET_CONFIG(WORD config_item, int k, ...);
  void SET_FUNCTION(WORD sect_num, WORD sect_func, WORD sect_stop, WORD flash);
  void GET_PROFILE(WORD prof_num, WORD prof_format);
  void CANCEL_PROFILE();
  void DO_RESET(WORD reset_level);
  void TRANS_IDLE();
  void TRANS_ROTATE(WORD frequency);
  void TRANS_MEASURE();



 private:
  HANDLE __handle;
  bool         __pre_init_done;

  std::string  __cfg_name;
  std::string  __cfg_prefix;

  bool         __cfg_use_default;
  bool         __cfg_set_default;
  unsigned int __cfg_rotation_freq;
  unsigned int __cfg_angle_step;
  unsigned int __cfg_max_pulse_freq;
  unsigned int __cfg_profile_format;
  unsigned int __cfg_can_id;
  unsigned int __cfg_can_id_resp;
  unsigned int __cfg_sensor_id;
  unsigned int __cfg_sensor_id_resp;
  unsigned int __cfg_interface_type;
  unsigned int __cfg_btr0btr1;
  unsigned int __cfg_port;
  unsigned int __cfg_irq;
  unsigned int __cfg_num_init_tries;
  float        __cfg_mount_rotation;

  unsigned int __min_angle_step;
  unsigned int __number_of_values;
  
};


#endif
