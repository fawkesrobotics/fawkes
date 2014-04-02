
/***************************************************************************
 *  lase_edl_aqt.cpp - Thread that retrieves the laser data
 *
 *  Created: Wed Oct 08 13:42:32 2008
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

#include "lase_edl_aqt.h"

#include <core/threading/mutex.h>

#include <vector>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>

using namespace fawkes;

const WORD  LaseEdlAcquisitionThread::RESETLEVEL_RESET                = 0x0000;
const WORD  LaseEdlAcquisitionThread::RESETLEVEL_RESTART              = 0x0001;
const WORD  LaseEdlAcquisitionThread::RESETLEVEL_HALT_IDLE            = 0x0002;
const WORD  LaseEdlAcquisitionThread::RESETLEVEL_RELOAD_VOLTSET       = 0x0010;
const WORD  LaseEdlAcquisitionThread::CONFIGITEM_ARCNET_HISTORIC      = 0x0000;
const WORD  LaseEdlAcquisitionThread::CONFIGITEM_RS232_RS422          = 0x0001;
const WORD  LaseEdlAcquisitionThread::CONFIGITEM_CAN                  = 0x0002;
const WORD  LaseEdlAcquisitionThread::CONFIGITEM_SPI                  = 0x0003;
const WORD  LaseEdlAcquisitionThread::CONFIGITEM_ARCNET               = 0x0004;
const WORD  LaseEdlAcquisitionThread::CONFIGITEM_GLOBAL               = 0x0010;
const WORD  LaseEdlAcquisitionThread::CONFIGDATA_LENGTH_RS232_RS422   = 4;
const WORD  LaseEdlAcquisitionThread::CONFIGDATA_LENGTH_CAN           = 5;
const WORD  LaseEdlAcquisitionThread::CONFIGDATA_LENGTH_ARCNET        = 2;
const WORD  LaseEdlAcquisitionThread::CONFIGDATA_LENGTH_GLOBAL        = 3;
const WORD  LaseEdlAcquisitionThread::SECTOR_0                        = 0x0000;
const WORD  LaseEdlAcquisitionThread::SECTOR_1                        = 0x0001;
const WORD  LaseEdlAcquisitionThread::SECTOR_2                        = 0x0002;
const WORD  LaseEdlAcquisitionThread::SECTOR_3                        = 0x0003;
const WORD  LaseEdlAcquisitionThread::SECTOR_4                        = 0x0004;
const WORD  LaseEdlAcquisitionThread::SECTOR_5                        = 0x0005;
const WORD  LaseEdlAcquisitionThread::SECTOR_6                        = 0x0006;
const WORD  LaseEdlAcquisitionThread::SECTOR_7                        = 0x0007;
const WORD  LaseEdlAcquisitionThread::SECTORFUNC_NOT_INITIALIZED      = 0x0000;
const WORD  LaseEdlAcquisitionThread::SECTORFUNC_NO_MEASUREMENT       = 0x0001;
const WORD  LaseEdlAcquisitionThread::SECTORFUNC_DUMMY_MEASUREMENT    = 0x0002;
const WORD  LaseEdlAcquisitionThread::SECTORFUNC_NORMAL_MEASUREMENT   = 0x0003;
const WORD  LaseEdlAcquisitionThread::SECTORFUNC_REFERENCE_TARGET     = 0x0004;
const WORD  LaseEdlAcquisitionThread::FLASH_YES                       = 0x0001;
const WORD  LaseEdlAcquisitionThread::FLASH_NO                        = 0x0000;
const WORD  LaseEdlAcquisitionThread::PROFILENUM_CONTINUOUS           = 0x0000;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_NUMBER            = 0x0001;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_COUNTER           = 0x0002;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_LAYER             = 0x0004;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_SECTOR            = 0x0008;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_ANGLE_STEP        = 0x0010;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_NUM_SECT_POINTS   = 0x0020;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_TIMESTAMP_START   = 0x0040;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_START_DIRECTION   = 0x0080;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_DISTANCE          = 0x0100;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_DIRECTION         = 0x0200;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_ECHO_AMPLITUDE    = 0x0400;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_TIMESTAMP_END     = 0x0800;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_END_DIRECTION     = 0x1000;
const WORD  LaseEdlAcquisitionThread::PROFILEFORMAT_SENSOR_MODE       = 0x2000;

const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_STATUS             = 0x0100;
const WORD  LaseEdlAcquisitionThread::CMD_GET_IDENTIFICATION          = 0x0101;
const WORD  LaseEdlAcquisitionThread::CMD_GET_STATUS                  = 0x0102;
const WORD  LaseEdlAcquisitionThread::CMD_GET_ERROR                   = 0x0103;
const WORD  LaseEdlAcquisitionThread::CMD_GET_SIGNAL                  = 0x0104;
const WORD  LaseEdlAcquisitionThread::CMD_SET_SIGNAL                  = 0x0105;
const WORD  LaseEdlAcquisitionThread::CMD_REGISTER_APPLICATION        = 0x0106;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_CONFIG             = 0x0200;
const WORD  LaseEdlAcquisitionThread::CMD_SET_CONFIG                  = 0x0201;
const WORD  LaseEdlAcquisitionThread::CMD_GET_CONFIG                  = 0x0202;
const WORD  LaseEdlAcquisitionThread::CMD_SET_SYNC_ABS                = 0x0203;
const WORD  LaseEdlAcquisitionThread::CMD_SET_SYNC_REL                = 0x0204;
const WORD  LaseEdlAcquisitionThread::CMD_SET_SYNC_CLOCK              = 0x0205;
const WORD  LaseEdlAcquisitionThread::CMD_SET_ZONE                    = 0x0206;
const WORD  LaseEdlAcquisitionThread::CMD_GET_ZONE                    = 0x0207;
const WORD  LaseEdlAcquisitionThread::CMD_RELEASE_ZONE                = 0x0208;
const WORD  LaseEdlAcquisitionThread::CMD_SET_FILTER                  = 0x0209;
const WORD  LaseEdlAcquisitionThread::CMD_SET_FUNCTION                = 0x020A;
const WORD  LaseEdlAcquisitionThread::CMD_GET_FUNCTION                = 0x020B;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_MEASUREMENT        = 0x0300;
const WORD  LaseEdlAcquisitionThread::CMD_GET_PROFILE                 = 0x0301;
const WORD  LaseEdlAcquisitionThread::CMD_CANCEL_PROFILE              = 0x0302;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_WORKING            = 0x0400;
const WORD  LaseEdlAcquisitionThread::CMD_DO_RESET                    = 0x0401;
const WORD  LaseEdlAcquisitionThread::CMD_TRANS_IDLE                  = 0x0402;
const WORD  LaseEdlAcquisitionThread::CMD_TRANS_ROTATE                = 0x0403;
const WORD  LaseEdlAcquisitionThread::CMD_TRANS_MEASURE               = 0x0404;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_MAINTENANCE        = 0x0500;
const WORD  LaseEdlAcquisitionThread::CMD_DO_ADJUST                   = 0x0501;
const WORD  LaseEdlAcquisitionThread::CMD_DO_TEST                     = 0x0502;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_INTERFACE_ROUTING  = 0x0600;
const WORD  LaseEdlAcquisitionThread::CMD_COM_ATTACH                  = 0x0601;
const WORD  LaseEdlAcquisitionThread::CMD_COM_DETACH                  = 0x0602;
const WORD  LaseEdlAcquisitionThread::CMD_COM_INIT                    = 0x0603;
const WORD  LaseEdlAcquisitionThread::CMD_COM_OUTPUT                  = 0x0604;
const WORD  LaseEdlAcquisitionThread::CMD_COM_DATA                    = 0x0605;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_FILE               = 0x0700;
const WORD  LaseEdlAcquisitionThread::CMD_DIR                         = 0x0701;
const WORD  LaseEdlAcquisitionThread::CMD_SAVE                        = 0x0702;
const WORD  LaseEdlAcquisitionThread::CMD_LOAD                        = 0x0703;
const WORD  LaseEdlAcquisitionThread::CMD_DELETE                      = 0x0704;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_MONITOR            = 0x0900;
const WORD  LaseEdlAcquisitionThread::CMD_MONITOR_ENABLE_LOG          = 0x0801;
const WORD  LaseEdlAcquisitionThread::CMD_MONITOR_DISABLE_LOG         = 0x0802;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_ADJUST             = 0x7E00;
const WORD  LaseEdlAcquisitionThread::SERVICEGROUP_SPECIAL            = 0x7F00;
const WORD  LaseEdlAcquisitionThread::CMD_SERVICE_FAILURE             = 0x7F00;
const WORD  LaseEdlAcquisitionThread::RESPONSE_BIT                    = 0x8000;


const float LaseEdlAcquisitionThread::DISTANCE_FACTOR                 = 256.00;


/** @class LaseEdlAcquisitionThread "lase_edl_aqt.h"
 * Laser acqusition thread for Lase EDL L A laser scanner.
 * This thread fetches the data from the laser.
 * @author Tim Niemueller
 * @author Christian Fritz
 */


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 */
LaseEdlAcquisitionThread::LaseEdlAcquisitionThread(std::string &cfg_name,
						   std::string &cfg_prefix)
  : LaserAcquisitionThread("LaseEdlAcquisitionThread")
{
  set_name("LaseEDL(%s)", cfg_name.c_str());
  __pre_init_done = false;
  __cfg_name   = cfg_name;
  __cfg_prefix = cfg_prefix;
}


void
LaseEdlAcquisitionThread::pre_init(fawkes::Configuration *config,
				   fawkes::Logger        *logger)
{
  if (__pre_init_done)  return;

  try {
    std::string canres  = config->get_string((__cfg_prefix + "canonical_resolution").c_str());
    if (canres == "low") {
      __cfg_rotation_freq = 20;
      __cfg_angle_step    = 16;
    } else if (canres == "high") {
      __cfg_rotation_freq = 15;
      __cfg_angle_step    =  8;
    } else {
      logger->log_error(name(), "Canonical resolution %s is invalid, must be 'low' "
			"or 'high', trying to read raw config data");
      throw Exception("");
    }
    logger->log_debug(name(), "Using canonical resolution %s, freq: %u, angle step: %u",
		      canres.c_str(), __cfg_rotation_freq, __cfg_angle_step);
  } catch (Exception &e) {
    // exceptions thrown here will propagate
    __cfg_rotation_freq  = config->get_uint((__cfg_prefix + "rotation_freq").c_str());
    __cfg_angle_step     = config->get_uint((__cfg_prefix + "angle_step").c_str());
  }

  try {
    __cfg_use_default    = config->get_bool((__cfg_prefix + "use_default").c_str());
    __cfg_set_default    = config->get_bool((__cfg_prefix + "set_default").c_str());
    __cfg_max_pulse_freq = config->get_uint((__cfg_prefix + "max_pulse_freq").c_str());
    __cfg_profile_format = config->get_uint((__cfg_prefix + "profile_format").c_str());
    __cfg_can_id         = config->get_uint((__cfg_prefix + "can_id").c_str());
    __cfg_can_id_resp    = config->get_uint((__cfg_prefix + "can_id_resp").c_str());
    __cfg_sensor_id      = config->get_uint((__cfg_prefix + "sensor_id").c_str());
    __cfg_sensor_id_resp = config->get_uint((__cfg_prefix + "sensor_id_resp").c_str());
    __cfg_btr0btr1       = config->get_uint((__cfg_prefix + "btr0btr1").c_str());
    __cfg_port           = config->get_uint((__cfg_prefix + "port").c_str());
    __cfg_irq            = config->get_uint((__cfg_prefix + "irq").c_str());
    __cfg_num_init_tries = config->get_uint((__cfg_prefix + "num_init_tries").c_str());
    __cfg_mount_rotation = config->get_float((__cfg_prefix + "mount_rotation").c_str());

    __min_angle_step     = calc_angle_step(__cfg_rotation_freq, __cfg_max_pulse_freq);
    if ( __cfg_angle_step < __min_angle_step ) {
      logger->log_warn(name(), "Configured angle step %u less than required minimum "
		       "of %u, raising to minimum", __cfg_angle_step, __min_angle_step);
      __cfg_angle_step = __min_angle_step;
    }
    __number_of_values = 16 * 360 / __cfg_angle_step;

    if ( (__number_of_values != 360) && (__number_of_values != 720) ) {
      throw Exception("At the moment only configurations with 360 or 720 "
		      "laser beams are supported, but %u requested", __number_of_values);
    }

    _distances_size = _echoes_size = __number_of_values;

    std::string interface_type = config->get_string((__cfg_prefix + "interface_type").c_str());
    if ( interface_type == "usb" ) {
      __cfg_interface_type = HW_USB;
    } else {
      throw Exception("Unknown interface type %s", interface_type.c_str());
    }

  } catch (Exception &e) {
    e.append("Could not read all required config values for %s", name());
    throw;
  }

  __pre_init_done = true;
}

void
LaseEdlAcquisitionThread::init()
{
  pre_init(config, logger);

  init_bus();

  for (unsigned int i = 1; i <= __cfg_num_init_tries; ++i) {

    try {
      CANCEL_PROFILE();
    } catch (Exception &e) {
      // ignored, happens often
    }

    try {
      logger->log_debug("LaseEdlAcquisitionThread", "Resetting Laser");
      DO_RESET(RESETLEVEL_HALT_IDLE);

      if ( ! __cfg_use_default ) {
	logger->log_debug("LaseEdlAcquisitionThread", "Setting configuration");
	// set configuration (rotation and anglestep)
	SET_CONFIG(CONFIGITEM_GLOBAL, CONFIGDATA_LENGTH_GLOBAL,
		   __cfg_sensor_id, __cfg_rotation_freq, __cfg_angle_step);

	// set functions (sector definition)
	SET_FUNCTION(SECTOR_0, SECTORFUNC_NORMAL_MEASUREMENT,
		     (16 * 360) - __cfg_angle_step,
		     __cfg_set_default ? FLASH_YES : FLASH_NO);
	SET_FUNCTION(SECTOR_1, SECTORFUNC_NOT_INITIALIZED, 0,
		     __cfg_set_default ? FLASH_YES : FLASH_NO);
      }

      logger->log_debug("LaseEdlAcquisitionThread", "Starting rotating");
      TRANS_ROTATE(__cfg_rotation_freq);
      logger->log_debug("LaseEdlAcquisitionThread", "Starting measuring");
      TRANS_MEASURE();
      logger->log_debug("LaseEdlAcquisitionThread", "Enable profile retrieval");
      GET_PROFILE(PROFILENUM_CONTINUOUS, __cfg_profile_format);

      break; // break for loop if initialization was successful
    } catch (Exception &e) {
      if (i < __cfg_num_init_tries) {
        logger->log_warn("LaseEdlAcquisitionThread", "Initialization, retrying %d more times", __cfg_num_init_tries - i);
        logger->log_warn("LaseEdlAcquisitionThread", e);
      } else {
        logger->log_error("LaseEdlAcquisitionThread", "Initialization failed, giving up after %u tries", __cfg_num_init_tries);
        throw;
      }
    }
  }

  _distances  = (float *)malloc(sizeof(float) * __number_of_values);
  _echoes     = (float *)malloc(sizeof(float) * __number_of_values);
}


void
LaseEdlAcquisitionThread::finalize()
{
  free(_distances);
  free(_echoes);
  _distances = _echoes = NULL;

  logger->log_debug("LaseEdlAcquisitionThread", "Resetting laser");
  DO_RESET(RESETLEVEL_HALT_IDLE);
}


void
LaseEdlAcquisitionThread::loop()
{
  process_profiles();
}


unsigned int
LaseEdlAcquisitionThread::calc_angle_step(unsigned int rotation_freq,
					  unsigned int max_pulse_freq)
{
  float tmp;
  unsigned int rv;
  tmp = ( ((float)max_pulse_freq) / 360.0 ) / ((float)rotation_freq);
  tmp = ceil( (1 / tmp) * 16.0 );
  rv  = (unsigned int)tmp;
    
  if (rv == 7 || rv == 11 || rv == 13 || rv == 14)  rv++;

  return rv;
}


void
LaseEdlAcquisitionThread::init_bus()
{
  FILE *f = fopen("/proc/pcan", "r");
  if (! f) {
    throw Exception("Cannot open /proc/pcan, PCAN driver not loaded?");
  }
  std::vector<std::string> config_lines;
  std::vector<std::string> device_lines;
  char tmp[128];
  while (fgets(tmp, sizeof(tmp), f)) {
    if (tmp[0] == '*') {
      config_lines.push_back(tmp);
    } else if (tmp[0] != '\n') {
      device_lines.push_back(tmp);
    }
  }
  fclose(f);

  std::vector<std::string>::iterator l;
  for (l = config_lines.begin(); l != config_lines.end(); ++l) {
    // proc is found
    std::string::size_type pos = 0;
    while ((pos = l->find("[", pos)) != std::string::npos) {
      pos += 1;
      std::string::size_type pos_end = l->find("]", pos);
      if (pos_end != std::string::npos) {
	std::string item = l->substr(pos, pos_end - pos);
	if (item == "net") {
	  throw Exception("PCAN driver has been compiled in netdev mode, but "
			  "chardev mode is required. Please read the plugin "
			  "documentation and recompile the PCAN driver.");
	}
      }
    }
  }

  __handle = CAN_Open(__cfg_interface_type, 0, __cfg_port, __cfg_irq);
  if (__handle == NULL) {
    throw Exception("Cannot open CAN bus");
  }
  if (CAN_Init(__handle, __cfg_btr0btr1, CAN_INIT_TYPE_ST) != CAN_ERR_OK) {
    throw Exception("Cannot initialize CAN bus");
  }
}


void
LaseEdlAcquisitionThread::send(WORD *data, int n)
{
  TPCANMsg msg;
  msg.ID      = __cfg_can_id;
  msg.MSGTYPE = MSGTYPE_STANDARD;
  msg.LEN     = 0;

  int send_words = 0;
  WORD number_of_frames = 0;

  // special case for less or equal two words 
  if (n <= 2) {
    number_of_frames = 1;
    append_to_msg( (WORD)0, &msg);
    append_to_msg( (WORD)__cfg_sensor_id, &msg);
    if (n >= 1) {
      append_to_msg( data[0], &msg);
    }
    if (n == 2) {
      append_to_msg( data[1], &msg);
    }
    //printf("send (1): "); print_message(&msg);
    if (CAN_Write( __handle, &msg ) != CAN_ERR_OK) {
      throw Exception("Laser send() failed (1)");
    }

  } else { // more than 2 words
    number_of_frames = ((n - 1) / 3) + 1;
    if ((n-1) % 3 != 0) {
      ++number_of_frames;
    }
    append_to_msg( (WORD)0xFFFF, &msg);
    append_to_msg( number_of_frames, &msg);
    append_to_msg( (WORD)__cfg_sensor_id, &msg);
    append_to_msg( data[send_words++], &msg);
    // printf("send (2): "); print_message(&msg);
    if (CAN_Write( __handle, &msg ) != CAN_ERR_OK) {
      throw Exception("Laser send() failed (2)");
    }

    for (WORD f=number_of_frames-1; f > 1; --f ) {
      msg.LEN = 0;
      append_to_msg( f, &msg);
      append_to_msg( data[send_words++], &msg);
      append_to_msg( data[send_words++], &msg);
      append_to_msg( data[send_words++], &msg);
      // printf("send (3): "); print_message(&msg);
      if (CAN_Write( __handle, &msg ) != CAN_ERR_OK) {
	throw Exception("Laser send() failed (3)");
      }
    }
    // last frame
    msg.LEN = 0;
    append_to_msg( (WORD)0x0001, &msg);
    for (int i=send_words; i < n; i++) {
      append_to_msg( data[send_words++], &msg);
    }
    // printf("send (4): "); print_message(&msg);
    if (CAN_Write( __handle, &msg ) != CAN_ERR_OK) {
      throw Exception("Laser send() failed (3)");
    }
  }
}


int
LaseEdlAcquisitionThread::recv(WORD **data, bool allocate)
{
  TPCANMsg msg;
  // read from CAN BUS
  if (CAN_Read( __handle, &msg) != CAN_ERR_OK) {
    throw Exception("Laser recv() failed (1)");
  }
  // If msg wasn't send by our laser: ignore it
  if (msg.ID != __cfg_can_id_resp) {
    logger->log_warn("LaseEdlAcquisitionThread", "CAN ID is not the expected ID, "
		     "ignoring message");
    return -1;
  }

  int  number_of_incoming_frames = 0;
  WORD number_of_incoming_words = 0;
  int  msg_index = 0;
  int  data_index = 0;
  WORD read;

  read = get_word_from_msg(&msg, &msg_index);

  // seek for beginning of a block
  while ((read != 0x0000) && (read != 0xFFFF) ) {
    if (CAN_Read( __handle, &msg) != CAN_ERR_OK) {
      throw Exception("Laser recv() failed (2)");
    }
    msg_index = 0;
    read = get_word_from_msg( &msg, &msg_index);
  }

  // got legal block: process it
  if (read == 0x0000) { // receiving only one frame
    read = get_word_from_msg( &msg, &msg_index);
    if (read != __cfg_sensor_id_resp) {
      logger->log_warn("LaseEdlAcquisitionThread", "Sensor ID is not the expected ID, "
		       "ignoring message");
      return -1;
    }
    number_of_incoming_words = (msg.LEN - msg_index) / 2;
    if (allocate) {
      (*data) = (WORD*)malloc( sizeof(WORD)* (number_of_incoming_words));
    }
    for (int i=0; i < number_of_incoming_words; ++i) {
      (*data)[i] = get_word_from_msg( &msg, &msg_index);
    }
    // printf("Received (1): "); print_word_array(number_of_incoming_words, *data);
    return number_of_incoming_words;
  } else if (read == 0xFFFF) {
    // get number of incoming frames
    number_of_incoming_frames = get_word_from_msg( &msg, &msg_index);
    if (allocate) {
      (*data) = (WORD*)malloc( sizeof(WORD)* (number_of_incoming_frames * 6 + 1));
    }
    data_index = 0;

    // get sensor response ID
    read = get_word_from_msg( &msg, &msg_index);
    if (read != __cfg_sensor_id_resp) {
      logger->log_warn("LaseEdlAcquisitionThread", "Sensor ID is not the expected ID, "
		       "ignoring message");
      return -1;
    }

    // two words remaining in first message
    (*data)[data_index++] = get_word_from_msg( &msg, &msg_index);

    // process all frames
    for (WORD f=number_of_incoming_frames-1; f > 0; --f ) {
      msg_index = 0;

      if (CAN_Read( __handle, &msg) != CAN_ERR_OK) {
	throw Exception("Laser recv() failed (3)");
      }

      // get and verify frame number indicator
      read = get_word_from_msg( &msg, &msg_index);
      if (read != f) {
	logger->log_warn("LaseEdlAcquisitionThread","Recv protocol violation, "
			 "wrong frame number: expected %u, but got %u", f, read);
	return -1;
      }

      // process all words in frame
      number_of_incoming_words = (msg.LEN - msg_index) >> 1;
      for (int i=0; i < number_of_incoming_words; ++i) {
	(*data)[data_index++] = get_word_from_msg( &msg, &msg_index);
      }
    }

    // printf("Received (2): "); print_word_array(data_index, *data);

    // might be different from number_of_incoming_words,
    // since last message can be not full
    return data_index;
    
  } else {
    logger->log_warn("LaseEdlAcquisitionThread", "Recv got strange first response word (neigther 0 nor FFFF)\n");
  } 
  return -1;
}


inline void
LaseEdlAcquisitionThread::append_to_msg(WORD word, TPCANMsg *msg)
{
  BYTE byte;
  byte = word >> 8;
  msg->DATA[(msg->LEN)++] = byte;
  byte = word;
  msg->DATA[(msg->LEN)++] = byte;
}


inline void
LaseEdlAcquisitionThread::append_to_msg(BYTE byte, TPCANMsg *msg)
{
  msg->DATA[(msg->LEN)++] = byte;
}

inline WORD
LaseEdlAcquisitionThread::get_word_from_msg(TPCANMsg *msg, int *index)
{
  WORD rv  = msg->DATA[(*index)++] << 8;
  rv += msg->DATA[((*index)++)];
  return rv;
}


WORD *
LaseEdlAcquisitionThread::make_word_array(int count, ...) {
  va_list word_list;
  va_start(word_list, count);
  WORD *rtv;
  rtv = (WORD*)malloc( sizeof(WORD) * count);
  for (int i=0; i<count; ++i) {
    rtv[i] = (WORD) va_arg(word_list, int);
  }
  va_end(word_list);
  return rtv;
}


int
LaseEdlAcquisitionThread::compare_word_arrays(int count, WORD* a, WORD* b)
{
  for (int i=0; i < count; ++i) {
    if (a[i] != b[i]) {
      return 0;
    }
  }
  return 1;
}


void
LaseEdlAcquisitionThread::print_word_array(int count, WORD* a)
{
  for (int i=0; i < count; ++i) {
    printf("%04x ", a[i]);
  }
  printf("\n");
}


void
LaseEdlAcquisitionThread::print_message(TPCANMsg *m)
{
  int i;
  printf("%c %c 0x%08x %1d  ", 
	 (m->MSGTYPE & MSGTYPE_RTR)      ? 'r' : 'm',
	 (m->MSGTYPE & MSGTYPE_EXTENDED) ? 'e' : 's',
	 m->ID, 
	 m->LEN); 
  
  for (i = 0; i < m->LEN; i++) {
    printf("0x%02x ", m->DATA[i]);
  }
    
  printf("\n");
}

void
LaseEdlAcquisitionThread::process_profiles()
{
  WORD* real_response;
  WORD* expected_response = make_word_array( 2, respcode(CMD_GET_PROFILE),
					     __cfg_profile_format);
  int response_size = recv(&real_response);
  if (response_size == -1) {
    logger->log_warn("LaseEdlAcquisitionThread", "process_profiles(): recv() failed");
    return;
  }

  // wrong answer ?
  if (! compare_word_arrays( 2, real_response, expected_response )) {
    logger->log_warn("LaseEdlAcquisitionThread", "process_profiles(): Invalid response received");
    return;
  }
  // wrong number of values ?
  if ( (response_size - 3 != (int)__number_of_values) &&
       (response_size - 3 != 2 * (int)__number_of_values) ) {
    logger->log_warn("LaseEdlAcquisitionThread", "number of received values "
		     "doesn't match my expectations, recvd %d, expected %d",
		     response_size - 3, __number_of_values);
    return;
  }

  // extract data from response
  register float dist = 0;
  register int echo = 0;
  register int dist_index = (int)roundf(__cfg_mount_rotation * 16 / __cfg_angle_step);
  register int echo_index = dist_index;

  _data_mutex->lock();
  _new_data = true;
  _timestamp->stamp();

  // see which data is requested
  if (__cfg_profile_format == PROFILEFORMAT_DISTANCE ) {
    // only distances
    for (int i=3; i < response_size; ++i ) {
      dist = ((float)real_response[i]) / DISTANCE_FACTOR;
      _distances[__number_of_values - dist_index] = dist;
      if (++dist_index >= (int)__number_of_values) dist_index = 0;
    }

  } else if (__cfg_profile_format == (PROFILEFORMAT_DISTANCE | PROFILEFORMAT_ECHO_AMPLITUDE) ) {
    // distances + echos
    for (int i=3; i < response_size; ++i) {
      dist = ((float)real_response[i]) / DISTANCE_FACTOR;
      _distances[__number_of_values - dist_index] = dist;
      if (++dist_index >= (int)__number_of_values) dist_index = 0;
      ++i;
      echo = real_response[i];
      _echoes[__number_of_values - echo_index] = echo;
      if (++echo_index >= (int)__number_of_values) echo_index = 0;
    }


  } else if (__cfg_profile_format == PROFILEFORMAT_ECHO_AMPLITUDE ) {
    // only echos
    for (int i=3; i < response_size; ++i ) {
      echo = real_response[i];
      _echoes[__number_of_values - echo_index] = echo;
      if (++echo_index >= (int)__number_of_values) echo_index = 0;
    }
  }

  _data_mutex->unlock();

  free( real_response );
  free( expected_response );
}


void
LaseEdlAcquisitionThread::send_and_check(WORD *command_data, int command_length,
				       WORD *expected_response, int n,
				       WORD **real_response, int *response_size)
{
  bool keep_response = (real_response != NULL);
  WORD **response;
  WORD *local_response;
  if (keep_response) {
    response = real_response;
  } else {
    response = &local_response;
  }
  send(command_data, command_length);
  int  response_s = recv(response);

  if (response_s <= 0) {
    throw Exception("Did not receive data for command");
  }

  bool match = compare_word_arrays(n, *response, expected_response);

  if ( ! match || ! keep_response ) {
    free(*response);
  }
  free(expected_response);
  free(command_data);

  if ( ! match) {
    throw Exception("Response to query did not match expectation");
  }

  if ( response_size != NULL ) {
    *response_size = response_s;
  }
}

void
LaseEdlAcquisitionThread::SET_CONFIG( WORD config_item, int k, ...)
{
  WORD *command;
  command = (WORD*)malloc( sizeof(WORD) * (2+k) );
  command[0] = CMD_SET_CONFIG;
  command[1] = config_item;
  va_list word_list;
  va_start( word_list, k);
  for (int i=0; i<k; ++i) {
    command[i+2] = (WORD) va_arg( word_list, int);
  }
  va_end( word_list );

  send_and_check(command, 2+k, make_word_array(2, respcode(CMD_SET_CONFIG), 0x0000), 2);
}


void
LaseEdlAcquisitionThread::SET_FUNCTION(WORD sect_num, WORD sect_func,
				       WORD sect_stop, WORD flash )
{
  WORD* command = make_word_array(5, CMD_SET_FUNCTION, sect_num, sect_func,
				  sect_stop, flash);
  send_and_check(command, 5, make_word_array(2, respcode(CMD_SET_FUNCTION), sect_num), 2);
}


void
LaseEdlAcquisitionThread::GET_PROFILE( WORD prof_num, WORD prof_format)
{
  WORD* command = make_word_array(3, CMD_GET_PROFILE, prof_num, prof_format);
  send_and_check(command, 3,
		 make_word_array(2, respcode(CMD_GET_PROFILE), prof_format), 2);
}


void
LaseEdlAcquisitionThread::CANCEL_PROFILE()
{
  send_and_check(make_word_array(1, CMD_CANCEL_PROFILE), 1,
		 make_word_array( 1, respcode(CMD_CANCEL_PROFILE)), 1);
}


void
LaseEdlAcquisitionThread::DO_RESET(WORD reset_level)
{
  WORD* command = make_word_array( 2, CMD_DO_RESET, reset_level);
  send_and_check(command, 2, make_word_array(2, respcode(CMD_DO_RESET), reset_level), 2);
}


void
LaseEdlAcquisitionThread::TRANS_IDLE()
{
  WORD* command = make_word_array( 1, CMD_TRANS_IDLE);
  WORD* real_response;
  int   response_size;

  send_and_check(command, 1, make_word_array( 1, respcode(CMD_TRANS_IDLE)), 1, &real_response, &response_size);

  bool failed = (real_response[response_size-1] != 0x0001);
  free(real_response);
  if (failed) throw Exception("Failed to set trans idle");
}


void
LaseEdlAcquisitionThread::TRANS_ROTATE(WORD frequency)
{
  WORD* command = make_word_array( 2, CMD_TRANS_ROTATE, frequency);
  WORD* real_response;
  int   response_size;
  send_and_check(command, 2, make_word_array( 1, respcode(CMD_TRANS_ROTATE)), 1,
		 &real_response, &response_size);
	     
  bool failed = (real_response[response_size-1] != 0x0002);
  free(real_response);
  if ( failed )  throw Exception("Failed to set trans rotate");
}


void
LaseEdlAcquisitionThread::TRANS_MEASURE()
{
  WORD* command = make_word_array( 1, CMD_TRANS_MEASURE);
  WORD* real_response;
  int   response_size;  
  send_and_check(command, 1, make_word_array( 1, respcode(CMD_TRANS_MEASURE)),
		 1, &real_response, &response_size);

  bool failed = (real_response[response_size-2] != 0x0003) ||
                (real_response[response_size-1] != 0x0000);
  unsigned int error_code = real_response[response_size-1];
  free(real_response);
  if ( failed )  throw Exception("Failed set trans measure, error code %u", error_code);
}
