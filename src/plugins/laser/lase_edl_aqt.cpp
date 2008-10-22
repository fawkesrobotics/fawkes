
/***************************************************************************
 *  acqusition_thread.cpp - Thread that retrieves the laser data
 *
 *  Created: Wed Oct 08 13:42:32 2008
 *  Copyright  2002  Christian Fritz
 *             2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <cstdlib>
#include <cmath>
#include <string>

using namespace fawkes;

/** @class LaseEdlAcquisitionThread "playerc_thread.h"
 * Laser acqusition thread for Lase EDL L A laser scanner.
 * This thread fetches the data from the laser.
 * @author Tim Niemueller
 * @author Christian Fritz
 */


/** Constructor. */
LaseEdlAcquisitionThread::LaseEdlAcquisitionThread()
  : LaserAcquisitionThread("LaseEdlAcquisitionThread")
{
}


void
LaseEdlAcquisitionThread::init()
{
  try {
    __cfg_use_default    = config->get_bool("/laser/use_default");
    __cfg_set_default    = config->get_bool("/laser/set_default");
    __cfg_rotation_freq  = config->get_uint("/laser/rotation_freq");
    __cfg_angle_step     = config->get_uint("/laser/angle_step");
    __cfg_max_pulse_freq = config->get_uint("/laser/max_pulse_freq");
    __cfg_profile_format = config->get_uint("/laser/profile_format");
    __cfg_can_id         = config->get_uint("/laser/can_id");
    __cfg_can_id_resp    = config->get_uint("/laser/can_id_resp");
    __cfg_sensor_id      = config->get_uint("/laser/sensor_id");
    __cfg_sensor_id_resp = config->get_uint("/laser/sensor_id_resp");
    __cfg_btr0btr1       = config->get_uint("/laser/btr0btr1");
    __cfg_port           = config->get_uint("/laser/port");
    __cfg_irq            = config->get_uint("/laser/irq");

    __min_angle_step     = calc_angle_step(__cfg_rotation_freq, __cfg_max_pulse_freq);
    if ( __cfg_angle_step < __min_angle_step )  __cfg_angle_step = __min_angle_step;
    __number_of_values = 16 * 360 / __cfg_angle_step;

    if ( __number_of_values != 360 ) {
      throw Exception("At the moment only configurations with 360 laser beams are supported");
    }

    _distances_size = _echoes_size = __number_of_values;

    std::string interface_type = config->get_string("/laser/interface_type");
    if ( interface_type == "usb" ) {
      __cfg_interface_type = HW_USB;
    } else {
      throw Exception("Unknown interface type %s", interface_type.c_str());
    }

  } catch (Exception &e) {
    e.append("Could not read all required config values for %s", name());
    throw;
  }

  init_bus();

  try {
    CANCEL_PROFILE();
  } catch (Exception &e) {
    // ignored, happens often
  }

  logger->log_debug("LaseEdlAcquisitionThread", "Resetting Laser");
  DO_RESET(0x0002);
  if ( ! __cfg_use_default ) {
    logger->log_debug("LaseEdlAcquisitionThread", "Setting configuration");
    // set configuration (rotation and anglestep)
    SET_CONFIG( 0x0010, 3, __cfg_sensor_id, __cfg_rotation_freq, __cfg_angle_step);
    
    // set functions (sector definition)
    SET_FUNCTION( 0x0000, 0x0003, (16 * 360) - __cfg_angle_step, __cfg_set_default);
    SET_FUNCTION( 0x0001, 0x0000, 0, __cfg_set_default);
  }

  logger->log_debug("LaseEdlAcquisitionThread", "Starting rotating");
  TRANS_ROTATE(__cfg_rotation_freq);
  logger->log_debug("LaseEdlAcquisitionThread", "Starting measuring");
  TRANS_MEASURE();
  logger->log_debug("LaseEdlAcquisitionThread", "Enable profile retrieval");
  GET_PROFILE(0, __cfg_profile_format);

  _distances  = (float *)malloc(sizeof(float) * __number_of_values);
  _echoes     = (float *)malloc(sizeof(float) * __number_of_values);
}


void
LaseEdlAcquisitionThread::finalize()
{
  free(_distances);
  free(_echoes);
  _distances = _echoes = NULL;

  //TRANS_IDLE();
  logger->log_debug("LaseEdlAcquisitionThread", "Resetting laser");
  DO_RESET(0x0002);
}


void
LaseEdlAcquisitionThread::loop()
{
  process_profiles();
}


unsigned int
LaseEdlAcquisitionThread::calc_angle_step(unsigned int rotation_freq, unsigned int max_pulse_freq)
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
  WORD* expected_response = make_word_array( 2, 0x8301, __cfg_profile_format);
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
  register int dist_index = __number_of_values / 2;
  register int echo_index = __number_of_values / 2;

  _data_mutex->lock();
  _new_data = true;

  // see which data is requested
  if (__cfg_profile_format == 0x0100) { // only distances
    for (int i=3; i < response_size; i++ ) {
      dist = ((float)real_response[i]) / 256.0;
      _distances[dist_index++] = dist;
      if (dist_index >= (int)__number_of_values) dist_index = 0;
    }
  } else if (__cfg_profile_format == 0x0500 ) { // distances + echos
    for (int i=3; i < response_size; ) {
      dist = ((float)real_response[i]) / 256.0;
      _distances[dist_index++] = dist;
      if (dist_index >= (int)__number_of_values) dist_index = 0;
      i++;
      echo = real_response[i];
      _echoes[echo_index++] = echo;
      if (echo_index >= (int)__number_of_values) echo_index = 0;
      i++;
    }
  } else if (__cfg_profile_format == 0x0400 ) { // only echos
    for (int i=3; i < response_size; i++ ) {
      echo = real_response[i];
      _echoes[echo_index++] = echo;
      if (echo_index >= (int)__number_of_values) echo_index = 0;
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
LaseEdlAcquisitionThread::SET_CONFIG( WORD CONFIGITEM, int k, ...)
{
  WORD *command;
  command = (WORD*)malloc( sizeof(WORD) * (2+k) );
  command[0] = 0x0201;
  command[1] = CONFIGITEM;
  va_list word_list;
  va_start( word_list, k);
  for (int i=0; i<k; ++i) {
    command[i+2] = (WORD) va_arg( word_list, int);
  }
  va_end( word_list );

  send_and_check(command, 2+k, make_word_array(2, 0x8201, 0x0000), 2);
}


void
LaseEdlAcquisitionThread::SET_FUNCTION( WORD SECTORNUM, WORD SECTORFUNC, WORD SECTORSTOP, WORD FLASHFLAG )
{
  WORD* command = make_word_array( 5, 0x020A, SECTORNUM, SECTORFUNC, SECTORSTOP, FLASHFLAG);
  send_and_check(command, 5, make_word_array( 2, 0x820A, SECTORNUM), 2);
}


void
LaseEdlAcquisitionThread::GET_PROFILE( WORD PROFILENUM, WORD PROFILEFORMAT)
{
  WORD* command = make_word_array( 3, 0x0301, PROFILENUM, PROFILEFORMAT);
  send_and_check(command, 3, make_word_array( 2, 0x8301, PROFILEFORMAT), 2);
}


void
LaseEdlAcquisitionThread::CANCEL_PROFILE()
{
  send_and_check(make_word_array(1, 0x0302), 1, make_word_array( 1, 0x8302), 1);
}


void
LaseEdlAcquisitionThread::DO_RESET(WORD RESETLEVEL)
{
  WORD* command = make_word_array( 2, 0x0401, RESETLEVEL);
  send_and_check(command, 2, make_word_array(2, 0x8401, RESETLEVEL), 2);
}


void
LaseEdlAcquisitionThread::TRANS_IDLE()
{
  WORD* command = make_word_array( 1, 0x0402);
  WORD* real_response;
  int   response_size;
  send_and_check(command, 1, make_word_array( 1, 0x8402), 1, &real_response, &response_size);

  bool failed = (real_response[response_size-1] != 0x0001);
  free(real_response);
  if (failed) throw Exception("Failed to set trans idle");
}


void
LaseEdlAcquisitionThread::TRANS_ROTATE(WORD REV)
{
  WORD* command = make_word_array( 2, 0x0403, REV);
  WORD* real_response;
  int   response_size;
  send_and_check(command, 2, make_word_array( 1, 0x8403), 1,
		 &real_response, &response_size);
	     
  bool failed = (real_response[response_size-1] != 0x0002);
  free(real_response);
  if ( failed )  throw Exception("Failed to set trans rotate");
}


void
LaseEdlAcquisitionThread::TRANS_MEASURE()
{
  WORD* command = make_word_array( 1, 0x0404);
  WORD* real_response;
  int   response_size;  
  send_and_check(command, 1, make_word_array( 1, 0x8404), 1, &real_response, &response_size);

  bool failed = (real_response[response_size-2] != 0x0003) ||
                (real_response[response_size-1] != 0x0000);
  free(real_response);
  if ( failed )  throw Exception("Failed set trans measure");
}
