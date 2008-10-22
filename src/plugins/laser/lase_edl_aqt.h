
/***************************************************************************
 *  acquisition_thread.h - Thread that retrieves the laser data
 *
 *  Created: Wed Oct 08 13:41:02 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_LASE_EDL_AQT_H_
#define __PLUGINS_LASER_LASE_EDL_AQT_H_

#include "acquisition_thread.h"

#include <libpcan.h>

namespace fawkes {
  class Mutex;
}

class LaseEdlAcquisitionThread : public LaserAcquisitionThread
{
 public:
  LaseEdlAcquisitionThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  unsigned int calc_angle_step(unsigned int rotation_freq, unsigned int max_pulse_freq);
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

  void SET_CONFIG(WORD CONFIGITEM, int k, ...);
  void SET_FUNCTION(WORD SECTORNUM, WORD SECTORFUNC,
		   WORD SECTORSTOP, WORD FLASHFLAG);
  void GET_PROFILE(WORD PROFILENUM, WORD PROFILEFORMAT);
  void CANCEL_PROFILE();
  void DO_RESET(WORD RESETLEVEL);
  void TRANS_IDLE();
  void TRANS_ROTATE(WORD REV);
  void TRANS_MEASURE();

 private:
  HANDLE __handle;

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

  unsigned int __min_angle_step;
  unsigned int __number_of_values;
  
};


#endif
