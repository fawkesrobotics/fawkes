
/***************************************************************************
 *  kinova_api.h - Kinova API for libusb connection
 *
 *  Created: Tue Jun 04 13:13:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_KINOVA_KINOVA_API_H_
#define __PLUGINS_KINOVA_KINOVA_API_H_

#include "types.h"

#include <libusb.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class JacoArm
{
 public:
  JacoArm();
  virtual ~JacoArm();

  void print_message(jaco_message_t &msg);

  // getter; receiving commands
  jaco_position_t get_cart_pos();
  jaco_position_t get_ang_pos();
  jaco_retract_mode_t get_status();

  // setter; sending command
  void start_api_ctrl();
  void stop_api_ctrl();

  void erase_trajectories();

  void set_control_ang();
  void set_control_cart();

  void set_target(jaco_basic_traj_t &traj);
  void set_target_cart(float x, float y, float z, float euler_1, float euler_2, float euler_3, float finger_1, float finger_2, float finger_3);
  void set_target_cart(float coord[], float fingers[3]);
  void set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, float finger_1, float finger_2, float finger_3);
  void set_target_ang(float joints[], float fingers[3]);

  void push_joystick_button(unsigned short id);
  void release_joystick();

 private:
  static libusb_context  *__lusb_ctx;

  libusb_device_handle   *__lusb_devh;

  void _init_libusb();
  void _get_device_handle();
  void _claim_interface();

  // generic USB data transfer methods
  int _cmd_out_in(jaco_message_t &msg, int cmd_size_in);
  int _cmd_out(short cmd);

  // Jaco specific commands
  int _get_cart_pos(jaco_position_t &pos);
  int _get_ang_pos(jaco_position_t &pos);
  int _send_basic_traj(jaco_basic_traj_t &traj);

  bool __lock;
};

} // end of namespace fawkes

#endif