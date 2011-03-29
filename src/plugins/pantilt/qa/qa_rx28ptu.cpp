
/***************************************************************************
 *  qa_rx28ptu.cpp - QA for RX 28 PTU
 *
 *  Created: Tue Jun 16 14:13:12 2009
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

/// @cond QA

#include "../robotis/rx28.h"
#include <utils/time/tracker.h>

#include <cstdio>
#include <unistd.h>

using namespace fawkes;

#define TEST_SERVO 2

int
main(int argc, char **argv)
{
  RobotisRX28 rx28("/dev/ttyUSB0");

  RobotisRX28::DeviceList devl = rx28.discover();

  if (devl.empty()) {
    printf("No devices found\n");
  } else {
    for (RobotisRX28::DeviceList::iterator i = devl.begin(); i != devl.end(); ++i) {
      printf("Found servo with ID %d\n", *i);
    }
  }

  /*
  rx28.set_status_return_level(RobotisRX28::BROADCAST_ID, RobotisRX28::SRL_RESPOND_READ);
  rx28.set_return_delay_time(RobotisRX28::BROADCAST_ID, 0);
  rx28.set_baudrate(RobotisRX28::BROADCAST_ID, 0x22);

  TimeTracker tt;
  unsigned int ttc_goto = tt.add_class("Send goto");
  unsigned int ttc_read_pos = tt.add_class("Read position");
  unsigned int ttc_read_all = tt.add_class("Read all table values");
  unsigned int ttc_start_read_all = tt.add_class("Starting to read all values");
  unsigned int ttc_finish_read_all_1 = tt.add_class("1 Finishing reading all values");

  rx28.goto_position(2, 512);
  rx28.set_compliance_values(1, 0, 96, 0, 96);

  rx28.goto_positions(2, 1, 230, 2, 300);

  return 0;

  for (unsigned int i = 0; i < 10; ++i) {
    tt.ping_start(ttc_goto);
    rx28.goto_position(1, 230);
    tt.ping_end(ttc_goto);
    sleep(1);
    tt.ping_start(ttc_goto);
    rx28.goto_position(1, 630);
    tt.ping_end(ttc_goto);
    sleep(1);

    tt.ping_start(ttc_read_all);
    rx28.read_table_values(1);
    tt.ping_end(ttc_read_all);

    tt.ping_start(ttc_read_pos);
    rx28.get_position(1, true);
    tt.ping_end(ttc_read_pos);

    tt.ping_start(ttc_start_read_all);
    rx28.start_read_table_values(1);
    tt.ping_end(ttc_start_read_all);
    tt.ping_start(ttc_finish_read_all_1);
    rx28.finish_read_table_values();
    tt.ping_end(ttc_finish_read_all_1);
  }

  tt.print_to_stdout();


  //printf("Setting ID\n");
  //rx28.set_id(1, 2);

  printf("Setting ID back\n");
  rx28.set_id(2, 1);

  for (unsigned char i = 0; i <= 1; ++i) {
    if (rx28.ping(i, 500)) {
      printf("****************** RX28 ID %u alive\n", i);
    } else {
      //printf("RX28 ID %u dead (not connected?)\n", i);
    }
  }

  try {
    rx28.read_table_values(1);
  } catch (Exception &e) {
    printf("Reading table values failed\n");
  }

  try {
    rx28.goto_position(2, 1000);
  } catch (Exception &e) {
  }

  sleep(2);
  */

  try {
    /*
    rx28.goto_position(1, 430);
    rx28.goto_position(2, 512);
    sleep(1);


    rx28.goto_position(1, 300);
    rx28.goto_position(2, 300);

    sleep(3);

    rx28.goto_position(1, 700);
    rx28.goto_position(2, 700);

    sleep(3);

    */

    //rx28.set_torque_enabled(0xFE, false);

    for (unsigned int i = 0; i < 5; ++i) {
      try {
        //rx28.goto_position(TEST_SERVO, 800);
        //sleep(1);
        //rx28.goto_position(TEST_SERVO, 400);
        rx28.read_table_values(TEST_SERVO);
        //sleep(1);
      } catch (Exception &e) {
        rx28.ping(TEST_SERVO);
        e.print_trace();
      }
    }
    // for (RobotisRX28::DeviceList::iterator i = devl.begin(); i != devl.end(); ++i) {
    //   unsigned int angle_cw_limit, angle_ccw_limit, down_calib, up_calib;
    //   unsigned char voltage_low, voltage_high;
    //   unsigned char compl_cw_margin, compl_cw_slope, compl_ccw_margin, compl_ccw_slope;
    //   rx28.get_angle_limits(*i, angle_cw_limit, angle_ccw_limit);
    //   rx28.get_voltage_limits(*i, voltage_low, voltage_high);
    //   rx28.get_calibration(*i, down_calib, up_calib);
    //   rx28.get_compliance_values(*i, compl_cw_margin, compl_cw_slope, compl_ccw_margin, compl_ccw_slope);

    //   printf("Servo %03u, model number:        %u\n", *i, rx28.get_model(*i));
    //   printf("Servo %03u, current position:    %u\n", *i, rx28.get_position(*i));
    //   printf("Servo %03u, firmware version:    %u\n", *i, rx28.get_firmware_version(*i));
    //   printf("Servo %03u, baudrate:            %u\n", *i, rx28.get_baudrate(*i));
    //   printf("Servo %03u, delay time:          %u\n", *i, rx28.get_delay_time(*i));
    //   printf("Servo %03u, angle limits:        CW: %u  CCW: %u\n", *i, angle_cw_limit, angle_ccw_limit);
    //   printf("Servo %03u, temperature limit:   %u\n", *i, rx28.get_temperature_limit(*i));
    //   printf("Servo %03u, voltage limits:      %u to %u\n", *i, voltage_low, voltage_high);
    //   printf("Servo %03u, max torque:          %u\n", *i, rx28.get_max_torque(*i));
    //   printf("Servo %03u, status return level: %u\n", *i, rx28.get_status_return_level(*i));
    //   printf("Servo %03u, alarm LED:           %u\n", *i, rx28.get_alarm_led(*i));
    //   printf("Servo %03u, alarm shutdown:      %u\n", *i, rx28.get_alarm_shutdown(*i));
    //   printf("Servo %03u, calibration:         %u to %u\n", *i, down_calib, up_calib);
    //   printf("Servo %03u, torque enabled:      %s\n", *i, rx28.is_torque_enabled(*i) ? "Yes" : "No");
    //   printf("Servo %03u, LED enabled:         %s\n", *i, rx28.is_led_enabled(*i) ? "Yes" : "No");
    //   printf("Servo %03u, compliance:          CW_M: %u  CW_S: %u  CCW_M: %u  CCW_S: %u\n", *i,
    // 	     compl_cw_margin, compl_cw_slope, compl_ccw_margin, compl_ccw_slope);
    //   printf("Servo %03u, goal position:       %u\n", *i, rx28.get_goal_position(*i));
    //   printf("Servo %03u, goal speed:          %u\n", *i, rx28.get_goal_speed(*i));
    //   printf("Servo %03u, torque limit:        %u\n", *i, rx28.get_torque_limit(*i));
    //   printf("Servo %03u, speed:               %u\n", *i, rx28.get_speed(*i));
    //   printf("Servo %03u, load:                %u\n", *i, rx28.get_load(*i));
    //   printf("Servo %03u, voltage:             %u\n", *i, rx28.get_voltage(*i));
    //   printf("Servo %03u, temperature:         %u\n", *i, rx28.get_temperature(*i));
    //   printf("Servo %03u, moving:              %s\n", *i, rx28.is_moving(*i) ? "Yes" : "No");
    //   printf("Servo %03u, Locked:              %s\n", *i, rx28.is_locked(*i) ? "Yes" : "No");
    //   printf("Servo %03u, Punch:               %u\n", *i, rx28.get_punch(*i));
    // }
  } catch (Exception &e) {
    e.print_trace();
  }

  /*
  sleep(2);

  try {
    rx28.goto_position(2, 800);
  } catch (Exception &e) {
  }
  */

//   std::list<unsigned char> disc = rx28.discover();

//   if (disc.empty()) {
//     printf("No devices found\n");
//   } else {
//     for (std::list<unsigned char>::iterator i = disc.begin(); i != disc.end(); ++i) {
//       printf("Found servo with ID %d\n", *i);
//     }
//   }

  return 0;
}

/// @endcond
