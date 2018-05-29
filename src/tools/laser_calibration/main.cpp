/***************************************************************************
 *  main.cpp - Laser calibration tool
 *
 *  Created: Tue 18 Jul 2017 15:47:58 CEST 15:47
 *  Copyright  2017-2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "laser_calibration.h"
#include "roll_calibration.h"
#include "pitch_calibration.h"
#include "yaw_calibration.h"
#include "time_offset_calibration.h"

#include <utils/system/argparser.h>
#include <blackboard/remote.h>
#include <config/netconf.h>
#include <netcomm/fawkes/client.h>

#include <tf/transformer.h>
#include <tf/transform_listener.h>

#include <interfaces/Laser360Interface.h>
#include <interfaces/MotorInterface.h>


using namespace fawkes;
using namespace std;

/** Print the usage message.
 * @param program_name The path of the program.
 */
void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] [-r host[:port]]\n"
      " -h                 This help message\n"
      " -r host[:port]     Remote host (and optionally port) to connect to\n"
      " -f front-laser-id  The ID of the front laser blackboard interface\n"
      " -b back-laser-id   The ID of the back laser blackboard interface\n"
      " -R                 Skip roll calibration\n"
      " -P                 Skip pitch calibration\n"
      " -Y                 Skip yaw calibration\n"
      " -T                 Skip time offset calibration\n",
      program_name);
}

/** Run all calibrations.
 *  The command line options allow to enable/disable certain calibrations. By
 *  default, calibrate everything.
 *  @param argc Number of commandline arguments
 *  @param argc The commandline arguments
 *  @return 0 on success, -1 if an error occured.
 */
int
main(int argc, char **argv)
{
  ArgumentParser arg_parser(argc, argv, "hr:f:b:RPYT");
  if (arg_parser.has_arg("h")) {
    print_usage(argv[0]);
    return 0;
  }

  FawkesNetworkClient *client = NULL;
  BlackBoard *blackboard = NULL;
  NetworkConfiguration *netconf = NULL;
  tf::Transformer *transformer = NULL;
  // Mark the tf listener as unused, we only use its callbacks.
  tf::TransformListener *tf_listener __attribute__((unused)) = NULL;


  string host = "localhost";
  unsigned short int port = FAWKES_TCP_PORT;
  if (arg_parser.has_arg("r")) {
    arg_parser.parse_hostport("r", host, port);
  }
  string front_laser_interface_id = "Laser front 360";
  if (arg_parser.has_arg("f")) {
    front_laser_interface_id = string(arg_parser.arg("f"));
  }
  string back_laser_interface_id = "Laser back 360";
  if (arg_parser.has_arg("b")) {
    back_laser_interface_id = string(arg_parser.arg("b"));
  }
  bool calibrate_roll = true;
  if (arg_parser.has_arg("R")) {
    calibrate_roll = false;
  }
  bool calibrate_pitch = true;
  if (arg_parser.has_arg("P")) {
    calibrate_pitch = false;
  }
  bool calibrate_yaw = true;
  if (arg_parser.has_arg("Y")) {
    calibrate_yaw = false;
  }
  bool calibrate_time_offset = true;
  if (arg_parser.has_arg("T")) {
    calibrate_time_offset = false;
  }

  try {
    client = new FawkesNetworkClient(host.c_str(), port);
    client->connect();
    blackboard = new RemoteBlackBoard(client);
    netconf = new NetworkConfiguration(client);
    transformer = new tf::Transformer();
    tf_listener = new tf::TransformListener(blackboard, transformer, true);
  } catch (Exception &e) {
    printf("Failed to connect to remote host at %s:%u\n", host.c_str(), port);
    e.print_trace();
    return -1;
  }

  LaserInterface *laser = NULL;
  try {
    laser = blackboard->open_for_reading<LaserInterface>(
        back_laser_interface_id.c_str());
  } catch (Exception &e) {
    printf("Failed to open Blackboard interface '%s'\n",
        back_laser_interface_id.c_str());
    e.print_trace();
    return -1;
  }
  if (!laser->has_writer()) {
    printf("Laser '%s' does not have a writer!\n",
        back_laser_interface_id.c_str());
    return -1;
  }
  LaserInterface *front_laser = NULL;
  try {
    front_laser = blackboard->open_for_reading<LaserInterface>(
        front_laser_interface_id.c_str());
  } catch (Exception &e) {
    printf("Failed to open Blackboard interface '%s'\n",
        front_laser_interface_id.c_str());
    e.print_trace();
    return -1;
  }
  if (!front_laser->has_writer()) {
    printf("Laser '%s' does not have a writer!\n",
        front_laser_interface_id.c_str());
    return -1;
  }
  MotorInterface *motor = NULL;
  string motor_interface_id = "Robotino";
  try {
    motor = blackboard->open_for_reading<MotorInterface>(
        motor_interface_id.c_str());
  } catch (Exception &e) {
    printf("Failed to open Blackboard interface '%s'\n",
        motor_interface_id.c_str());
    e.print_trace();
    return -1;
  }
  if (!motor->has_writer()) {
    printf("motor '%s' does not have a writer!\n", motor_interface_id.c_str());
    return -1;
  }

  const string cfg_transforms_prefix =
      "/plugins/static-transforms/transforms/back_laser/";

  RollCalibration roll_calibration(
      laser, transformer, netconf, cfg_transforms_prefix + "rot_roll");
  PitchCalibration pitch_calibration(
      laser, transformer, netconf, cfg_transforms_prefix + "rot_pitch");
  YawCalibration yaw_calibration(
      laser, front_laser, transformer, netconf,
      cfg_transforms_prefix + "rot_yaw");
  // TODO: make config path a commandline argument
  TimeOffsetCalibration time_offset_front_calibration(
      front_laser, motor, transformer, netconf, "/hardware/laser/front/time_offset");
  TimeOffsetCalibration time_offset_back_calibration(
      laser, motor, transformer, netconf, "/hardware/laser/back/time_offset");
  if (calibrate_pitch || calibrate_roll) {
    cout << "Please put the robot in a position such that you only have ground "
         << "behind the robot." << endl;
  }
  if (calibrate_pitch) {
    cout << "To start pitch calibration, press enter" << endl;
    cin.get();
    pitch_calibration.calibrate();
    printf("--------------------\n");
  }
  if (calibrate_roll) {
    cout << "To start roll calibration, press enter" << endl;
    cin.get();
    roll_calibration.calibrate();
    printf("--------------------\n");
}
  if (calibrate_yaw) {
    cout << "Please move the robot such that it can see a wall." << endl
         << "To start yaw calibration, press enter." << endl;
    cin.get();
    yaw_calibration.calibrate();
    printf("--------------------\n");
}
  if (calibrate_time_offset) {
    cout << "Move the robot into a corner and make sure that it can rotate "
         << "without hitting any obstacles." << endl
         << "Careful: The robot will start rotating in the next step." << endl
         << "Press Enter to start time offset calibration." << endl;
    cin.get();
    printf("Starting time offset calibration for front laser.\n");
    time_offset_front_calibration.calibrate();
    printf("--------------------\n");
    printf("Starting time offset calibration for back laser.\n");
    time_offset_back_calibration.calibrate();
  }

  return 0;
}
