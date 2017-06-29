
/***************************************************************************
 *  ffset_pose.cpp - tool to set pose
 *
 *  Created: Mon Mar 23 14:20:07 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/fawkes/client.h>
#include <blackboard/remote.h>
#include <utils/system/argparser.h>
#include <core/threading/thread.h>
#include <netcomm/fawkes/client_handler.h>
#include <tf/types.h>
#include <config/netconf.h>

#include <cstdio>
#include <cmath>
#include <unistd.h>

#include <interfaces/LocalizationInterface.h>

using namespace fawkes;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] [-r host[:port]] [-i ID] [-t SEC] -f FRAME -- <x y theta|x y z qx qy qz qw>\n"
	 " -h              This help message\n"
	 " -r host[:port]  Remote host (and optionally port) to connect to\n"
	 " -i ID           Blackboard interface ID, defaults to \"AMCL\"\n"
	 " -t SEC          Try connecting for SEC seconds\n"
	 " -f FRAME        Frame in which the coordinates are given, defaults to /map\n"
	 "<x y theta>          Set 2D position on ground plane with coordinates\n"
	 "                     (x,y) and orientation theta.\n"
	 "<x y z qx qy qz qw>  Set full 3D pose with position (x,y,z) and\n"
	 "                     orientation quaternion (qx,qy,qz,qw)\n",
	 program_name);
}

void
try_localize(const std::string &host, unsigned short int port, std::string &interface_id,
             std::string frame,
             double translation[3], double rotation[4], double covariance[36])
{
  FawkesNetworkClient *c = new FawkesNetworkClient(host.c_str(), port);
  c->connect();

	if (frame == "") {
		NetworkConfiguration *netconf = NULL;
		try {
			netconf = new NetworkConfiguration(c);
			frame = netconf->get_string("/frames/fixed");
		} catch (Exception &e) {
			printf("WARNING: no frame set and failed to get frame from remote.\n");
			e.print_trace();
		}
		delete netconf;
	}

  BlackBoard *bb = new RemoteBlackBoard(c);
  LocalizationInterface *loc_if =
    bb->open_for_reading<LocalizationInterface>(interface_id.c_str());

  if (! loc_if->has_writer()) {
    bb->close(loc_if);
    delete bb;
    throw Exception("No writer for interface %s, aborting",
		    loc_if->uid());
  }

  LocalizationInterface::SetInitialPoseMessage *ipm =
    new LocalizationInterface::SetInitialPoseMessage();
  ipm->set_frame(frame.c_str());
  ipm->set_translation(translation);
  ipm->set_rotation(rotation);
  ipm->set_covariance(covariance);
  loc_if->msgq_enqueue(ipm);

  // allow for some time so message is actually sent
  usleep(500000);

  bb->close(loc_if);
  delete bb;
  delete c;
}


/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hr:i:t:f:");

  if ( argp.has_arg("h") ) {
    print_usage(argv[0]);
    exit(0);
  }

  char *host_s = (char *)"localhost";
  unsigned short int port = 1910;
  bool free_host = argp.parse_hostport("r", &host_s, &port);
  float try_sec = 0.0;

  std::string host = host_s;
  if ( free_host )  free(host_s);

  std::string interface_id = "AMCL";
  if (argp.has_arg("i")) {
    interface_id = argp.arg("i");
  }

  std::string frame;
  double translation[3] = {0, 0, 0};
  double rotation[4] = {0, 0, 0, 1};
  double covariance[36];
  for (int i = 0; i < 36; ++i)  covariance[i] = 0.f;
  covariance[6*0+0] = 0.5 * 0.5;
  covariance[6*1+1] = 0.5 * 0.5;
  covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

  if (argp.num_items() != 3 && argp.num_items() != 7) {
    fprintf(stderr, "Invalid pose");
    print_usage(argv[0]);
    return -1;
  }

  if (argp.num_items() == 3) {
    translation[0] = argp.parse_item_float(0);
    translation[1] = argp.parse_item_float(1);
    tf::Quaternion q = tf::create_quaternion_from_yaw(argp.parse_item_float(2));
    for (int i = 0; i < 4; ++i) rotation[i] = q[i];
  } else {
    for (int i = 0; i < 3; ++i) translation[i] = argp.parse_item_float(i);
    for (int i = 0; i < 4; ++i) rotation[i] = argp.parse_item_float(i+3);
  }

  if (argp.has_arg("t")) {
    try_sec = argp.parse_float("t");
  }

  if (argp.has_arg("f")) {
    frame = argp.arg("f");
  }

  fawkes::Time start;
  fawkes::Time now(start);
  bool localized = false;
  while (! localized) {
    now.stamp();
    try {
      try_localize(host, port, interface_id, frame, translation, rotation, covariance);
      localized = true;
    } catch (Exception &e) {
      if ((now - &start) > try_sec) {
	      fprintf(stderr, "Failed to localize %s:%u: %s\n",
	              host.c_str(), port, e.what_no_backtrace());
	      break;
      }
      usleep(1000000);
    }
  }

  return localized ? 0 : -1;
}
