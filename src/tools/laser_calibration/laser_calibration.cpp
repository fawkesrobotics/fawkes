/***************************************************************************
 *  laser_calibration.cpp - Tool to calibrate laser transforms
 *
 *  Created: Mon 10 Jul 2017 17:37:21 CEST 17:37
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include <utils/system/argparser.h>

#include <blackboard/remote.h>
#include <config/netconf.h>
#include <netcomm/fawkes/client.h>

#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

#include <unistd.h>

using namespace fawkes;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef RefPtr<PointCloud> PointCloudPtr;
typedef Laser360Interface LaserInterface;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h]\n", program_name);
}

inline float
deg2rad(float deg)
{
  return (deg * M_PI / 180.f);
}

class LaserCalibration
{
public:
  LaserCalibration(LaserInterface *laser,
      NetworkConfiguration *config)
  {
    laser_ = laser;
    config_ = config;
  }
  virtual ~LaserCalibration();

  virtual void calibrate() = 0;

protected:
  PointCloudPtr
  laser_to_pointcloud(const LaserInterface &laser) {
    PointCloudPtr cloud = PointCloudPtr(new PointCloud());
    cloud->points.resize(laser.maxlenof_distances());
    cloud->header.frame_id = laser.frame();
    cloud->height = 1;
    cloud->width = laser.maxlenof_distances();
    float const *distances = laser.distances();
    for (uint i = 0; i < laser.maxlenof_distances(); i++) {
      cloud->points[i].x = distances[i]
          * cosf(deg2rad(i) * (360. / laser.maxlenof_distances()));
      cloud->points[i].y = distances[i]
          * sinf(deg2rad(i) * (360. / laser.maxlenof_distances()));
    }
    return cloud;
  }

protected:
  LaserInterface *laser_;
  NetworkConfiguration *config_;
};


int
main(int argc, char **argv)
{
  ArgumentParser arg_parser(argc, argv, "h");
  if (arg_parser.has_arg("h")) {
    print_usage(argv[0]);
    return 0;
  }

  FawkesNetworkClient *client = NULL;
  BlackBoard *blackboard = NULL;
  NetworkConfiguration *netconf = NULL;

  // TODO: make these configurable
  const string host = "robotino-base-3";
  const unsigned short int port = FAWKES_TCP_PORT;
  try {
    client = new FawkesNetworkClient(host.c_str(), port);
    client->connect();
    blackboard = new RemoteBlackBoard(client);
    netconf = new NetworkConfiguration(client);
  } catch (Exception &e) {
    printf("Failed to connect to remote host at %s:%u\n", host.c_str(), port);
    e.print_trace();
    return -1;
  }

  // TODO: make laser interface configurable
  const string interface_id = "Laser back 360";
  LaserInterface *laser = NULL;
  try {
    laser = blackboard->open_for_reading<LaserInterface>(
        interface_id.c_str());
  } catch (Exception &e) {
    printf("Failed to open Blackboard interface %s\n", interface_id.c_str());
    e.print_trace();
    return -1;
  }
  if (!laser->has_writer()) {
    printf("Laser %s does not have a writer!\n", interface_id.c_str());
    return -1;
  }

  // TODO: create calibration objects here and calibrate

  return 0;
}
