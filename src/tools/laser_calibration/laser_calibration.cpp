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

#include <core/exception.h>
#include <blackboard/remote.h>
#include <config/netconf.h>
#include <netcomm/fawkes/client.h>
#include <tf/transformer.h>
#include <tf/transform_listener.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>
#include <utils/hungarian_method/hungarian.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/geometry.h>

#include <string>
#include <map>
#include <numeric>
#include <random>
#include <limits>

#include <cmath>

using namespace fawkes;
using namespace std;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
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

class InsufficientDataException : public Exception
{
public:
  InsufficientDataException(const char *error) : Exception(error) {}
};

class LaserCalibration
{
public:
  LaserCalibration(LaserInterface *laser, tf::Transformer *tf_transformer,
      NetworkConfiguration *config, string config_path)
: laser_(laser), tf_transformer_(tf_transformer), config_(config),
  config_path_(config_path) {}
  virtual ~LaserCalibration() {}

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
  void
  transform_pointcloud(const string &target_frame, PointCloudPtr cloud) {
    for (auto &point : cloud->points) {
      // TODO: convert time stamp correctly
     tf::Stamped<tf::Point> point_in_laser_frame(
         tf::Point(point.x, point.y, point.z),
         fawkes::Time(), cloud->header.frame_id);
     tf::Stamped<tf::Point> point_in_base_frame;
     tf_transformer_->transform_point(
         target_frame, point_in_laser_frame, point_in_base_frame);
     point.x = static_cast<float>(point_in_base_frame[0]);
     point.y = static_cast<float>(point_in_base_frame[1]);
     point.z = static_cast<float>(point_in_base_frame[2]);
    }
  }
  PointCloudPtr filter_cloud_in_rear(PointCloudPtr input) {
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-2., -0.8);
    PointCloudPtr output(new PointCloud());
    pass.filter(*output);
    return output;
  }
  float get_mean_z(PointCloudPtr cloud) {
    if (cloud->points.size() < min_points) {
      stringstream error;
      error << "Not enough laser points in rear cloud, got "
            << cloud->size() << ", need " << min_points;
      throw InsufficientDataException(error.str().c_str());
    }
    vector<float> zs;
    zs.resize(cloud->points.size());
    for (auto &point : *cloud) {
      zs.push_back(point.z);
    }
    return accumulate(zs.begin(), zs.end(), 0.) / zs.size();
  }
  PointCloudPtr filter_left_cloud(PointCloudPtr input) {
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0., 2.);
    PointCloudPtr output(new PointCloud());
    pass.filter(*output);
    return output;
  }
  PointCloudPtr filter_right_cloud(PointCloudPtr input) {
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2., 0.);
    PointCloudPtr output(new PointCloud());
    pass.filter(*output);
    return output;
  }

protected:
  LaserInterface *laser_;
  tf::Transformer *tf_transformer_;
  NetworkConfiguration *config_;
  const string config_path_;
  const static long sleep_time_ = 50000;
  const static uint max_iterations_ = 100;
  const static size_t min_points = 10;
};

class RollCalibration : public LaserCalibration
{
public:
  RollCalibration(LaserInterface *laser, tf::Transformer *tf_transformer,
      NetworkConfiguration *config, string config_path)
  : LaserCalibration(laser, tf_transformer, config, config_path) {}

  virtual void calibrate() {
    printf("Starting to calibrate roll angle.\n");
    float lrd = 2 * threshold;
    uint iterations = 0;
    do {
      try {
        lrd = get_lr_mean_diff();
      } catch (InsufficientDataException &e) {
        printf("Insufficient data: %s\n", e.what_no_backtrace());
        usleep(sleep_time_);
        continue;
      }
      printf("Left-right difference is %f.\n", lrd);
      float old_roll = config_->get_float(config_path_.c_str());
      float new_roll = get_new_roll(lrd, old_roll);
      printf("Updating roll from %f to %f.\n", old_roll, new_roll);
      config_->set_float(config_path_.c_str(), new_roll);
      usleep(sleep_time_);
    } while (abs(lrd) > threshold && ++iterations < max_iterations_);
    printf("Roll calibration finished.\n");
  }


protected:
  float get_lr_mean_diff() {
    laser_->read();
    PointCloudPtr cloud = laser_to_pointcloud(*laser_);
    PointCloudPtr calib_cloud = filter_calibration_cloud(cloud);
    transform_pointcloud("base_link", cloud);
    PointCloudPtr rear_cloud = filter_cloud_in_rear(cloud);
    PointCloudPtr left_cloud = filter_left_cloud(rear_cloud);
    PointCloudPtr right_cloud = filter_right_cloud(rear_cloud);
    if (left_cloud->size() < min_points) {
      stringstream error;
      error << "Not enough laser points on the left, got "
            << left_cloud->size() << ", need " << min_points;
      throw InsufficientDataException(error.str().c_str());
    }
    if (right_cloud->size() < min_points) {
      stringstream error;
      error << "Not enough laser points on the right, got "
            << right_cloud->size() << ", need " << min_points;
      throw InsufficientDataException(error.str().c_str());
    }
    printf("Using %zu points on the left, %zu points on the right\n",
        left_cloud->size(), right_cloud->size());
    return get_mean_z(left_cloud) - get_mean_z(right_cloud);
  }
  float get_new_roll(float mean_error, float old_roll) {
    return old_roll + 0.5 * mean_error;
  }
  PointCloudPtr filter_calibration_cloud(PointCloudPtr input) {
    PointCloudPtr filtered(new PointCloud());
    std::vector<int> indices;
    input->is_dense = false;
    pcl::removeNaNFromPointCloud(*input, *filtered, indices);
    return filtered;
  }

protected:
  // TODO: make threshold and min_points configurable
  constexpr static float threshold = 0.00001;
};

class PitchCalibration : public LaserCalibration
{
public:
  PitchCalibration(LaserInterface *laser, tf::Transformer *tf_transformer,
      NetworkConfiguration *config, string config_path)
  : LaserCalibration(laser, tf_transformer, config, config_path) {}
  virtual void calibrate() {
    printf("Starting pitch angle calibration.\n");
    float mean_z;
    do {
      laser_->read();
      PointCloudPtr cloud = laser_to_pointcloud(*laser_);
      transform_pointcloud("base_link", cloud);
      PointCloudPtr rear_cloud = filter_cloud_in_rear(cloud);
      printf("Rear cloud has %zu points.\n", rear_cloud->points.size());
      try {
        mean_z = get_mean_z(rear_cloud);
      } catch (InsufficientDataException &e) {
        printf("Insufficient data: %s\n", e.what_no_backtrace());
        usleep(sleep_time_);
        continue;
      }
      printf("Mean z is %f.\n", mean_z);
      float old_pitch = config_->get_float(config_path_.c_str());
      float new_pitch = get_new_pitch(mean_z, old_pitch);
      printf("Updating pitch from %f to %f.\n", old_pitch, new_pitch);
      config_->set_float(config_path_.c_str(), new_pitch);
      usleep(sleep_time_);
    } while (abs(mean_z) > threshold);
    printf("Pitch calibration finished.\n");
  }
protected:
  float get_new_pitch(float z, float old_pitch) {
    // Note: We could also compute a more accurate new value using the measured
    // distance and height, but this works well enough.
    return old_pitch - z;
  }
protected:
  constexpr static float threshold = 0.001;
};

class YawCalibration : public LaserCalibration
{
public:
  YawCalibration(LaserInterface *laser, LaserInterface *front_laser,
      tf::Transformer *tf_transformer, NetworkConfiguration *config,
      string config_path)
  : LaserCalibration(laser, tf_transformer, config, config_path),
    front_laser_(front_laser), step_(init_step_), random_float_dist(0,1),
    min_cost_(numeric_limits<float>::max()), min_cost_yaw_(0.) {}
  virtual void calibrate() {
    printf("Starting to calibrate yaw angle.\n");
    float current_cost;
    while (true) {
      try {
        current_cost = get_current_cost();
        break;
      } catch (InsufficientDataException &e) {
        printf("Insufficient data, please move the robot\n");
      }
    }
    uint iterations = 0;
    while (abs(step_) > 0.0005 && iterations++ < max_iterations_) {
      float last_yaw = config_->get_float(config_path_.c_str());
      float next_yaw = get_new_yaw(current_cost, last_yaw);
      printf("Updating yaw from %f to %f (step %f), last cost %f\n",
          last_yaw, next_yaw, step_, current_cost);
      config_->set_float(config_path_.c_str(), next_yaw);
      usleep(sleep_time_);
      try {
        current_cost = get_current_cost();
        if (current_cost < min_cost_) {
          min_cost_ = current_cost;
          min_cost_yaw_ = next_yaw;
        }
      } catch (InsufficientDataException &e) {
        printf("Insufficient data, skipping loop.\n");
        continue;
      }
    }
    if (current_cost > min_cost_) {
      printf("Setting yaw to %f with minimal cost %f\n",
          min_cost_yaw_, min_cost_);
      config_->set_float(config_path_.c_str(), min_cost_yaw_);
    }
    printf("Yaw calibration finished.\n");
  }
protected:
  float get_current_cost() {
    front_laser_->read();
    laser_->read();
    PointCloudPtr front_cloud = laser_to_pointcloud(*front_laser_);
    PointCloudPtr back_cloud = laser_to_pointcloud(*laser_);
    transform_pointcloud("base_link", front_cloud);
    transform_pointcloud("base_link", back_cloud);
    front_cloud = filter_center_cloud(front_cloud);
    back_cloud = filter_center_cloud(back_cloud);
    hungarian_problem_t hp;
    hp.num_rows = front_cloud->points.size();
    hp.num_cols = back_cloud->points.size();
    if ((uint) hp.num_rows < min_points || (uint) hp.num_cols < min_points) {
      stringstream error;
      error << "Not enough points, got " << hp.num_rows << " in the front, "
          << hp.num_cols << " in the back, need " << min_points;
      throw InsufficientDataException(error.str().c_str());
    }
    hp.cost = (int**) calloc(hp.num_rows, sizeof(int*));
    for (int row = 0; row < hp.num_rows; row++) {
      hp.cost[row] = (int*) calloc(hp.num_cols, sizeof(int));
      for (int col = 0; col < hp.num_cols; col++) {
        int cost =
            (int) 100000 * pcl::geometry::distance<Point>(
                front_cloud->points[row],
                back_cloud->points[col]);
        hp.cost[row][col] = cost;
      }
    }
    HungarianMethod solver;
    solver.init(hp.cost, hp.num_rows, hp.num_cols,
        HUNGARIAN_MODE_MINIMIZE_COST);
    solver.solve();
    float total_cost = 0.;
    int assignment_size;
    int *assignment = solver.get_assignment(assignment_size);
    for (int row = 0; row < assignment_size; row++) {
      if (row >= hp.num_rows) {
        continue;
      }
      if (assignment[row] >= hp.num_cols) {
        continue;
      }
      total_cost += hp.cost[row][assignment[row]];
    }
    for (int row = 0; row < hp.num_rows; row++) {
      free(hp.cost[row]);
    }
    free(hp.cost);
    float mean_cost = total_cost / (hp.num_rows * hp.num_cols);
    return mean_cost;
  }
  float get_new_yaw(float current_cost, float last_yaw) {
    static float last_cost = current_cost;
    costs_[last_yaw] = current_cost;
    float next_yaw = last_yaw + step_;
    for (auto &cost_pair : costs_) {
      if (cost_pair.second < current_cost && cost_pair.first != last_yaw) {
        float jump_probability =
            static_cast<float>((current_cost - cost_pair.second)) / current_cost;
        float rand_01 = random_float_dist(random_generator_);
        if (rand_01 > 1 - jump_probability) {
          last_cost = current_cost;
          if (random_float_dist(random_generator_) >= 0.5) {
            step_ = init_step_;
          } else {
            step_ = -init_step_;
          }
          printf("Jumping to %f, cost %f -> %f (probability was %f)\n",
              cost_pair.first, current_cost, cost_pair.second, jump_probability);
          return cost_pair.first;
        }
      }
    }
    if (current_cost > last_cost) {
      step_ = -step_/2;
    }
    last_cost = current_cost;
    return next_yaw;
  }
  PointCloudPtr filter_center_cloud(PointCloudPtr input) {
    pcl::PassThrough<Point> pass_x;
    pass_x.setInputCloud(input);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-2, 2);
    PointCloudPtr x_filtered(new PointCloud());
    pass_x.filter(*x_filtered);
    pcl::PassThrough<Point> pass_y;;
    pass_y.setInputCloud(x_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimitsNegative(true);
    pass_y.setFilterLimits(-0.5,0.5);
    PointCloudPtr xy_filtered_inner(new PointCloud());
    pass_y.filter(*xy_filtered_inner);
    pcl::PassThrough<Point> pass_y_outer;
    pass_y_outer.setInputCloud(xy_filtered_inner);
    pass_y_outer.setFilterFieldName("y");
    pass_y_outer.setFilterLimits(-3,3);
    PointCloudPtr xy_filtered(new PointCloud());
    pass_y_outer.filter(*xy_filtered);
    pcl::PassThrough<Point> pass_z;
    pass_z.setInputCloud(xy_filtered);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.1, 1);
    PointCloudPtr xyz_filtered(new PointCloud());
    pass_z.filter(*xyz_filtered);
    return xyz_filtered;
  }

protected:
  LaserInterface *front_laser_;
  const float init_step_ = 0.02;
  float step_;
  mt19937 random_generator_;
  uniform_real_distribution<float> random_float_dist;
  map<float, float> costs_;
  float min_cost_;
  float min_cost_yaw_;
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
  tf::Transformer *transformer = NULL;
  // Mark the tf listener as unused, we only use its callbacks.
  tf::TransformListener *tf_listener __attribute__((unused)) = NULL;


  // TODO: make these configurable
  const string host = "robotino-base-3";
//  const string host = "localhost";
  const unsigned short int port = FAWKES_TCP_PORT;
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
  // TODO: make laser interface configurable
  const string front_interface_id = "Laser front 360";
  LaserInterface *front_laser = NULL;
  try {
    front_laser = blackboard->open_for_reading<LaserInterface>(
        front_interface_id.c_str());
  } catch (Exception &e) {
    printf("Failed to open Blackboard interface %s\n", interface_id.c_str());
    e.print_trace();
    return -1;
  }
  if (!front_laser->has_writer()) {
    printf("Laser %s does not have a writer!\n", interface_id.c_str());
    return -1;
  }

  const string cfg_transforms_prefix =
      "/plugins/static-transforms/transforms/back_laser/";

  // TODO: create calibration objects here and calibrate
  RollCalibration roll_calibration(
      laser, transformer, netconf, cfg_transforms_prefix + "rot_roll");
  PitchCalibration pitch_calibration(
      laser, transformer, netconf, cfg_transforms_prefix + "rot_pitch");
  YawCalibration yaw_calibration(
      laser, front_laser, transformer, netconf,
      cfg_transforms_prefix + "rot_yaw");
  cout << "Please put the robot in a position such that you only have ground "
       << "behind the robot." << endl
       << "To start pitch and roll calibration, press enter" << endl;
  cin.get();
  pitch_calibration.calibrate();
  roll_calibration.calibrate();
  cout << "Please move the robot such that it can see a wall." << endl
       << "To start yaw calibration, press enter." << endl;
  cin.get();
  yaw_calibration.calibrate();

  return 0;
}
