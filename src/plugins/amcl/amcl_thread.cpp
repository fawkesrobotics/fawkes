/***************************************************************************
 *  amcl_thread.cpp - Thread to perform localization
 *
 *  Created: Wed May 16 16:04:41 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 *             2012  Daniel Ewert
 *             2012  Kathrin Goffart (Robotino Hackathon 2012)
 *             2012  Kilian Hinterwaelder  (Robotino Hackathon 2012)
 *             2012  Marcel Prochnau (Robotino Hackathon 2012)
 *             2012  Jannik Altgen (Robotino Hackathon 2012)
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

/* Based on amcl_node.cpp from the ROS amcl package
 * Copyright (c) 2008, Willow Garage, Inc.
 */

#include "amcl_thread.h"

#include <utils/math/angle.h>
#include <fvutils/readers/png.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <cstdlib>
#include <cstdio>

#ifdef HAVE_ROS
#  include <ros/node_handle.h>
#  include <geometry_msgs/PoseArray.h>
#endif

#define CFG_PREFIX "/plugins/amcl/"

using namespace fawkes;

static double normalize(double z) {
  return atan2(sin(z), cos(z));
}

static double angle_diff(double a, double b) {
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0)
    d2 *= -1.0;
  if (fabs(d1) < fabs(d2))
    return (d1);
  else
    return (d2);
}

/** @class AmclThread "amcl_thread.h"
 * Thread to perform Adaptive Monte Carlo Localization.
 * @author Tim Niemueller
 */

std::vector<std::pair<int,int> > AmclThread::free_space_indices;

/** Constructor. */
AmclThread::AmclThread()
  : Thread("AmclThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
    TransformAspect(TransformAspect::BOTH, "Pose")
{
  map_ = NULL;
  conf_mutex_ = new Mutex();
}

/** Destructor. */
AmclThread::~AmclThread()
{
  delete conf_mutex_;
}

void AmclThread::init()
{
  cfg_map_file_ =
    std::string(CONFDIR) + "/" + config->get_string(CFG_PREFIX"map_file");
  cfg_resolution_ = config->get_float(CFG_PREFIX"resolution");
  cfg_origin_x_ = config->get_float(CFG_PREFIX"origin_x");
  cfg_origin_y_ = config->get_float(CFG_PREFIX"origin_y");
  cfg_origin_theta_ = config->get_float(CFG_PREFIX"origin_theta");
  cfg_occupied_thresh_ = config->get_float(CFG_PREFIX"occupied_threshold");
  cfg_free_thresh_ = config->get_float(CFG_PREFIX"free_threshold");
  cfg_laser_ifname_ = config->get_string(CFG_PREFIX"laser_interface_id");
  cfg_pose_ifname_ = config->get_string(CFG_PREFIX"pose_interface_id");

  read_map();

  sent_first_transform_ = false;
  latest_tf_valid_ = false;
  pf_ = NULL;
  resample_count_ = 0;
  odom_ = NULL;
  laser_ = NULL;
  // private_nh_="~";
  initial_pose_hyp_ = NULL;
  first_map_received_ = false;
  first_reconfigure_call_ = true;

  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);

  save_pose_rate = config->get_float(CFG_PREFIX"save_pose_rate");
  laser_min_range = config->get_float(CFG_PREFIX"laser_min_range");
  laser_max_range = config->get_float(CFG_PREFIX"laser_max_range");
  pf_err = config->get_float(CFG_PREFIX"pf_err");
  pf_z = config->get_float(CFG_PREFIX"pf_z");
  alpha1_ = config->get_float(CFG_PREFIX"alpha1");
  alpha2_ = config->get_float(CFG_PREFIX"alpha2");
  alpha3_ = config->get_float(CFG_PREFIX"alpha3");
  alpha4_ = config->get_float(CFG_PREFIX"alpha4");
  alpha5_ = config->get_float(CFG_PREFIX"alpha5");
  z_hit_ = config->get_float(CFG_PREFIX"z_hit");
  z_short_ = config->get_float(CFG_PREFIX"z_short");
  z_max_ = config->get_float(CFG_PREFIX"z_max");
  z_rand_ = config->get_float(CFG_PREFIX"z_rand");
  sigma_hit_ = config->get_float(CFG_PREFIX"sigma_hit");
  lambda_short_ = config->get_float(CFG_PREFIX"lambda_short");
  laser_likelihood_max_dist_ =
    config->get_float(CFG_PREFIX"laser_likelihood_max_dist");
  d_thresh_ = config->get_float(CFG_PREFIX"d_thresh");
  a_thresh_ = config->get_float(CFG_PREFIX"a_thresh");
  alpha_slow_ = config->get_float(CFG_PREFIX"alpha_slow");
  alpha_fast_ = config->get_float(CFG_PREFIX"alpha_fast");
  angle_increment_ = config->get_float(CFG_PREFIX"angle_increment");

  max_beams_ = config->get_uint(CFG_PREFIX"max_beams");
  min_particles_ = config->get_uint(CFG_PREFIX"min_particles");
  max_particles_ = config->get_uint(CFG_PREFIX"max_particles");
  resample_interval_ = config->get_uint(CFG_PREFIX"resample_interval");

  odom_frame_id_ = config->get_string(CFG_PREFIX"odom_frame_id");
  base_frame_id_ = config->get_string(CFG_PREFIX"base_frame_id");
  laser_frame_id_ = config->get_string(CFG_PREFIX"laser_frame_id");
  global_frame_id_ = config->get_string(CFG_PREFIX"global_frame_id");

  std::string tmp_model_type;
  tmp_model_type = config->get_string(CFG_PREFIX"laser_model_type");

  if (tmp_model_type == "beam")
    laser_model_type_ = amcl::LASER_MODEL_BEAM;
  else if (tmp_model_type == "likelihood_field")
    laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
  else {
    logger->log_warn(name(),
		     "Unknown laser model type \"%s\"; "
                     "defaulting to likelihood_field model",
		     tmp_model_type.c_str());
    laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
  }

  tmp_model_type = config->get_string(CFG_PREFIX"odom_model_type");
  if (tmp_model_type == "diff")
    odom_model_type_ = amcl::ODOM_MODEL_DIFF;
  else if (tmp_model_type == "omni")
    odom_model_type_ = amcl::ODOM_MODEL_OMNI;
  else {
    logger->log_warn(name(),
		     "Unknown odom model type \"%s\"; defaulting to diff model",
		     tmp_model_type.c_str());
    odom_model_type_ = amcl::ODOM_MODEL_DIFF;
  }

  try {
    init_pose_[0] = config->get_float(CFG_PREFIX"init_pose_x");
  } catch (Exception &e) {} // ignored, use default

  try {
    init_pose_[1] = config->get_float(CFG_PREFIX"init_pose_y");
  } catch (Exception &e) {} // ignored, use default

  try {
    init_pose_[2] = config->get_float(CFG_PREFIX"init_pose_a");
  } catch (Exception &e) {} // ignored, use default

  try {
    init_cov_[0] = config->get_float(CFG_PREFIX"init_cov_xx");
  } catch (Exception &e) {} // ignored, use default

  try {
    init_cov_[1] = config->get_float(CFG_PREFIX"init_cov_yy");
  } catch (Exception &e) {} // ignored, use default

  try {
    init_cov_[2] = config->get_float(CFG_PREFIX"init_cov_aa");
  } catch (Exception &e) {} // ignored, use default

  transform_tolerance_ = config->get_float(CFG_PREFIX"transform_tolerance");

  if (min_particles_ > max_particles_) {
    logger->log_warn(name(),
		     "You've set min_particles to be less than max particles, "
                     "this isn't allowed so they'll be set to be equal.");
    max_particles_ = min_particles_;
  }

  //logger->log_info(name(),"calling pf_alloc with %d min and %d max particles",
  //                 min_particles_, max_particles_);
  pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
		 (pf_init_model_fn_t) AmclThread::uniform_pose_generator,
		 (void *) map_);

  pf_init_model(pf_, (pf_init_model_fn_t)AmclThread::uniform_pose_generator,
		(void *)map_);

  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter

  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  //pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  //pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  //double *q_values = pos3d_if_->rotation();
  //tf::Quaternion q(q_values[0], q_values[1], q_values[2], q_values[3]);
  //btScalar unused_pitch, unused_roll, yaw;
  //btMatrix3x3(q).getRPY(unused_roll, unused_pitch, yaw);

  // TODO: meaningful initial pose
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0]; //last_covariance_[6 * 0 + 0];
  pf_init_pose_cov.m[1][1] = init_cov_[1]; //last_covariance_[6 * 1 + 1];
  pf_init_pose_cov.m[2][2] = init_cov_[2]; //last_covariance_[6 * 5 + 5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  odom_ = new amcl::AMCLOdom();

  if (odom_model_type_ == amcl::ODOM_MODEL_OMNI)
    odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  else
    odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_);

  // Laser
  laser_ = new amcl::AMCLLaser(max_beams_, map_);

  if (laser_model_type_ == amcl::LASER_MODEL_BEAM) {
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_,
			 lambda_short_, 0.0);
  } else {
    logger->log_info(name(),
		     "Initializing likelihood field model; "
                     "this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
				    laser_likelihood_max_dist_);
    logger->log_info(name(), "Done initializing likelihood field model.");
  }

  single_laser_ = new amcl::AMCLLaser(max_beams_, map_);

  //cloud_pub_interval.fromSec(1.0);

#ifdef HAVE_ROS
  pose_pub_ =
    rosnode->advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ =
    rosnode->advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  initial_pose_sub_ =
    rosnode->subscribe("initialpose", 2,
		       &AmclThread::initial_pose_received, this);
#endif

  laser_if_ =
    blackboard->open_for_reading<Laser360Interface>(cfg_laser_ifname_.c_str());
  pos3d_if_ =
    blackboard->open_for_writing<Position3DInterface>(cfg_pose_ifname_.c_str());

  apply_initial_pose();
}


void
AmclThread::loop()
{
  laser_if_->read();
  float* laser_distances = laser_if_->distances();

  MutexLocker lock(conf_mutex_);

  //logger->log_debug(name(), "Transform 1");
  tf::Stamped<tf::Pose>
    ident(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
          Time(), laser_frame_id_);
  tf::Stamped<tf::Pose> laser_pose;
  try {
    tf_listener->transform_pose(base_frame_id_, ident, laser_pose);
  } catch (fawkes::tf::LookupException& e) {
    logger->log_error(name(), "Failed to lookup transform from %s to %s.",
                      laser_frame_id_.c_str(), base_frame_id_.c_str());
    logger->log_error(name(), e);
    return;
  } catch (fawkes::tf::TransformException& e) {
    logger->log_error(name(), "Transform error from %s to %s, exception follows.",
                      laser_frame_id_.c_str(), base_frame_id_.c_str());
    logger->log_error(name(), e);
    return;
  } catch (fawkes::Exception& e) {
    logger->log_error(name(), "Generic exception for transform from %s to %s.",
                      laser_frame_id_.c_str(), base_frame_id_.c_str());
    logger->log_error(name(), e);
    return;
  }

  laser_update_ = true;

  /*
  tf::Stamped<tf::Pose>
    ident(tf::Transform(tf::Quaternion(0, 0, 0, 1),
                        tf::Vector3(0, 0, 0)), Time(), laser_frame_id_);
  tf::Stamped<tf::Pose> laser_pose;

  try {
    tf_listener->transform_pose(base_frame_id_, ident, laser_pose);
  } catch (tf::TransformException& e) {
    logger->log_error(name(), "Couldn't transform from %s to %s, "
                      "even though the message notifier is in use",
                      laser_frame_id_.c_str(), base_frame_id_.c_str());
    logger->log_error(name(), e);
    return;
  }
  */

  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.getOrigin().x();
  laser_pose_v.v[1] = laser_pose.getOrigin().y();

  // laser mounting angle gets computed later -> set to 0 here!
  laser_pose_v.v[2] = 0;
  single_laser_->SetLaserPose(laser_pose_v);
  //logger->log_debug(name(),
  //		      "Received laser's pose wrt robot: %.3f %.3f %.3f",
  //		      laser_pose_v.v[0], laser_pose_v.v[1], laser_pose_v.v[2]);

  // Where was the robot when this scan was taken?
  tf::Stamped<tf::Pose> odom_pose;
  pf_vector_t pose;
  Time t(0, 0);
  if (!get_odom_pose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
                     &t, base_frame_id_)) 
  {
    logger->log_error(name(), "Couldn't determine robot's pose "
                      "associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();

  if (pf_init_) {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update =
      fabs(delta.v[0]) > d_thresh_ ||
      fabs(delta.v[1]) > d_thresh_ ||
      fabs(delta.v[2]) > a_thresh_;

    if (update) {
      laser_update_ = true;
    }
  }

  bool force_publication = false;
  if (!pf_init_) {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    laser_update_ = true;
    force_publication = true;

    resample_count_ = 0;
  } else if (pf_init_ && laser_update_) {
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    amcl::AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    //logger->log_debug(name(), "Updating Odometry");
    odom_->UpdateAction(pf_, (amcl::AMCLSensorData*) &odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if (laser_update_) {
    amcl::AMCLLaserData ldata;
    ldata.sensor = single_laser_;
    ldata.range_count = laser_if_->maxlenof_distances();

    double angle_min = 0;
    double angle_increment = deg2rad(angle_increment_);

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

    // Apply range min/max thresholds, if the user supplied them
    if (laser_max_range_ > 0.0)
      ldata.range_max = (float) laser_max_range_;
    else
      ldata.range_max = HUGE;
    double range_min;
    if (laser_min_range_ > 0.0)
      range_min = (float) laser_min_range_;
    else
      range_min = 0.0;
    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];

    for (int i = 0; i < ldata.range_count; i++) {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if (laser_distances[i] <= range_min)
	ldata.ranges[i][0] = ldata.range_max;
      else
	ldata.ranges[i][0] = laser_distances[i];
      // Compute bearing
      ldata.ranges[i][1] = angle_min + (i * angle_increment);
    }

    single_laser_->UpdateSensor(pf_, (amcl::AMCLSensorData*) &ldata);

    laser_update_ = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if (!(++resample_count_ % resample_interval_)) {
      //logger->log_info(name(), "Resample INFO!");
      pf_update_resample(pf_);
      resampled = true;
    }
    pf_sample_set_t* set = (pf_->sets) + pf_->current_set;
    logger->log_debug(name(), "Num samples: %d", set->sample_count);

#ifdef HAVE_ROS
    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = global_frame_id_;
    cloud_msg.poses.resize(set->sample_count);
    for (int i = 0; i < set->sample_count; i++) {
      tf::Quaternion q(tf::Vector3(0,0,1),
		       set->samples[i].pose.v[2]);
      cloud_msg.poses[i].position.x = set->samples[i].pose.v[0];
      cloud_msg.poses[i].position.y = set->samples[i].pose.v[1];
      cloud_msg.poses[i].position.z = 0.;
      cloud_msg.poses[i].orientation.x   = q.x();
      cloud_msg.poses[i].orientation.y   = q.y();
      cloud_msg.poses[i].orientation.z   = q.z();
      cloud_msg.poses[i].orientation.w   = q.w();
    }

    particlecloud_pub_.publish(cloud_msg);
#endif
  }

  if (resampled || force_publication) {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for (int hyp_count = 0;
	 hyp_count < pf_->sets[pf_->current_set].cluster_count;
	 hyp_count++) {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean,
				&pose_cov)) {
	logger->log_error(name(), "Couldn't get stats on cluster %d",
			  hyp_count);
	break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if (hyps[hyp_count].weight > max_weight) {
	max_weight = hyps[hyp_count].weight;
	max_weight_hyp = hyp_count;
      }
    }

    if (max_weight > 0.0) {
      logger->log_debug(name(), "Max weight pose: %.3f %.3f %.3f",
			hyps[max_weight_hyp].pf_pose_mean.v[0],
			hyps[max_weight_hyp].pf_pose_mean.v[1],
			hyps[max_weight_hyp].pf_pose_mean.v[2]);

      /*
	puts("");
	pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
	puts("");
      */
#ifdef HAVE_ROS
      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = ros::Time();
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::Quaternion q(tf::Vector3(0,0,1),
		       hyps[max_weight_hyp].pf_pose_mean.v[2]);
      p.pose.pose.orientation.x = q.x();
      p.pose.pose.orientation.y = q.y();
      p.pose.pose.orientation.z = q.z();
      p.pose.pose.orientation.w = q.w();
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for (int i = 0; i < 2; i++) {
	for (int j = 0; j < 2; j++) {
	  // Report the overall filter covariance, rather than the
	  // covariance for the highest-weight cluster
	  //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
	  last_covariance_[6 * i + j] = set->cov.m[i][j];
	}
      }

      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      last_covariance_[6 * 5 + 5] = set->cov.m[2][2];

      pose_pub_.publish(p);
#endif
      //last_published_pose = p;
      //logger->log_debug(name(), "New pose: %6.3f %6.3f %6.3f",
      //		hyps[max_weight_hyp].pf_pose_mean.v[0],
      //		hyps[max_weight_hyp].pf_pose_mean.v[1],
      //		hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;

      try {
	tf::Transform
          tmp_tf(tf::create_quaternion_from_yaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                 tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                             hyps[max_weight_hyp].pf_pose_mean.v[1], 0.0));
	tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                             laser_if_->timestamp(),
                                             base_frame_id_);
	tf_listener->transform_pose(odom_frame_id_,
                                    tmp_tf_stamped, odom_to_map);
      } catch (Exception &e) {
	logger->log_debug(name(),
			  "Failed to subtract base to odom transform");
	return;
      }
      latest_tf_ =
        tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                      tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      Time transform_expiration =
        (*laser_if_->timestamp() + transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
					  transform_expiration,
					  global_frame_id_, odom_frame_id_);
      this->tf_publisher->send_transform(tmp_tf_stamped);
      sent_first_transform_ = true;
    } else {
      logger->log_error(name(), "No pose!");
    }
  } else if (latest_tf_valid_) {
    // Nothing changed, so we'll just republish the last transform, to keep
    // everybody happy.
    Time transform_expiration =
      (*laser_if_->timestamp() + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
					transform_expiration,
                                        global_frame_id_, odom_frame_id_);
    tf_publisher->send_transform(tmp_tf_stamped);

    // Is it time to save our last pose to the param server
    /*
      Time now(clock);
      if ((save_pose_period > 0.0) &&
      (now - save_pose_last_time) >= save_pose_period) {
      // We need to apply the last transform to the latest odom pose to get
      // the latest map pose to store.  We'll take the covariance from
      // last_published_pose.
      tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
      double yaw, pitch, roll;
      map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

      private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
      private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
      private_nh_.setParam("initial_pose_a", yaw);
      private_nh_.setParam("initial_cov_xx",
      last_published_pose.pose.covariance[6 * 0 + 0]);
      private_nh_.setParam("initial_cov_yy",
      last_published_pose.pose.covariance[6 * 1 + 1]);
      private_nh_.setParam("initial_cov_aa",
      last_published_pose.pose.covariance[6 * 5 + 5]);
      save_pose_last_time = now;
      }
    */
  }

}

void AmclThread::finalize()
{
  if (map_) {
    map_free(map_);
    map_ = NULL;
  }
  delete initial_pose_hyp_;
  initial_pose_hyp_ = NULL;

  pose_pub_.shutdown();
}

bool
AmclThread::get_odom_pose(tf::Stamped<tf::Pose>& odom_pose, double& x,
                          double& y, double& yaw,
                          const fawkes::Time* t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose>
    ident(tf::Transform(tf::Quaternion(0, 0, 0, 1),
			tf::Vector3(0, 0, 0)), t, f);
  try {
    tf_listener->transform_pose(odom_frame_id_, ident, odom_pose);
  } catch (Exception &e) {
    logger->log_warn(name(),
		     "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch, roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  return true;
}

void
AmclThread::read_map()
{
  firevision::PNGReader png_reader(cfg_map_file_.c_str());
  map_width_ = png_reader.pixel_width();
  map_height_ = png_reader.pixel_height();
  logger->log_info(name(), "Reading map of size %ux%u", map_width_,
		   map_height_);
  unsigned char *img_buffer = malloc_buffer(firevision::YUV422_PLANAR,
					    map_width_, map_height_);
  png_reader.set_buffer(img_buffer);
  png_reader.read();

  map_ = map_alloc();
  map_->size_x = map_width_;
  map_->size_y = map_height_;
  map_->scale = cfg_resolution_;
  map_->origin_x = cfg_origin_x_ + (map_->size_x / 2) * map_->scale;
  map_->origin_y = cfg_origin_y_ + (map_->size_y / 2) * map_->scale;
  map_->cells =
    (map_cell_t*) malloc(sizeof(map_cell_t) * map_->size_x * map_->size_y);

  for (unsigned int h = 0; h < map_height_; ++h) {
    for (unsigned int w = 0; w < map_width_; ++w) {
      unsigned int i = h * map_width_ + w;
      float y = img_buffer[i] / 255.;

      if (y > cfg_occupied_thresh_) {
	map_->cells[i].occ_state = 1;
      } else if (y <= cfg_free_thresh_) {
	map_->cells[i].occ_state = 0;
	free_space_indices.push_back(std::make_pair(w,h));
      } else {
	map_->cells[i].occ_state = -1;
      }
    }

  }
  free(img_buffer);
}

void
AmclThread::apply_initial_pose()
{
  if (initial_pose_hyp_ != NULL && map_ != NULL) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean,
	    initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;
  }
}

pf_vector_t
AmclThread::uniform_pose_generator(void* arg)
{
  map_t* map = (map_t*) arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;
  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

  pf_vector_t p;
  for (;;) {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == 0))
      break;
  }
#endif
  return p;
}


#ifdef HAVE_ROS
void
AmclThread::initial_pose_received(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  MutexLocker lock(conf_mutex_);
  if(msg->header.frame_id == "") {
    // This should be removed at some point
    logger->log_warn(name(), "Received initial pose with empty frame_id. "
		     "You should always supply a frame_id.");
  } else if (tf_listener->resolve(msg->header.frame_id) != tf_listener->resolve(global_frame_id_))
  {
    // We only accept initial pose estimates in the global frame, #5148.
    logger->log_warn(name(),"Ignoring initial pose in frame \"%s\"; "
		     "initial poses must be in the global frame, \"%s\"",
		     msg->header.frame_id.c_str(),
		     global_frame_id_.c_str());
    return;
  }

  fawkes::Time now(clock);
  fawkes::Time msg_time(msg->header.stamp.sec,
			msg->header.stamp.nsec / 1000);
  
  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try {
    tf_listener->lookup_transform(base_frame_id_, now,
				  base_frame_id_, msg_time,
				  global_frame_id_, tx_odom);
  } catch(tf::TransformException &e) {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      logger->log_warn(name(), "Failed to transform initial pose "
		       "in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  //tf::poseMsgToTF(msg->pose.pose, pose_old);
  pose_old =
    tf::Transform(tf::Quaternion(msg->pose.pose.orientation.x,
				 msg->pose.pose.orientation.y,
				 msg->pose.pose.orientation.z,
				 msg->pose.pose.orientation.w), 
		  tf::Vector3(msg->pose.pose.position.x,
			      msg->pose.pose.position.y,
			      msg->pose.pose.position.z));
  pose_new = tx_odom.inverse() * pose_old;

  // Transform into the global frame

  logger->log_info(name(), "Setting pose (%s): %.3f %.3f %.3f",
		   now.str(),
		   pose_new.getOrigin().x(),
		   pose_new.getOrigin().y(),
		   tf::get_yaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf::get_yaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++) {
    for(int j=0; j<2; j++) {
      pf_init_pose_cov.m[i][j] = msg->pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg->pose.covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  apply_initial_pose();
}
#endif
