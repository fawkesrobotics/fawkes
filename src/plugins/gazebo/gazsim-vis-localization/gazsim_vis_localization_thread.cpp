/***************************************************************************
 *  gazsim_vis_localization_thread.h - Plugin visualizes the localization
 *
 *  Created: Tue Sep 17 15:38:34 2013
 *  Copyright  2013  Frederik Zwilling
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

#include "gazsim_vis_localization_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <cmath>
#include <utils/math/angle.h>

#include <interfaces/Position3DInterface.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class VisLocalizationThread "gazsim_localization_thread.h"
 * Thread simulates the Localization in Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
VisLocalizationThread::VisLocalizationThread()
  : Thread("VisLocalizationThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void VisLocalizationThread::init()
{
  logger->log_debug(name(), "Initializing Visualization of the Localization");

  //read config values
  update_rate_ = config->get_float("/gazsim/visualization/localization/update-rate");
  robot_name_ = config->get_string("/gazsim/robot-name");
  label_script_name_ = config->get_string("/gazsim/visualization/label-script-name");
  arrow_script_name_ = config->get_string("/gazsim/visualization/label-arrow-name");
  location_scripts_ = config->get_string("/gazsim/visualization/location-scripts");
  location_textures_ = config->get_string("/gazsim/visualization/location-textures");
  parent_name_ = config->get_string("/gazsim/visualization/localization/parent-name");
  label_size_ = config->get_float("/gazsim/visualization/localization/label-size");
  label_height_ = config->get_float("/gazsim/visualization/localization/label-height");

  last_update_time_ = clock->now().in_sec();

  //open interface
  pose_if_ = blackboard->open_for_reading<Position3DInterface>("Pose");

  //create publisher
  visual_publisher_ = gazebo_world_node->Advertise<gazebo::msgs::Visual>("~/visual", 5);
}

void VisLocalizationThread::finalize()
{
  blackboard->close(pose_if_);
}

void VisLocalizationThread::loop()
{
  //visualize the estimated position of the robot every few seconds
  fawkes::Time new_time = clock->now();
  double time_elapsed = new_time.in_sec() - last_update_time_.in_sec();
  if(time_elapsed > 1 / update_rate_)
  {
    last_update_time_ = new_time;

    //read pose
    pose_if_->read();
    double x = pose_if_->translation(0);
    double y = pose_if_->translation(1);
    //calculate ori from quaternion in interface
    double* quat = pose_if_->rotation();
    double ori = tf::get_yaw(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
    if(std::isnan(ori))
    {
      ori = 0.0;
    }

    //create label with number
    msgs::Visual msg_number;
    msg_number.set_name((robot_name_ + "-localization-label").c_str());
    msg_number.set_parent_name(parent_name_.c_str());
    msgs::Geometry *geomMsg = msg_number.mutable_geometry();
    geomMsg->set_type(msgs::Geometry::PLANE);
#if GAZEBO_MAJOR_VERSION > 5
    msgs::Set(geomMsg->mutable_plane()->mutable_normal(), ignition::math::Vector3d(0.0, 0.0, 1.0));
    msgs::Set(geomMsg->mutable_plane()->mutable_size(), ignition::math::Vector2d(label_size_, label_size_));
#else
    msgs::Set(geomMsg->mutable_plane()->mutable_normal(), math::Vector3(0.0, 0.0, 1.0));
    msgs::Set(geomMsg->mutable_plane()->mutable_size(), math::Vector2d(label_size_, label_size_));
    msg_number.set_transparency(0.2);  
#endif
    msg_number.set_cast_shadows(false);
#if GAZEBO_MAJOR_VERSION > 5
    msgs::Set(msg_number.mutable_pose(), ignition::math::Pose3d(x, y, label_height_, 0, 0, 0));
#else
    msgs::Set(msg_number.mutable_pose(), math::Pose(x, y, label_height_, 0, 0, 0));
#endif
    msgs::Material::Script* script = msg_number.mutable_material()->mutable_script();
    script->add_uri(location_scripts_.c_str());
    script->add_uri(location_textures_.c_str());
    script->set_name(label_script_name_.c_str());

    visual_publisher_->Publish(msg_number);

    //create label with direction arrow
#if GAZEBO_MAJOR_VERSION <= 5
    msgs::Visual msg_arrow;
    msg_arrow.set_name((robot_name_ + "-localization-arrow").c_str());
    msg_arrow.set_parent_name(parent_name_.c_str());
    msgs::Geometry *geomArrowMsg = msg_arrow.mutable_geometry();
    geomArrowMsg->set_type(msgs::Geometry::PLANE);
#if GAZEBO_MAJOR_VERSION > 5
    msgs::Set(geomArrowMsg->mutable_plane()->mutable_normal(), ignition::math::Vector3d(0.0, 0.0, 1.0));
    msgs::Set(geomArrowMsg->mutable_plane()->mutable_size(), ignition::math::Vector2d(0.17, 0.17));
#else
    msgs::Set(geomArrowMsg->mutable_plane()->mutable_normal(), math::Vector3(0.0, 0.0, 1.0));
    msgs::Set(geomArrowMsg->mutable_plane()->mutable_size(), math::Vector2d(0.17, 0.17));
    msg_arrow.set_transparency(0.4);  
#endif
    msg_arrow.set_cast_shadows(false);
#if GAZEBO_MAJOR_VERSION > 5
    msgs::Set(msg_arrow.mutable_pose(), ignition::math::Pose3d(x, y, label_height_ + 0.01, 0, 0, ori - /*turn image right*/ M_PI / 2));
#else
    msgs::Set(msg_arrow.mutable_pose(), math::Pose(x, y, label_height_ + 0.01, 0, 0, ori - /*turn image right*/ M_PI / 2));
#endif
    msgs::Material::Script* arrow_script = msg_arrow.mutable_material()->mutable_script();
    arrow_script->add_uri(location_scripts_.c_str());
    arrow_script->add_uri(location_textures_.c_str());
    arrow_script->set_name(arrow_script_name_.c_str());

    visual_publisher_->Publish(msg_arrow);
#endif
  }
}
