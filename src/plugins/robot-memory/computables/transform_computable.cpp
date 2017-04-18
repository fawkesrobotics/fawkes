/***************************************************************************
 *  transform_computable.cpp - Computable for doing transforms
 *    
 *
 *  Created: 4:11:27 PM 2016
 *  Copyright  2016  Frederik Zwilling
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

#include "transform_computable.h"

using namespace fawkes;
using namespace mongo;

/** @class TransformComputable  transform_computable.h
 * Computable proving positions in other frames by using transforms
 * @author Frederik Zwilling
 */

/**
 * Constructor for Transform computable with objects of thread aspects
 * @param robot_memory Robot Memory
 * @param tf Transform
 * @param logger Logger
 * @param config Configuration
 */
TransformComputable::TransformComputable(RobotMemory* robot_memory, fawkes::tf::Transformer* tf, fawkes::Logger* logger, fawkes::Configuration* config)
{
  robot_memory_ = robot_memory;
  tf_ = tf;
  logger_ = logger;
  config_ = config;

  //register computable
  Query query = fromjson("{frame:{$exists:true},allow_tf:true}");
  std::vector<std::string>  collections = config->get_strings("plugins/robot-memory/computables/transform/collections");
  int priority = config->get_int("plugins/robot-memory/computables/transform/priority");
  float caching_time = config->get_float("plugins/robot-memory/computables/transform/caching-time");
  for(std::string col : collections)
  {
    computables.push_back(robot_memory_->register_computable(query, col, &TransformComputable::compute_transform, this, caching_time, priority));
  }
}

TransformComputable::~TransformComputable()
{
  for(Computable* comp : computables)
  {
    robot_memory_->remove_computable(comp);
  }
}

std::list<mongo::BSONObj> TransformComputable::compute_transform(mongo::BSONObj query, std::string collection)
{
  //get positions in other frames
  BSONObjBuilder query_other_frames;
  query_other_frames.appendElements(query.removeField("frame").removeField("allow_tf"));
  query_other_frames.append("frame", fromjson("{$exists:true}"));
  QResCursor cur = robot_memory_->query(query_other_frames.obj(), collection);

  //transform them is possible
  std::list<mongo::BSONObj> res;
  std::string target_frame = query.getField("frame").String();
  while(cur->more())
  {
    BSONObj pos = cur->next();
    if(pos.hasField("frame") && pos.hasField("translation") && pos.hasField("rotation"))
    {
      std::string src_frame = pos.getField("frame").String();
      Time now(0, 0);
      if(tf_->can_transform(target_frame.c_str(), src_frame.c_str(), now))
      {
        BSONObjBuilder res_pos;
        std::vector<BSONElement> src_trans = pos.getField("translation").Array();
        std::vector<BSONElement> src_rot = pos.getField("rotation").Array();
        fawkes::tf::Transform pose_tf(fawkes::tf::Quaternion(src_rot[0].Double(), src_rot[1].Double(), src_rot[2].Double(), src_rot[3].Double()),
          fawkes::tf::Vector3(src_trans[0].Double(), src_trans[1].Double(), src_trans[2].Double()));
        fawkes::tf::Stamped<fawkes::tf::Pose> src_stamped_pose(pose_tf, Time(0, 0), src_frame.c_str());
        fawkes::tf::Stamped<fawkes::tf::Pose> res_stamped_pose;
        tf_->transform_pose(target_frame.c_str(), src_stamped_pose, res_stamped_pose);

        res_pos.appendElements(pos.removeField("frame").removeField("translation").removeField("rotation").removeField("_id"));
        res_pos.append("frame", target_frame);
        res_pos.append("allow_tf", true);
        BSONArrayBuilder arrb_trans;
        arrb_trans.append(res_stamped_pose.getOrigin().x());
        arrb_trans.append(res_stamped_pose.getOrigin().y());
        arrb_trans.append(res_stamped_pose.getOrigin().z());
        res_pos.append("translation", arrb_trans.arr());
        BSONArrayBuilder arrb_rot;
        arrb_rot.append(res_stamped_pose.getRotation().x());
        arrb_rot.append(res_stamped_pose.getRotation().y());
        arrb_rot.append(res_stamped_pose.getRotation().z());
        arrb_rot.append(res_stamped_pose.getRotation().w());
        res_pos.append("rotation", arrb_rot.arr());
        res.push_back(res_pos.obj());
      }
//      else
//      {
//        logger_->log_info(name_, "Cant transform %s to %s", src_frame.c_str(), target_frame.c_str());
//      }
    }
  }
  return res;
}
