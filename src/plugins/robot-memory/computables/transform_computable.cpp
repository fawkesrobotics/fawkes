/***************************************************************************
 *  transform_computable.cpp - Computable for doing transforms
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
using namespace mongocxx;
using namespace bsoncxx;

/** @class TransformComputable  transform_computable.h
 * Computable proving positions in other frames by using transforms
 * @author Frederik Zwilling
 */

/** Constructor for Transform computable with objects of thread aspects.
 * @param robot_memory Robot Memory
 * @param tf Transform
 * @param logger Logger
 * @param config Configuration
 */
TransformComputable::TransformComputable(RobotMemory *            robot_memory,
                                         fawkes::tf::Transformer *tf,
                                         fawkes::Logger *         logger,
                                         fawkes::Configuration *  config)
{
	robot_memory_ = robot_memory;
	tf_           = tf;
	logger_       = logger;
	config_       = config;

	//register computable
	document::value          query = from_json("{frame:{$exists:true},allow_tf:true}");
	std::vector<std::string> collections =
	  config->get_strings("plugins/robot-memory/computables/transform/collections");
	int   priority     = config->get_int("plugins/robot-memory/computables/transform/priority");
	float caching_time = config->get_float("plugins/robot-memory/computables/transform/caching-time");
	for (std::string col : collections) {
		computables.push_back(
		  robot_memory_->register_computable(std::move(query),
		                                     col,
		                                     &TransformComputable::compute_transform,
		                                     this,
		                                     caching_time,
		                                     priority));
	}
}

TransformComputable::~TransformComputable()
{
	for (Computable *comp : computables) {
		robot_memory_->remove_computable(comp);
	}
}

std::list<document::value>
TransformComputable::compute_transform(const document::view &query, const std::string &collection)
{
	//get positions in other frames
	using namespace bsoncxx::builder;
	basic::document query_other_frames;
	for (auto it = query.begin(); it != query.end(); it++) {
		if (it->key() == "frame" || it->key() == "allow_tf") {
			continue;
		}
		query_other_frames.append(basic::kvp(it->key(), it->get_value()));
	}
	query_other_frames.append(basic::kvp("frame", [](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("$exists", true));
	}));
	cursor cur = robot_memory_->query(query_other_frames, collection);

	//transform them is possible
	std::list<document::value> res;
	std::string                target_frame = query["frame"].get_utf8().value.to_string();
	auto                       it           = cur.begin();
	while (it != cur.end()) {
		document::view pos = *it;
		if (pos.find("frame") != pos.end() && pos.find("translation") != pos.end()
		    && pos.find("rotation") != pos.end()) {
			std::string src_frame = pos["frame"].get_utf8().value.to_string();
			Time        now(0, 0);
			if (tf_->can_transform(target_frame.c_str(), src_frame.c_str(), now)) {
				basic::document       res_pos;
				array::view           src_trans = pos["translation"].get_array();
				array::view           src_rot   = pos["rotation"].get_array();
				fawkes::tf::Transform pose_tf(fawkes::tf::Quaternion(src_rot[0].get_double(),
				                                                     src_rot[1].get_double(),
				                                                     src_rot[2].get_double(),
				                                                     src_rot[3].get_double()),
				                              fawkes::tf::Vector3(src_trans[0].get_double(),
				                                                  src_trans[1].get_double(),
				                                                  src_trans[2].get_double()));
				fawkes::tf::Stamped<fawkes::tf::Pose> src_stamped_pose(pose_tf,
				                                                       Time(0, 0),
				                                                       src_frame.c_str());
				fawkes::tf::Stamped<fawkes::tf::Pose> res_stamped_pose;
				tf_->transform_pose(target_frame.c_str(), src_stamped_pose, res_stamped_pose);

				for (auto it = pos.begin(); it != pos.end(); it++) {
					if (!(it->key() == "frame" || it->key() == "translation" || it->key() == "rotation"
					      || it->key() == "_id")) {
						res_pos.append(basic::kvp(it->key(), it->get_value()));
					}
				}
				res_pos.append(basic::kvp("frame", target_frame));
				res_pos.append(basic::kvp("allow_tf", true));
				res_pos.append(basic::kvp("translation", [res_stamped_pose](basic::sub_array array) {
					array.append(res_stamped_pose.getOrigin().x());
					array.append(res_stamped_pose.getOrigin().y());
					array.append(res_stamped_pose.getOrigin().z());
				}));
				res_pos.append(basic::kvp("rotation", [res_stamped_pose](basic::sub_array array) {
					array.append(res_stamped_pose.getRotation().x());
					array.append(res_stamped_pose.getRotation().y());
					array.append(res_stamped_pose.getRotation().z());
					array.append(res_stamped_pose.getRotation().w());
				}));
				res.push_back(res_pos.extract());
			}
			//      else
			//      {
			//        logger_->log_info(name_, "Cant transform %s to %s", src_frame.c_str(), target_frame.c_str());
			//      }
		}
	}
	return res;
}
