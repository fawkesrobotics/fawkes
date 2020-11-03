/***************************************************************************
 *  gologpp_fawkes_backend.cpp - Fawkes backend for Golog++
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
 *                   Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "gologpp_fawkes_backend.h"

#include "message_action_executor.h"
#include "print_action_executor.h"
#include "remote_skiller_executor.h"
#include "skiller_action_executor.h"
#include "sleep_action_executor.h"

#include <config/config.h>
#include <golog++/execution/activity.h>
#include <golog++/execution/transition.h>

namespace fawkes {
namespace gpp {

using namespace gologpp;

/** @class GologppFawkesBackend
 *  A Golog++ backend to get data from and send commands to Fawkes.
 *  The backend currently only provides access to the skiller for action
 *  execution.
 */

/** Constructor.
 *  @param config The configuration to read from
 *  @param cfg_prefix The spec-specific config prefix to use
 *  @param logger The logger to use for log messages
 *  @param blackboard The blackboard to use to access the skiller
 *  @param tf_listener Used for some exog_function implementations
 */
GologppFawkesBackend::GologppFawkesBackend(Configuration *  config,
                                           std::string      cfg_prefix,
                                           Logger *         logger,
                                           BlackBoard *     blackboard,
                                           tf::Transformer *tf_listener)
: AspectProviderAspect(&dispatcher_inifin_),
  logger_(logger),
  blackboard_(blackboard),
  tf_listener_(tf_listener),
  config_(config)
{
	// Register RemoteSkillerActionExecutors before the local
	// SkillerActionExecutor. This way, any action that cannot be executed on any
	// remote will be tried locally.
	for (const string &robot :
	     config->get_strings_or_defaults((cfg_prefix + "/agents/names").c_str(), {})) {
		const std::string  agent_prefix = cfg_prefix + "/agents/" + robot;
		const std::string &hostname =
		  config->get_string_or_default((agent_prefix + "/host").c_str(), "localhost");
		const unsigned short int &port =
		  config->get_uint_or_default((agent_prefix + "/port").c_str(), 1910);
		action_dispatcher_.register_executor(std::make_shared<RemoteSkillerActionExecutor>(
		  logger, "robot", robot, hostname, port, config, cfg_prefix));
	}
	if (config->get_bool_or_default((cfg_prefix + "/use_local_skiller").c_str(), true)) {
		action_dispatcher_.register_executor(
		  std::make_shared<SkillerActionExecutor>(logger, blackboard, config, cfg_prefix));
	}
	action_dispatcher_.register_executor(
	  std::make_shared<BBMessageActionExecutor>(logger, blackboard, config, cfg_prefix));
	action_dispatcher_.register_executor(std::make_shared<SleepActionExecutor>(logger));
	action_dispatcher_.register_executor(std::make_shared<PrintActionExecutor>(logger));
}

GologppFawkesBackend::~GologppFawkesBackend()
{
}

/** Preempt the currently running activity.
 *  Determine the right executor and instruct the executor to stop the activity.
 *  @param a The activity to stop
 */
void
GologppFawkesBackend::preempt_activity(shared_ptr<Activity> a)
{
	auto executor = action_dispatcher_.get_executor(a);
	executor->stop(a);
}

/** Get the current time from Fawkes.
 *  @return the current time
 */
gologpp::Clock::time_point
GologppFawkesBackend::time() const noexcept
{
	return gologpp::Clock::time_point{
	  gologpp::Clock::duration{clock->now().in_sec() / gologpp::Clock::duration::period::den}};
}

Value
GologppFawkesBackend::eval_exog_function(const Type &                                  ret_type,
                                         const std::string &                           name,
                                         const std::unordered_map<std::string, Value> &args)
{
	const NumberType &number_type = get_type<NumberType>();

	if (name == "fawkes_transform_pose") {
		const CompoundType &pose_type = dynamic_cast<const CompoundType &>(ret_type);

		string to_frame = static_cast<string>(args.at("to_frame"));

		const auto &  gpp_pose   = static_cast<CompoundType::Representation>(args.at("src"));
		const auto &  gpp_trans  = static_cast<CompoundType::Representation>(*gpp_pose.at("trans"));
		const auto &  gpp_rot    = static_cast<CompoundType::Representation>(*gpp_pose.at("rot"));
		const string &from_frame = static_cast<string>(*gpp_pose.at("frame"));

		tf::Pose pose_in({gpp_rot.at("x")->numeric_convert<tf::Scalar>(),
		                  gpp_rot.at("y")->numeric_convert<tf::Scalar>(),
		                  gpp_rot.at("z")->numeric_convert<tf::Scalar>(),
		                  gpp_rot.at("w")->numeric_convert<tf::Scalar>()},
		                 {gpp_trans.at("x")->numeric_convert<tf::Scalar>(),
		                  gpp_trans.at("y")->numeric_convert<tf::Scalar>(),
		                  gpp_trans.at("z")->numeric_convert<tf::Scalar>()});

		tf::Stamped<tf::Pose> out;
		tf_listener_->transform_pose(to_frame,
		                            tf::Stamped<tf::Pose>(pose_in, Time(0, 0), from_frame),
		                            out);

		CompoundType::Representation trans_raw;
		trans_raw.emplace("x", new Value(number_type, out.getOrigin().getX()));
		trans_raw.emplace("y", new Value(number_type, out.getOrigin().getY()));
		trans_raw.emplace("z", new Value(number_type, out.getOrigin().getZ()));

		CompoundType::Representation rot_raw;
		rot_raw.emplace("x", new Value(number_type, out.getRotation().getX()));
		rot_raw.emplace("y", new Value(number_type, out.getRotation().getY()));
		rot_raw.emplace("z", new Value(number_type, out.getRotation().getZ()));
		rot_raw.emplace("w", new Value(number_type, out.getRotation().getW()));

		CompoundType::Representation pose_raw;
		pose_raw.emplace("trans", new Value(pose_type.field_type("trans"), std::move(trans_raw)));
		pose_raw.emplace("rot", new Value(pose_type.field_type("rot"), std::move(rot_raw)));
		pose_raw.emplace("frame", new Value(get_type<StringType>(), to_frame));

		return Value(pose_type, pose_raw);

	} else if (name == "fawkes_rpy_to_quaternion") {
		const CompoundType &quaternion_type = dynamic_cast<const CompoundType &>(ret_type);
		tf::Quaternion q = tf::create_quaternion_from_rpy(args.at("roll").numeric_convert<double>(),
		                                                  args.at("pitch").numeric_convert<double>(),
		                                                  args.at("yaw").numeric_convert<double>());
		CompoundType::Representation rv_raw;
		rv_raw.emplace("x", new Value(number_type, q.getX()));
		rv_raw.emplace("y", new Value(number_type, q.getY()));
		rv_raw.emplace("z", new Value(number_type, q.getZ()));
		rv_raw.emplace("w", new Value(number_type, q.getW()));

		return Value(quaternion_type, rv_raw);

	} else if (name == "fawkes_quaternion_to_rpy") {
		const ListType &list_type = dynamic_cast<const ListType &>(ret_type);

		CompoundType::Representation quat_in =
		  static_cast<CompoundType::Representation>(args.at("quaternion"));

		tf::Quaternion q(quat_in.at("x")->numeric_convert<tf::Scalar>(),
		                 quat_in.at("y")->numeric_convert<tf::Scalar>(),
		                 quat_in.at("z")->numeric_convert<tf::Scalar>(),
		                 quat_in.at("w")->numeric_convert<tf::Scalar>());

		tf::Scalar roll, pitch, yaw;
		q.getEulerZYX(yaw, pitch, roll);
		ListType::Representation list_raw;
		list_raw.emplace_back(new Value(number_type, roll));
		list_raw.emplace_back(new Value(number_type, pitch));
		list_raw.emplace_back(new Value(number_type, yaw));

		return Value(list_type, list_raw);

	} else if (name == "fawkes_config_get_float") {
		(void)dynamic_cast<const NumberType &>(ret_type);
		return Value(number_type,
		             config_->get_float_or_default(static_cast<std::string>(args.at("path")).c_str(),
		                                          args.at("default").numeric_convert<float>()));
	} else
		throw gologpp::UserError("No such exog_function: " + name);
}

/** Terminate execution of running actions
 */
void
GologppFawkesBackend::terminate_()
{
	action_dispatcher_.terminate();
}

/** Execute the given activity using a suitable executor.
 *  @param a The activity to start.
 */
void
GologppFawkesBackend::execute_activity(shared_ptr<Activity> a)
{
	auto executor = action_dispatcher_.get_executor(a);
	executor->start(a);
}

} // namespace gpp
} // namespace fawkes
