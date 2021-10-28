
/***************************************************************************
 *  dynamic_reconfigure_thread.cpp - Robotino ROS Dynamic Reconfigure Thread
 *
 *  Created: Fri May 05 20:07:27 2017
 *  Copyright  2017  Christoph Henke
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

#include "dynamic_reconfigure_thread.h"

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/StrParameter.h>

using namespace fawkes;

/** @class ROS2DynamicReconfigureThread "dynamic_reconfigure_thread.h"
 * Performing dynamic reconfiguration between Fawkes and ROS
 * @author Christoph Henke
 */

/** Contructor. */
ROS2DynamicReconfigureThread::ROS2DynamicReconfigureThread()
: Thread("ROS2DynamicReconfigureThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

/** Method for initialization
 */
void
ROS2DynamicReconfigureThread::init()
{
	try {
		dynrec_if_ = blackboard->open_for_writing<DynamicReconfigureInterface>("DynamicReconfigure");
	} catch (Exception &e) {
		e.append("%s initialization failed, could not open dynamic reconfigure "
		         "interface for writing",
		         name());
		logger->log_error(name(), e);
		throw;
	}

	// Initialize interface
	dynrec_if_->set_last_service("");
	dynrec_if_->set_last_parameter("");
	dynrec_if_->set_last_msg_id(0);
	dynrec_if_->set_last_bool_value(false);
	dynrec_if_->set_last_str_value("");
	dynrec_if_->set_last_uint32_value(0);
	dynrec_if_->set_last_uint64_value(0);
	dynrec_if_->set_last_float_value(0.0);
	dynrec_if_->write();
}

/** Method for finalization
 */
void
ROS2DynamicReconfigureThread::finalize()
{
	// close interfaces
	try {
		blackboard->close(dynrec_if_);
	} catch (Exception &e) {
		logger->log_error(name(), "Closing interface failed!");
		logger->log_error(name(), e);
	}
}

/** Method for setting an dynamic reconfigure of type bool
 * @param service the service to call for dynamic reconfiguration
 * @param path path of the ROS parameter
 * @param value value for the ROS parameter
 */
bool
ROS2DynamicReconfigureThread::set_dynreconf_value(const std::string &service,
                                                 const std::string &parameter,
                                                 const bool         value)
{
	dynamic_reconfigure::BoolParameter       dynreconf_bool_param;
	dynamic_reconfigure::Config              dynreconf_conf;
	dynamic_reconfigure::ReconfigureRequest  dynreconf_srv_req;
	dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;

	dynreconf_bool_param.name  = parameter;
	dynreconf_bool_param.value = value;
	dynreconf_conf.bools.push_back(dynreconf_bool_param);
	dynreconf_srv_req.config = dynreconf_conf;

	if (!ros::service::exists(service, false)) {
		logger->log_error(name(),
		                  "Error in setting dynreconf value %s to %s - "
		                  "service %s does not exists",
		                  parameter.c_str(),
		                  value ? "true" : "false",
		                  service.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Failed);
		dynrec_if_->write();
		dynreconf_conf.bools.clear();
		return false;
	} else if (!ros::service::call(service, dynreconf_srv_req, dynreconf_srv_resp)) {
		logger->log_error(name(),
		                  "Error in setting dynreconf value %s to %s with service %s",
		                  parameter.c_str(),
		                  value ? "true" : "false",
		                  service.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Failed);
		dynrec_if_->write();
		dynreconf_conf.bools.clear();
		return false;
	} else {
		logger->log_info(name(), "Setting %s to %s", parameter.c_str(), value ? "true" : "false");
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Succeeded);
		dynrec_if_->write();
		dynreconf_conf.bools.clear();
		return true;
	}
}

/** Method for setting an dynamic reconfigure of type string
 * @param service the service to call for dynamic reconfiguration
 * @param path path of the ROS parameter
 * @param value value for the ROS parameter
 */
bool
ROS2DynamicReconfigureThread::set_dynreconf_value(const std::string &service,
                                                 const std::string &parameter,
                                                 const std::string &value)
{
	dynamic_reconfigure::StrParameter        dynreconf_str_param;
	dynamic_reconfigure::Config              dynreconf_conf;
	dynamic_reconfigure::ReconfigureRequest  dynreconf_srv_req;
	dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;

	dynreconf_str_param.name  = parameter;
	dynreconf_str_param.value = value;
	dynreconf_conf.strs.push_back(dynreconf_str_param);
	dynreconf_srv_req.config = dynreconf_conf;

	if (!ros::service::exists(service, false)) {
		logger->log_error(name(),
		                  "Error in setting dynreconf value %s to %s - "
		                  "service %s does not exists",
		                  parameter.c_str(),
		                  value.c_str(),
		                  service.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Failed);
		dynrec_if_->write();
		dynreconf_conf.strs.clear();
		return false;
	} else if (!ros::service::call(service, dynreconf_srv_req, dynreconf_srv_resp)) {
		logger->log_error(name(),
		                  "Error in setting dynreconf value %s to %s with service %s",
		                  parameter.c_str(),
		                  value.c_str(),
		                  service.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Failed);
		dynrec_if_->write();
		dynreconf_conf.strs.clear();
		return false;
	} else {
		logger->log_info(name(), "Setting %s to %s", parameter.c_str(), value.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Succeeded);
		dynrec_if_->write();
		dynreconf_conf.strs.clear();
		return true;
	}
}

/** Method for setting an dynamic reconfigure of type int
 * @param service the service to call for dynamic reconfiguration
 * @param path path of the ROS parameter
 * @param value value for the ROS parameter
 */
bool
ROS2DynamicReconfigureThread::set_dynreconf_value(const std::string &service,
                                                 const std::string &parameter,
                                                 const int          value)
{
	dynamic_reconfigure::IntParameter        dynreconf_int_param;
	dynamic_reconfigure::Config              dynreconf_conf;
	dynamic_reconfigure::ReconfigureRequest  dynreconf_srv_req;
	dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;

	dynreconf_int_param.name  = parameter;
	dynreconf_int_param.value = value;
	dynreconf_conf.ints.push_back(dynreconf_int_param);
	dynreconf_srv_req.config = dynreconf_conf;

	if (!ros::service::exists(service, false)) {
		logger->log_error(name(),
		                  "Error in setting dynreconf value %s to %d - "
		                  "service %s does not exists",
		                  parameter.c_str(),
		                  value,
		                  service.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Failed);
		dynrec_if_->write();
		dynreconf_conf.ints.clear();
		return false;
	} else if (!ros::service::call(service, dynreconf_srv_req, dynreconf_srv_resp)) {
		logger->log_error(name(),
		                  "Error in setting dynreconf value %s to %d with service %s",
		                  parameter.c_str(),
		                  value,
		                  service.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Failed);
		dynrec_if_->write();
		dynreconf_conf.ints.clear();
		return false;
	} else {
		logger->log_info(name(), "Setting %s to %d", parameter.c_str(), value);
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Succeeded);
		dynrec_if_->write();
		dynreconf_conf.ints.clear();
		return true;
	}
}

/** Method for setting an dynamic reconfigure of type float
 * @param service the service to call for dynamic reconfiguration
 * @param path path of the ROS parameter
 * @param value value for the ROS parameter
 */
bool
ROS2DynamicReconfigureThread::set_dynreconf_value(const std::string &service,
                                                 const std::string &parameter,
                                                 const double       value)
{
	dynamic_reconfigure::DoubleParameter     dynreconf_double_param;
	dynamic_reconfigure::Config              dynreconf_conf;
	dynamic_reconfigure::ReconfigureRequest  dynreconf_srv_req;
	dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;

	dynreconf_double_param.name  = parameter;
	dynreconf_double_param.value = value;
	dynreconf_conf.doubles.push_back(dynreconf_double_param);
	dynreconf_srv_req.config = dynreconf_conf;

	if (!ros::service::exists(service, false)) {
		logger->log_error(name(),
		                  "Error in setting dynreconf value %s to %f - "
		                  "service %s does not exists",
		                  parameter.c_str(),
		                  value,
		                  service.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Failed);
		dynrec_if_->write();
		dynreconf_conf.doubles.clear();
		return false;
	} else if (!ros::service::call(service, dynreconf_srv_req, dynreconf_srv_resp)) {
		logger->log_error(name(),
		                  "Error in setting dynreconf value %s to %f with service %s",
		                  parameter.c_str(),
		                  value,
		                  service.c_str());
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Failed);
		dynrec_if_->write();
		dynreconf_conf.doubles.clear();
		return false;
	} else {
		logger->log_info(name(), "Setting %s to %f", parameter.c_str(), value);
		dynrec_if_->set_last_msg_status(fawkes::DynamicReconfigureInterface::LastMsgStatus::Succeeded);
		dynrec_if_->write();
		dynreconf_conf.doubles.clear();
		return true;
	}
}

/** Method for resetting all interface values
 */
void
ROS2DynamicReconfigureThread::reset_dynamic_reconfigure_interface()
{
	dynrec_if_->set_last_service("");
	dynrec_if_->set_last_parameter("");
	dynrec_if_->set_last_msg_id(0);
	dynrec_if_->set_last_bool_value(false);
	dynrec_if_->set_last_str_value("");
	dynrec_if_->set_last_uint32_value(0);
	dynrec_if_->set_last_uint64_value(0);
	dynrec_if_->set_last_float_value(0.0);
}

void
ROS2DynamicReconfigureThread::loop()
{
	while (!dynrec_if_->msgq_empty()) {
		if (DynamicReconfigureInterface::SetBoolMessage *msg = dynrec_if_->msgq_first_safe(msg)) {
			logger->log_info(name(), "Bool message received");
			reset_dynamic_reconfigure_interface();
			// Writing the last called service and the according parameters into the blackboard
			dynrec_if_->set_last_service(msg->service());
			dynrec_if_->set_last_parameter(msg->parameter());
			dynrec_if_->set_last_msg_id(msg->id());
			dynrec_if_->set_last_bool_value(msg->is_value());
			dynrec_if_->write();
			set_dynreconf_value(msg->service(), msg->parameter(), msg->is_value());
		} else if (DynamicReconfigureInterface::SetStringMessage *msg =
		             dynrec_if_->msgq_first_safe(msg)) {
			logger->log_info(name(), "String message received");
			reset_dynamic_reconfigure_interface();
			// Writing the last called service and the according parameters into the blackboard
			dynrec_if_->set_last_service(msg->service());
			dynrec_if_->set_last_parameter(msg->parameter());
			dynrec_if_->set_last_msg_id(msg->id());
			dynrec_if_->set_last_str_value(msg->value());
			dynrec_if_->write();
			set_dynreconf_value(msg->service(), msg->parameter(), msg->value());
		} else if (DynamicReconfigureInterface::SetUint32Message *msg =
		             dynrec_if_->msgq_first_safe(msg)) {
			logger->log_info(name(), "Uint32 message received");
			reset_dynamic_reconfigure_interface();
			// Writing the last called service and the according parameters into the blackboard
			dynrec_if_->set_last_service(msg->service());
			dynrec_if_->set_last_parameter(msg->parameter());
			dynrec_if_->set_last_msg_id(msg->id());
			dynrec_if_->set_last_uint32_value(msg->value());
			dynrec_if_->write();
			set_dynreconf_value(msg->service(), msg->parameter(), (int)msg->value());
		} else if (DynamicReconfigureInterface::SetUint64Message *msg =
		             dynrec_if_->msgq_first_safe(msg)) {
			logger->log_info(name(), "Uint64 message received");
			reset_dynamic_reconfigure_interface();
			// Writing the last called service and the according parameters into the blackboard
			dynrec_if_->set_last_service(msg->service());
			dynrec_if_->set_last_parameter(msg->parameter());
			dynrec_if_->set_last_msg_id(msg->id());
			dynrec_if_->set_last_uint64_value(msg->value());
			dynrec_if_->write();
			set_dynreconf_value(msg->service(), msg->parameter(), (int)msg->value());
		} else if (DynamicReconfigureInterface::SetFloatMessage *msg =
		             dynrec_if_->msgq_first_safe(msg)) {
			logger->log_info(name(), "Float message received");
			reset_dynamic_reconfigure_interface();
			// Writing the last called service and the according parameters into the blackboard
			dynrec_if_->set_last_service(msg->service());
			dynrec_if_->set_last_parameter(msg->parameter());
			dynrec_if_->set_last_msg_id(msg->id());
			dynrec_if_->set_last_float_value(msg->value());
			dynrec_if_->write();
			set_dynreconf_value(msg->service(), msg->parameter(), (double)msg->value());
		}

		dynrec_if_->msgq_pop();
	} // while
}
