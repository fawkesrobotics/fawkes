/***************************************************************************
 *  exog_manager.cpp - Insert exog actions into Golog++
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
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

#include "exog_manager.h"

#include "execution_thread.h"
#include "utils.h"

#include <core/exception.h>
#include <golog++/model/grounding.h>
#include <libs/interface/field_iterator.h>

using namespace fawkes;
using namespace gologpp;

namespace fawkes {
namespace gpp {

const std::unordered_map<interface_fieldtype_t, std::string> ExogManager::iface_type_to_golog_type_{
  {IFT_BOOL, BoolType::static_name()},
  {IFT_BYTE, NumberType::static_name()},
  {IFT_ENUM, SymbolType::static_name()},
  {IFT_INT8, NumberType::static_name()},
  {IFT_FLOAT, NumberType::static_name()},
  {IFT_INT16, NumberType::static_name()},
  {IFT_INT32, NumberType::static_name()},
  {IFT_INT64, NumberType::static_name()},
  {IFT_UINT8, NumberType::static_name()},
  {IFT_DOUBLE, NumberType::static_name()},
  {IFT_STRING, StringType::static_name()},
  {IFT_UINT16, NumberType::static_name()},
  {IFT_UINT32, NumberType::static_name()},
  {IFT_UINT64, NumberType::static_name()}};

/** @class ExogManager
 * Watch/observe blackboard interfaces according to the mappings
 * specified for exogenous actions in the agent program. The config
 * has to specify whether some mapped backend name is supposed to be
 * an interface ID or a pattern.
 */

/** @class ConfigError
 * Thrown if the config is somehow inconsistent with the agent program.
 */

/** Construct a ConfigError.
 * @param msg A message describing the error.
 */
ConfigError::ConfigError(const std::string &msg) : Exception(msg.c_str())
{
}

/** Constructor.
 * Construct an ExogManager.
 * @param exec_thread The Golog++ ExecutionContext to use
 * @param config The Fawkes configuration to read config values from
 * @param cfg_prefix The spec-specific config prefix to use
 * @param blackboard The blackboard to use to read data from
 * @param logger A logger instance to use for logging messages
 */
ExogManager::ExogManager(GologppThread *    exec_thread,
                         Configuration *    config,
                         const std::string &cfg_prefix,
                         BlackBoard *       blackboard,
                         Logger *           logger)
: exec_thread_(exec_thread), config_(config), blackboard_(blackboard), logger_(logger)
{
	// Build a map to lookup all exog actions by their mapped name (i.e. the
	// interface ID or pattern)
	for (shared_ptr<Global> g : global_scope().globals()) {
		shared_ptr<ExogAction> exog = std::dynamic_pointer_cast<ExogAction>(g);
		if (exog)
			mapped_exogs_.emplace(exog->mapping().backend_name(), exog);
	}

	// Register an InterfaceWatcher and a PatternObserver for each
	// watched/observed interface (pattern). These also implement the event
	// handlers.
	for (const string &id :
	     config->get_strings_or_defaults((cfg_prefix + "/blackboard/watch").c_str(), {})) {
		shared_ptr<ExogAction> exog = find_mapped_exog(id);
		watchers_.push_back(std::make_unique<InterfaceWatcher>(blackboard, id, exog, *this));
	}

	for (const string &pattern :
	     config->get_strings_or_defaults((cfg_prefix + "/blackboard/observe").c_str(), {})) {
		shared_ptr<ExogAction> exog = find_mapped_exog(pattern);
		observers_.push_back(std::make_unique<PatternObserver>(blackboard, pattern, exog, *this));
	}
}

/** Get the ExogManager's thread name.
 * @return the thread name
 */
const char *
ExogManager::name()
{
	return "ExogManager";
}

shared_ptr<ExogAction>
ExogManager::find_mapped_exog(const std::string &mapped_name)
{
	auto map_it = mapped_exogs_.find(mapped_name);

	if (map_it == mapped_exogs_.end())
		throw ConfigError("No exogenous action handles " + mapped_name);

	return map_it->second;
}

void
ExogManager::exog_queue_push(shared_ptr<ExogEvent> evt)
{
	exec_thread_->gologpp_context().exog_queue_push(evt);
}

/** Construct an event handler.
 * @param bb The fawkes blackboard to listen to
 * @param exog The ExogAction to trigger on data change
 * @param exog_mgr The ExogManager to send the ExogAction to
 */
ExogManager::BlackboardEventHandler::BlackboardEventHandler(
  BlackBoard *                             bb,
  gologpp::shared_ptr<gologpp::ExogAction> exog,
  ExogManager &                            exog_mgr)
: blackboard_(bb), target_exog_(exog), exog_manager_(exog_mgr)
{
	for (const auto &pair : target_exog_->mapping().arg_mapping()) {
		/* TODO: This is not very nice. First we have to look up the index of the
		 * mapped parameter variable and then remember to put arg values at that
		 * index when an actual event happens. This is necessary because the
		 * ExogEvent (or rather the ReferenceBase) constructor accepts only vectors
		 * with positional arguments.
		 */

		auto &var_ref = dynamic_cast<Reference<Variable> &>(*pair.second);

		// Make sure argument types match interface
		string     iface_type = extract_type_name(target_exog_->mapping().backend_name());
		Interface *iface =
		  blackboard_->open_for_reading(iface_type.c_str(), "/gologpp_interface_analysis");
		InterfaceFieldIterator fi;
		for (fi = iface->fields(); fi != iface->fields_end(); ++fi) {
			if (pair.first == fi.get_name())
				break;
		}
		if (fi == iface->fields_end()) {
			throw ConfigError("Interface type " + iface_type + " does not have a field `" + pair.first
			                  + "'");
		}
		auto it = iface_type_to_golog_type_.find(fi.get_type());
		if (it == iface_type_to_golog_type_.end()) {
			throw Exception("Unhandled interface field type %s", fi.get_typename());
		}

		shared_ptr<const Type> desired_type = gologpp::global_scope().lookup_type(it->second);
		if (!desired_type)
			throw Exception("Type %s required for field %s is not defined in the golog++ code model",
			                it->second.c_str(),
			                pair.first.c_str());

		if (!(*desired_type <= gologpp::get_type<StringType>()
		      || *desired_type >= gologpp::get_type<StringType>())
		    && fi.get_length() > 1) {
			desired_type = gologpp::global_scope().lookup_list_type(*desired_type);
		}

		if (!(var_ref.type() <= *desired_type || var_ref.type() >= *desired_type)) {
			throw ConfigError(target_exog_->name() + "'s argument " + var_ref.target()->name() + " is a "
			                  + var_ref.type().name() + ", but the interface field requires "
			                  + desired_type->name());
		}
		blackboard_->close(iface);

		auto param_it =
		  std::find(target_exog_->params().begin(), target_exog_->params().end(), var_ref.target());
		auto param_idx = param_it - target_exog_->params().begin();
		fields_order_.emplace(pair.first, arity_t(param_idx));
	}
}

std::string
ExogManager::BlackboardEventHandler::extract_type_name(const std::string &iface_uid)
{
	auto idx = iface_uid.find("::");
	if (idx == std::string::npos)
		throw ConfigError(iface_uid + " is not an interface UID. Must be IFACE_TYPE::IFACE_ID.");
	return iface_uid.substr(0, idx);
}

std::string
ExogManager::BlackboardEventHandler::extract_id(const std::string &iface_uid)
{
	auto idx = iface_uid.find("::");
	if (idx == std::string::npos)
		throw ConfigError(iface_uid + " is not an interface UID. Must be IFACE_TYPE::IFACE_ID.");
	return iface_uid.substr(idx + 2);
}

ExogManager::InterfaceWatcher::InterfaceWatcher(BlackBoard *           bb,
                                                const string &         uid,
                                                shared_ptr<ExogAction> exog,
                                                ExogManager &          exog_mgr)
: BlackboardEventHandler(bb, exog, exog_mgr),
  BlackBoardInterfaceListener("gologpp_blackboard_manager"),
  iface_(blackboard_->open_for_reading(extract_type_name(uid).c_str(), extract_id(uid).c_str()))
{
	bbil_add_data_interface(iface_);
	blackboard_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
}

ExogManager::InterfaceWatcher::~InterfaceWatcher()
{
	blackboard_->unregister_listener(this);
	blackboard_->close(iface_);
}

shared_ptr<ExogEvent>
ExogManager::BlackboardEventHandler::make_exog_event(Interface *iface) const
{
	// clang-format off
	// alignment of assignments just makes this unreadable
	iface->read();
	InterfaceFieldIterator fi = iface->fields();
	vector<unique_ptr<Value>> args(target_exog_->arity());

	while (fi != iface->fields_end()) {
		if (target_exog_->mapping().is_mapped(fi.get_name())) {
			auto order_it = fields_order_.find(fi.get_name());

			if (fi.get_length() == 1 || (fi.get_length() > 1 && fi.get_type() == IFT_STRING))
				args[order_it->second].reset(field_to_value(fi, 0));
			else if (fi.get_length() > 1) {
				vector<unique_ptr<Value>> list_init;
				for (unsigned int idx = 0; idx < fi.get_length(); ++idx)
					list_init.emplace_back(
						field_to_value(fi, idx)
					);
				shared_ptr<const Type> list_type = global_scope().lookup_list_type(list_init[0]->type());
				args[order_it->second].reset(
					new Value(*list_type, list_init)
				);
			}
			else
				throw IllegalArgumentException("%s: Field %s has length %d and type %s, which shouldn't happen",
			                                           iface->uid(), fi.get_name(), fi.get_length(), fi.get_typename());
		}
		++fi;
	}

	return std::make_shared<ExogEvent>(target_exog_, std::move(args));
	// clang-format on
}

void
ExogManager::InterfaceWatcher::bb_interface_data_changed(Interface *iface) throw()
{
	try {
		exog_manager_.exog_queue_push(make_exog_event(iface));
	} catch (IllegalArgumentException &e) {
		exog_manager_.logger_->log_error(exog_manager_.name(),
		                                 "Error when creating exogenous event: %s, ignoring event!",
		                                 e.what_no_backtrace());
	} catch (gologpp::UserError &e) {
		exog_manager_.logger_->log_error(exog_manager_.name(),
		                                 "Error when creating exogenous event: %s, ignoring event!",
		                                 e.what());
	}
}

ExogManager::PatternObserver::PatternObserver(BlackBoard *           bb,
                                              const std::string &    pattern,
                                              shared_ptr<ExogAction> exog,
                                              ExogManager &          exog_mgr)
: BlackboardEventHandler(bb, exog, exog_mgr)
{
	bbio_add_observed_create(extract_type_name(pattern).c_str(), extract_id(pattern).c_str());
	blackboard_->register_observer(this);
}

ExogManager::PatternObserver::~PatternObserver()
{
	blackboard_->unregister_observer(this);
}

void
ExogManager::PatternObserver::bb_interface_created(const char *type, const char *id) throw()
{
	std::lock_guard<std::mutex> locked{handler_mutex_};
	exog_manager_.watchers_.push_back(std::make_unique<InterfaceWatcher>(
	  blackboard_, string(type) + "::" + id, target_exog_, exog_manager_));
}

} // namespace gpp
} // namespace fawkes
