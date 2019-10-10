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

#include <core/exception.h>
#include <libs/interface/field_iterator.h>

using namespace fawkes;
using namespace gologpp;

namespace fawkes_gpp {

/** @class ExogManagerThread
 * Watch/observe blackboard interfaces according to the mappings
 * specified for exogenous actions in the agent program. The config
 * has to specify whether some mapped backend name is supposed to be
 * an interface ID or a pattern.
 */

/** The prefix to use for the config of the ExogManager. */
const string ExogManager::cfg_prefix{"/plugins/gologpp/blackboard"};

static Value *field_to_value(InterfaceFieldIterator &fi, unsigned int idx);

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
 * Construct an ExogManager thread and attach it to the main loop.
 * @param exec_thread The Golog++ ExecutionContext to use
 */
ExogManager::ExogManager(GologppThread *exec_thread,
                         Configuration *config,
                         BlackBoard *   blackboard,
                         Logger *       logger)
: exec_thread_(exec_thread), config_(config), blackboard_(blackboard), logger_(logger)
{
	// We know that lists of elementary types will most likely occur, so simply
	// define the types unconditionally. The elementary types themselves are
	// already defined.
	global_scope().register_type(new ListType(*global_scope().lookup_type(BoolType::name())));
	global_scope().register_type(new ListType(*global_scope().lookup_type(NumberType::name())));
	global_scope().register_type(new ListType(*global_scope().lookup_type(SymbolType::name())));

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
	std::unique_ptr<Configuration::ValueIterator> watch_it(config->search(cfg_prefix + "/watch"));
	while (watch_it->next()) {
		if (cfg_prefix + "/watch/id" == watch_it->path()) {
			string                 id   = config->get_string(watch_it->path());
			shared_ptr<ExogAction> exog = find_mapped_exog(id);
			watchers_.push_back(std::make_unique<InterfaceWatcher>(blackboard, id, exog, *this));
		} else
			logger->log_error(name(), "Unexpected config entry %s", watch_it->path());
	}

	std::unique_ptr<Configuration::ValueIterator> observe_it(config->search(cfg_prefix + "/observe"));
	while (observe_it->next()) {
		if (cfg_prefix + "/observe/pattern" == observe_it->path()) {
			string                 id   = config->get_string(observe_it->path());
			shared_ptr<ExogAction> exog = find_mapped_exog(id);
			observers_.push_back(std::make_unique<PatternObserver>(blackboard, id, exog, *this));
		} else
			logger->log_error(name(), "Unexpected config entry %s", watch_it->path());
	}
}

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

ExogManager::BlackboardEventHandler::BlackboardEventHandler(fawkes::BlackBoard *   bb,
                                                            shared_ptr<ExogAction> exog,
                                                            ExogManager &          exog_mgr)
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
		auto  param_it =
		  std::find(target_exog_->params().begin(), target_exog_->params().end(), var_ref.target());
		auto param_idx = target_exog_->params().end() - param_it;
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
	InterfaceFieldIterator    fi = iface->fields();
	vector<unique_ptr<Value>> args;
	args.resize(target_exog_->arity());
	while (fi != iface->fields_end()) {
		if (target_exog_->mapping().is_mapped(fi.get_name())) {
			if (fi.get_length() > 0) {
				vector<unique_ptr<Value>> list_init;
				for (unsigned int idx = 0; idx < fi.get_length(); ++idx)
					list_init.emplace_back(field_to_value(fi, idx));
				auto                   order_it = fields_order_.find(fi.get_name());
				shared_ptr<const Type> list_type =
				  global_scope().lookup_type("list[" + list_init[0]->type_name() + "]");
				args[order_it->second].reset(new Value(list_type->name(), list_init));
			}
		}
		++fi;
	}
	return std::make_shared<ExogEvent>(target_exog_, std::move(args));
}

void
ExogManager::InterfaceWatcher::bb_interface_data_changed(Interface *iface) throw()
{
	exog_manager_.exog_queue_push(make_exog_event(iface));
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
	Interface *iface = blackboard_->open_for_reading(type, id);
	exog_manager_.exog_queue_push(make_exog_event(iface));
	blackboard_->close(iface);
}

static Value *
field_to_value(InterfaceFieldIterator &fi, unsigned int idx)
{
	switch (fi.get_type()) {
	case IFT_BOOL: return new Value(BoolType::name(), fi.get_bool(idx));
	case IFT_BYTE: return new Value(NumberType::name(), fi.get_byte(idx));
	case IFT_ENUM: return new Value(SymbolType::name(), fi.get_enum_string(idx));
	case IFT_INT8: return new Value(NumberType::name(), fi.get_int8(idx));
	case IFT_FLOAT: return new Value(NumberType::name(), fi.get_float(idx));
	case IFT_INT16: return new Value(NumberType::name(), fi.get_int16(idx));
	case IFT_INT32: return new Value(NumberType::name(), fi.get_int32(idx));
	case IFT_INT64: return new Value(NumberType::name(), fi.get_int64(idx));
	case IFT_UINT8: return new Value(NumberType::name(), fi.get_uint8(idx));
	case IFT_DOUBLE: return new Value(NumberType::name(), fi.get_double(idx));
	case IFT_STRING: return new Value(StringType::name(), fi.get_string());
	case IFT_UINT16: return new Value(NumberType::name(), fi.get_uint16(idx));
	case IFT_UINT32: return new Value(NumberType::name(), fi.get_uint32(idx));
	case IFT_UINT64: return new Value(NumberType::name(), fi.get_uint64(idx));
	}
	throw fawkes::Exception("Unhandled interface field type");
}

} // namespace fawkes_gpp
