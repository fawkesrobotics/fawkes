
/***************************************************************************
 *  clips-rest-api.cpp -  CLIPS REST API
 *
 *  Created: Sat Mar 31 01:36:11 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "clips-rest-api.h"

#include <webview/rest_api_manager.h>
#include <core/threading/mutex_locker.h>

#include <clips/clips.h>

using namespace fawkes;

/** @class ClipsRestApi "clips-rest-api.h"
 * REST API backend for CLIPS.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsRestApi::ClipsRestApi()
	: Thread("ClipsRestApi", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
ClipsRestApi::~ClipsRestApi()
{
}

void
ClipsRestApi::init()
{
	rest_api_ = new WebviewRestApi("clips", logger);
	rest_api_->add_handler<WebviewRestArray<Fact>>
		(WebRequest::METHOD_GET, "/{env}/facts",
		 std::bind(&ClipsRestApi::cb_get_facts, this, std::placeholders::_1));
	rest_api_->add_handler<WebviewRestArray<Environment>>
		(WebRequest::METHOD_GET, "/",
		 std::bind(&ClipsRestApi::cb_list_environments, this));
	webview_rest_api_manager->register_api(rest_api_);
}


void
ClipsRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
}


void
ClipsRestApi::loop()
{
}

/** Get a value from a fact.
 * @param fact pointer to CLIPS fact
 * @param slot_name name of field to retrieve
 * @return template-specific return value
 */
// template <typename T>
// T get_value(const CLIPS::Fact::pointer &fact, const std::string &slot_name)
// {
// 	CLIPS::Values v = fact->slot_value(slot_name);
// 	if (v.empty()) {
// 		throw Exception("No value for slot '%s'", slot_name.c_str());
// 	}
// 	if (v[0].type() == CLIPS::TYPE_SYMBOL && v[0].as_string() == "nil") {
// 		return T();
// 	}
// 	return v[0];
// }

/** Specialization for bool.
 * @param fact pointer to CLIPS fact
 * @param slot_name name of field to retrieve
 * @return boolean value
 */
// template <>
// bool get_value(const CLIPS::Fact::pointer &fact, const std::string &slot_name)
// {
// 	CLIPS::Values v = fact->slot_value(slot_name);
// 	if (v.empty()) {
// 		throw Exception("No value for slot '%s'", slot_name.c_str());
// 	}
// 	if (v[0].type() != CLIPS::TYPE_SYMBOL) {
// 		throw Exception("Value for slot '%s' is not a boolean", slot_name.c_str());
// 	}
// 	return (v[0].as_string() == "TRUE");
// }

/** Get value array.
 * This is not a template because the overly verbose operator API
 * of CLIPS::Value can lead to ambiguous overloads, e.g., resolving
 * std::string to std::string or const char * operators.
 * @param fact pointer to CLIPS fact
 * @param slot_name name of field to retrieve
 * @return vector of strings from multislot
 */
// static std::vector<std::string>
// get_values(const CLIPS::Fact::pointer &fact, const std::string &slot_name)
// {
// 	CLIPS::Values v = fact->slot_value(slot_name);
// 	std::vector<std::string> rv(v.size());
// 	for (size_t i = 0; i < v.size(); ++i) {
// 		rv[i] = static_cast<std::string&>(v[i]);
// 	}
// 	return rv;
// }

Fact
ClipsRestApi::gen_fact(LockPtr<CLIPS::Environment>& clips, CLIPS::Fact::pointer& fact, bool formatted)
{
	Fact retf;
	retf.set_kind("Fact");
	retf.set_apiVersion(Fact::api_version());
	retf.set_index(fact->index());
	CLIPS::Template::pointer fact_template = fact->get_template();
	if (fact_template) {
		retf.set_template_name(fact_template->name());
	} else {
		retf.set_template_name("implied");
	}

	if (formatted) {
		char tmp[16384];
		tmp[16383] = 0;
		OpenStringDestination(clips->cobj(), (char *)"ProcPPForm", tmp, 16383);
		PrintFact(clips->cobj(), (char *)"ProcPPForm",
		          (struct fact *)fact->cobj(), FALSE, FALSE);
		CloseStringDestination(clips->cobj(), (char *)"ProcPPForm");
		retf.set_formatted(tmp);
	} else {
		std::vector<std::string> slots = fact->slot_names();
		for (const auto &s : slots) {
			CLIPS::Values fval = fact->slot_value(s);
			SlotValue sval;
			sval.set_name(s);
			sval.set_is_multifield(fact_template ? fact_template->is_multifield_slot(s) : (fval.size() > 1));
			for (const auto &v : fval) {
				switch (v.type()) {
				case CLIPS::TYPE_FLOAT:
					sval.addto_values(std::to_string(v.as_float())); break;
				case CLIPS::TYPE_INTEGER:
					sval.addto_values(std::to_string(v.as_integer())); break;
				case CLIPS::TYPE_SYMBOL:
				case CLIPS::TYPE_STRING:
				case CLIPS::TYPE_INSTANCE_NAME:
					sval.addto_values(v.as_string()); break;
				default:
					sval.addto_values("ADDR"); break;
				}
			}
			retf.addto_slots(std::move(sval));
		}
	}

	return retf;
}


WebviewRestArray<Fact>
ClipsRestApi::cb_get_facts(WebviewRestParams& params)
{
	bool formatted = (params.query_arg("formatted") == "true");

	WebviewRestArray<Fact> rv;

	MutexLocker lock(clips_env_mgr.objmutex_ptr());
	std::map<std::string, LockPtr<CLIPS::Environment>> envs =
		clips_env_mgr->environments();
	if (envs.find(params.path_arg("env")) == envs.end()) {
		throw WebviewRestException(WebReply::HTTP_NOT_FOUND, "Environment '%s' is unknown",
		                           params.path_arg("env").c_str());
	}

	auto clips = envs[params.path_arg("env")];
	MutexLocker clips_lock(clips.objmutex_ptr());

	CLIPS::Fact::pointer fact = clips->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		rv.push_back(std::move(gen_fact(clips, fact, formatted)));
		fact = fact->next();
	}

	return rv;
}


WebviewRestArray<Environment>
ClipsRestApi::cb_list_environments()
{
	WebviewRestArray<Environment> rv;

	MutexLocker lock(clips_env_mgr.objmutex_ptr());
	std::map<std::string, LockPtr<CLIPS::Environment>> envs =
		clips_env_mgr->environments();

	for (const auto &e : envs) {
		Environment env;
		env.set_kind("Environment");
		env.set_apiVersion(Environment::api_version());
		env.set_name(e.first);
		rv.push_back(std::move(env));
	}

	return rv;
}
