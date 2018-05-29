
/****************************************************************************
 *  Skill
 *  (auto-generated, do not modify directly)
 *
 *  Behavior Engine REST API.
 *  Visualize, monitor, and instruct the Skill Execution Run-Time of
 *  the Lua-based Behavior Engine.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "Skill.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

Skill::Skill()
{
}

Skill::Skill(const std::string &json)
{
	from_json(json);
}

Skill::Skill(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
Skill::to_json(bool pretty) const
{
	rapidjson::Document d;

	to_json_value(d, d);

	rapidjson::StringBuffer buffer;
	if (pretty) {
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
		d.Accept(writer);
	} else {
		rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
		d.Accept(writer);
	}

	return buffer.GetString();
}

void
Skill::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
{
	rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
	v.SetObject();
	// Avoid unused variable warnings
	(void)allocator;

	if (kind_) {
		rapidjson::Value v_kind;
		v_kind.SetString(*kind_, allocator);
		v.AddMember("kind", v_kind, allocator);
	}
	if (apiVersion_) {
		rapidjson::Value v_apiVersion;
		v_apiVersion.SetString(*apiVersion_, allocator);
		v.AddMember("apiVersion", v_apiVersion, allocator);
	}
	if (name_) {
		rapidjson::Value v_name;
		v_name.SetString(*name_, allocator);
		v.AddMember("name", v_name, allocator);
	}
	if (graph_) {
		rapidjson::Value v_graph;
		v_graph.SetString(*graph_, allocator);
		v.AddMember("graph", v_graph, allocator);
	}
	if (skill_string_) {
		rapidjson::Value v_skill_string;
		v_skill_string.SetString(*skill_string_, allocator);
		v.AddMember("skill-string", v_skill_string, allocator);
	}
	if (error_) {
		rapidjson::Value v_error;
		v_error.SetString(*error_, allocator);
		v.AddMember("error", v_error, allocator);
	}
	if (msg_id_) {
		rapidjson::Value v_msg_id;
		v_msg_id.SetInt64(*msg_id_);
		v.AddMember("msg_id", v_msg_id, allocator);
	}
	if (exclusive_controller_) {
		rapidjson::Value v_exclusive_controller;
		v_exclusive_controller.SetInt64(*exclusive_controller_);
		v.AddMember("exclusive_controller", v_exclusive_controller, allocator);
	}
	if (status_) {
		rapidjson::Value v_status;
		v_status.SetString(*status_, allocator);
		v.AddMember("status", v_status, allocator);
	}

}

void
Skill::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
Skill::from_json_value(const rapidjson::Value& d)
{
	if (d.HasMember("kind") && d["kind"].IsString()) {
		kind_ = d["kind"].GetString();
	}
	if (d.HasMember("apiVersion") && d["apiVersion"].IsString()) {
		apiVersion_ = d["apiVersion"].GetString();
	}
	if (d.HasMember("name") && d["name"].IsString()) {
		name_ = d["name"].GetString();
	}
	if (d.HasMember("graph") && d["graph"].IsString()) {
		graph_ = d["graph"].GetString();
	}
	if (d.HasMember("skill-string") && d["skill-string"].IsString()) {
		skill_string_ = d["skill-string"].GetString();
	}
	if (d.HasMember("error") && d["error"].IsString()) {
		error_ = d["error"].GetString();
	}
	if (d.HasMember("msg_id") && d["msg_id"].IsInt64()) {
		msg_id_ = d["msg_id"].GetInt64();
	}
	if (d.HasMember("exclusive_controller") && d["exclusive_controller"].IsInt64()) {
		exclusive_controller_ = d["exclusive_controller"].GetInt64();
	}
	if (d.HasMember("status") && d["status"].IsString()) {
		status_ = d["status"].GetString();
	}

}

void
Skill::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! name_)  missing.push_back("name");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "Skill is missing field"
			  << ((missing.size() > 0) ? "s" : "")
			  << ": ";
			for (std::vector<std::string>::size_type i = 0; i < missing.size(); ++i) {
				s << missing[i];
				if (i < (missing.size() - 1)) {
					s << ", ";
				}
			}
			throw std::runtime_error(s.str());
		}
	}
}