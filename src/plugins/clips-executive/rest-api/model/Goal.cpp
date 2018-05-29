
/****************************************************************************
 *  Goal
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "Goal.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

Goal::Goal()
{
}

Goal::Goal(const std::string &json)
{
	from_json(json);
}

Goal::Goal(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
Goal::to_json(bool pretty) const
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
Goal::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (id_) {
		rapidjson::Value v_id;
		v_id.SetString(*id_, allocator);
		v.AddMember("id", v_id, allocator);
	}
	if (type_) {
		rapidjson::Value v_type;
		v_type.SetString(*type_, allocator);
		v.AddMember("type", v_type, allocator);
	}
	if (_class_) {
		rapidjson::Value v__class;
		v__class.SetString(*_class_, allocator);
		v.AddMember("class", v__class, allocator);
	}
	if (mode_) {
		rapidjson::Value v_mode;
		v_mode.SetString(*mode_, allocator);
		v.AddMember("mode", v_mode, allocator);
	}
	if (outcome_) {
		rapidjson::Value v_outcome;
		v_outcome.SetString(*outcome_, allocator);
		v.AddMember("outcome", v_outcome, allocator);
	}
	if (message_) {
		rapidjson::Value v_message;
		v_message.SetString(*message_, allocator);
		v.AddMember("message", v_message, allocator);
	}
	if (parent_) {
		rapidjson::Value v_parent;
		v_parent.SetString(*parent_, allocator);
		v.AddMember("parent", v_parent, allocator);
	}
	if (priority_) {
		rapidjson::Value v_priority;
		v_priority.SetInt64(*priority_);
		v.AddMember("priority", v_priority, allocator);
	}
	rapidjson::Value v_parameters(rapidjson::kArrayType);
	v_parameters.Reserve(parameters_.size(), allocator);
	for (const auto & e : parameters_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_parameters.PushBack(v, allocator);
	}
	v.AddMember("parameters", v_parameters, allocator);
	rapidjson::Value v_plans(rapidjson::kArrayType);
	v_plans.Reserve(plans_.size(), allocator);
	for (const auto & e : plans_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_plans.PushBack(v, allocator);
	}
	v.AddMember("plans", v_plans, allocator);

}

void
Goal::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
Goal::from_json_value(const rapidjson::Value& d)
{
	if (d.HasMember("kind") && d["kind"].IsString()) {
		kind_ = d["kind"].GetString();
	}
	if (d.HasMember("apiVersion") && d["apiVersion"].IsString()) {
		apiVersion_ = d["apiVersion"].GetString();
	}
	if (d.HasMember("id") && d["id"].IsString()) {
		id_ = d["id"].GetString();
	}
	if (d.HasMember("type") && d["type"].IsString()) {
		type_ = d["type"].GetString();
	}
	if (d.HasMember("class") && d["class"].IsString()) {
		_class_ = d["class"].GetString();
	}
	if (d.HasMember("mode") && d["mode"].IsString()) {
		mode_ = d["mode"].GetString();
	}
	if (d.HasMember("outcome") && d["outcome"].IsString()) {
		outcome_ = d["outcome"].GetString();
	}
	if (d.HasMember("message") && d["message"].IsString()) {
		message_ = d["message"].GetString();
	}
	if (d.HasMember("parent") && d["parent"].IsString()) {
		parent_ = d["parent"].GetString();
	}
	if (d.HasMember("priority") && d["priority"].IsInt64()) {
		priority_ = d["priority"].GetInt64();
	}
	if (d.HasMember("parameters") && d["parameters"].IsArray()) {
		const rapidjson::Value& a = d["parameters"];
		parameters_ = std::vector<std::string>{};
;
		parameters_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			parameters_.push_back(v.GetString());
		}	
	}	
	if (d.HasMember("plans") && d["plans"].IsArray()) {
		const rapidjson::Value& a = d["plans"];
		plans_ = std::vector<std::string>{};
;
		plans_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			plans_.push_back(v.GetString());
		}	
	}	

}

void
Goal::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! id_)  missing.push_back("id");
	if (! type_)  missing.push_back("type");
	if (! _class_)  missing.push_back("class");
	if (! mode_)  missing.push_back("mode");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "Goal is missing field"
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