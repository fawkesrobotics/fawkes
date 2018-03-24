
/****************************************************************************
 *  DomainPrecondition
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "DomainPrecondition.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

DomainPrecondition::DomainPrecondition()
{
}

DomainPrecondition::DomainPrecondition(const std::string &json)
{
	from_json(json);
}

DomainPrecondition::DomainPrecondition(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
DomainPrecondition::to_json(bool pretty) const
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
DomainPrecondition::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (type_) {
		rapidjson::Value v_type;
		v_type.SetString(*type_, allocator);
		v.AddMember("type", v_type, allocator);
	}
	if (grounded_) {
		rapidjson::Value v_grounded;
		v_grounded.SetBool(*grounded_);
		v.AddMember("grounded", v_grounded, allocator);
	}
	if (is_satisfied_) {
		rapidjson::Value v_is_satisfied;
		v_is_satisfied.SetBool(*is_satisfied_);
		v.AddMember("is-satisfied", v_is_satisfied, allocator);
	}

}

void
DomainPrecondition::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
DomainPrecondition::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("type") && d["type"].IsString()) {
		type_ = d["type"].GetString();
	}
	if (d.HasMember("grounded") && d["grounded"].IsBool()) {
		grounded_ = d["grounded"].GetBool();
	}
	if (d.HasMember("is-satisfied") && d["is-satisfied"].IsBool()) {
		is_satisfied_ = d["is-satisfied"].GetBool();
	}

}

void
DomainPrecondition::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! name_)  missing.push_back("name");
	if (! type_)  missing.push_back("type");
	if (! grounded_)  missing.push_back("grounded");
	if (! is_satisfied_)  missing.push_back("is-satisfied");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "DomainPrecondition is missing field"
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