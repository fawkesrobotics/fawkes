
/****************************************************************************
 *  DomainPredicate
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "DomainPredicate.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

DomainPredicate::DomainPredicate()
{
}

DomainPredicate::DomainPredicate(const std::string &json)
{
	from_json(json);
}

DomainPredicate::DomainPredicate(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
DomainPredicate::to_json(bool pretty) const
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
DomainPredicate::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (sensed_) {
		rapidjson::Value v_sensed;
		v_sensed.SetBool(*sensed_);
		v.AddMember("sensed", v_sensed, allocator);
	}
	rapidjson::Value v_param_names(rapidjson::kArrayType);
	v_param_names.Reserve(param_names_.size(), allocator);
	for (const auto & e : param_names_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_param_names.PushBack(v, allocator);
	}
	v.AddMember("param-names", v_param_names, allocator);
	rapidjson::Value v_param_types(rapidjson::kArrayType);
	v_param_types.Reserve(param_types_.size(), allocator);
	for (const auto & e : param_types_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_param_types.PushBack(v, allocator);
	}
	v.AddMember("param-types", v_param_types, allocator);

}

void
DomainPredicate::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
DomainPredicate::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("sensed") && d["sensed"].IsBool()) {
		sensed_ = d["sensed"].GetBool();
	}
	if (d.HasMember("param-names") && d["param-names"].IsArray()) {
		const rapidjson::Value& a = d["param-names"];
		param_names_ = std::vector<std::string>{};
;
		param_names_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			param_names_.push_back(v.GetString());
		}	
	}	
	if (d.HasMember("param-types") && d["param-types"].IsArray()) {
		const rapidjson::Value& a = d["param-types"];
		param_types_ = std::vector<std::string>{};
;
		param_types_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			param_types_.push_back(v.GetString());
		}	
	}	

}

void
DomainPredicate::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! name_)  missing.push_back("name");
	if (! sensed_)  missing.push_back("sensed");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "DomainPredicate is missing field"
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