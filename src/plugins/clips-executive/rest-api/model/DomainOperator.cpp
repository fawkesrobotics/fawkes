
/****************************************************************************
 *  DomainOperator
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "DomainOperator.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

DomainOperator::DomainOperator()
{
}

DomainOperator::DomainOperator(const std::string &json)
{
	from_json(json);
}

DomainOperator::DomainOperator(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
DomainOperator::to_json(bool pretty) const
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
DomainOperator::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (wait_sensed_) {
		rapidjson::Value v_wait_sensed;
		v_wait_sensed.SetBool(*wait_sensed_);
		v.AddMember("wait-sensed", v_wait_sensed, allocator);
	}
	rapidjson::Value v_parameters(rapidjson::kArrayType);
	v_parameters.Reserve(parameters_.size(), allocator);
	for (const auto & e : parameters_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_parameters.PushBack(v, allocator);
	}
	v.AddMember("parameters", v_parameters, allocator);

}

void
DomainOperator::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
DomainOperator::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("wait-sensed") && d["wait-sensed"].IsBool()) {
		wait_sensed_ = d["wait-sensed"].GetBool();
	}
	if (d.HasMember("parameters") && d["parameters"].IsArray()) {
		const rapidjson::Value& a = d["parameters"];
		parameters_ = std::vector<std::shared_ptr<DomainOperatorParameter>>{};
;
		parameters_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			std::shared_ptr<DomainOperatorParameter> nv{new DomainOperatorParameter()};
			nv->from_json_value(v);
			parameters_.push_back(std::move(nv));
		}	
	}	

}

void
DomainOperator::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! name_)  missing.push_back("name");
	if (! wait_sensed_)  missing.push_back("wait-sensed");
	for (size_t i = 0; i < parameters_.size(); ++i) {
		if (! parameters_[i]) {
			missing.push_back("parameters[" + std::to_string(i) + "]");
		} else {
			try {
				parameters_[i]->validate(true);
			} catch (std::vector<std::string> &subcall_missing) {
				for (const auto &s : subcall_missing) {
					missing.push_back("parameters[" + std::to_string(i) + "]." + s);
				}
			}
		}
	}

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "DomainOperator is missing field"
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