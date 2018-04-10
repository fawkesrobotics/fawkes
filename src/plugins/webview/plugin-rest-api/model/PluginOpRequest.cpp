
/****************************************************************************
 *  PluginOpRequest
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Plugin REST API.
 *  List, load, and unload plugins.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "PluginOpRequest.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

PluginOpRequest::PluginOpRequest()
{
}

PluginOpRequest::PluginOpRequest(const std::string &json)
{
	from_json(json);
}

PluginOpRequest::PluginOpRequest(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
PluginOpRequest::to_json(bool pretty) const
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
PluginOpRequest::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (type_) {
		rapidjson::Value v_type;
		v_type.SetString(*type_, allocator);
		v.AddMember("type", v_type, allocator);
	}
	if (desired_state_) {
		rapidjson::Value v_desired_state;
		v_desired_state.SetString(*desired_state_, allocator);
		v.AddMember("desired_state", v_desired_state, allocator);
	}

}

void
PluginOpRequest::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
PluginOpRequest::from_json_value(const rapidjson::Value& d)
{
	if (d.HasMember("kind") && d["kind"].IsString()) {
		kind_ = d["kind"].GetString();
	}
	if (d.HasMember("apiVersion") && d["apiVersion"].IsString()) {
		apiVersion_ = d["apiVersion"].GetString();
	}
	if (d.HasMember("type") && d["type"].IsString()) {
		type_ = d["type"].GetString();
	}
	if (d.HasMember("desired_state") && d["desired_state"].IsString()) {
		desired_state_ = d["desired_state"].GetString();
	}

}

void
PluginOpRequest::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! desired_state_)  missing.push_back("desired_state");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "PluginOpRequest is missing field"
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