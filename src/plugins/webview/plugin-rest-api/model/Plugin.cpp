
/****************************************************************************
 *  Plugin
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Plugin REST API.
 *  List, load, and unload plugins.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "Plugin.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

Plugin::Plugin()
{
}

Plugin::Plugin(const std::string &json)
{
	from_json(json);
}

Plugin::Plugin(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
Plugin::to_json(bool pretty) const
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
Plugin::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (description_) {
		rapidjson::Value v_description;
		v_description.SetString(*description_, allocator);
		v.AddMember("description", v_description, allocator);
	}
	if (is_meta_) {
		rapidjson::Value v_is_meta;
		v_is_meta.SetBool(*is_meta_);
		v.AddMember("is_meta", v_is_meta, allocator);
	}
	rapidjson::Value v_meta_children(rapidjson::kArrayType);
	v_meta_children.Reserve(meta_children_.size(), allocator);
	for (const auto & e : meta_children_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_meta_children.PushBack(v, allocator);
	}
	v.AddMember("meta_children", v_meta_children, allocator);
	if (is_loaded_) {
		rapidjson::Value v_is_loaded;
		v_is_loaded.SetBool(*is_loaded_);
		v.AddMember("is_loaded", v_is_loaded, allocator);
	}

}

void
Plugin::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
Plugin::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("description") && d["description"].IsString()) {
		description_ = d["description"].GetString();
	}
	if (d.HasMember("is_meta") && d["is_meta"].IsBool()) {
		is_meta_ = d["is_meta"].GetBool();
	}
	if (d.HasMember("meta_children") && d["meta_children"].IsArray()) {
		const rapidjson::Value& a = d["meta_children"];
		meta_children_ = std::vector<std::string>{};
;
		meta_children_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			meta_children_.push_back(v.GetString());
		}	
	}	
	if (d.HasMember("is_loaded") && d["is_loaded"].IsBool()) {
		is_loaded_ = d["is_loaded"].GetBool();
	}

}

void
Plugin::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! name_)  missing.push_back("name");
	if (! description_)  missing.push_back("description");
	if (! is_meta_)  missing.push_back("is_meta");
	if (! is_loaded_)  missing.push_back("is_loaded");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "Plugin is missing field"
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