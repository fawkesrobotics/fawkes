
/****************************************************************************
 *  InterfaceFieldType
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "InterfaceFieldType.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

InterfaceFieldType::InterfaceFieldType()
{
}

InterfaceFieldType::InterfaceFieldType(const std::string &json)
{
	from_json(json);
}

InterfaceFieldType::InterfaceFieldType(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
InterfaceFieldType::to_json(bool pretty) const
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
InterfaceFieldType::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
{
	rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
	v.SetObject();
	// Avoid unused variable warnings
	(void)allocator;

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
	if (is_array_) {
		rapidjson::Value v_is_array;
		v_is_array.SetBool(*is_array_);
		v.AddMember("is_array", v_is_array, allocator);
	}
	rapidjson::Value v_enums(rapidjson::kArrayType);
	v_enums.Reserve(enums_.size(), allocator);
	for (const auto & e : enums_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_enums.PushBack(v, allocator);
	}
	v.AddMember("enums", v_enums, allocator);

}

void
InterfaceFieldType::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
InterfaceFieldType::from_json_value(const rapidjson::Value& d)
{
	if (d.HasMember("name") && d["name"].IsString()) {
		name_ = d["name"].GetString();
	}
	if (d.HasMember("type") && d["type"].IsString()) {
		type_ = d["type"].GetString();
	}
	if (d.HasMember("is_array") && d["is_array"].IsBool()) {
		is_array_ = d["is_array"].GetBool();
	}
	if (d.HasMember("enums") && d["enums"].IsArray()) {
		const rapidjson::Value& a = d["enums"];
		enums_ = std::vector<std::string>{};
;
		enums_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			enums_.push_back(v.GetString());
		}	
	}	

}

void
InterfaceFieldType::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! name_)  missing.push_back("name");
	if (! type_)  missing.push_back("type");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "InterfaceFieldType is missing field"
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