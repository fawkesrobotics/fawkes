
/****************************************************************************
 *  SlotValue
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS REST API.
 *  Enables access to CLIPS environments.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "SlotValue.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

SlotValue::SlotValue()
{
}

SlotValue::SlotValue(const std::string &json)
{
	from_json(json);
}

SlotValue::SlotValue(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
SlotValue::to_json(bool pretty) const
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
SlotValue::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (is_multifield_) {
		rapidjson::Value v_is_multifield;
		v_is_multifield.SetBool(*is_multifield_);
		v.AddMember("is-multifield", v_is_multifield, allocator);
	}
	rapidjson::Value v_values(rapidjson::kArrayType);
	v_values.Reserve(values_.size(), allocator);
	for (const auto & e : values_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_values.PushBack(v, allocator);
	}
	v.AddMember("values", v_values, allocator);

}

void
SlotValue::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
SlotValue::from_json_value(const rapidjson::Value& d)
{
	if (d.HasMember("name") && d["name"].IsString()) {
		name_ = d["name"].GetString();
	}
	if (d.HasMember("type") && d["type"].IsString()) {
		type_ = d["type"].GetString();
	}
	if (d.HasMember("is-multifield") && d["is-multifield"].IsBool()) {
		is_multifield_ = d["is-multifield"].GetBool();
	}
	if (d.HasMember("values") && d["values"].IsArray()) {
		const rapidjson::Value& a = d["values"];
		values_ = std::vector<std::string>{};
;
		values_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			values_.push_back(v.GetString());
		}	
	}	

}

void
SlotValue::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! name_)  missing.push_back("name");
	if (! is_multifield_)  missing.push_back("is-multifield");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "SlotValue is missing field"
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