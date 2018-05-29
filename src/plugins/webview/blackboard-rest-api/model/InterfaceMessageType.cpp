
/****************************************************************************
 *  InterfaceMessageType
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "InterfaceMessageType.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

InterfaceMessageType::InterfaceMessageType()
{
}

InterfaceMessageType::InterfaceMessageType(const std::string &json)
{
	from_json(json);
}

InterfaceMessageType::InterfaceMessageType(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
InterfaceMessageType::to_json(bool pretty) const
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
InterfaceMessageType::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	rapidjson::Value v_fields(rapidjson::kArrayType);
	v_fields.Reserve(fields_.size(), allocator);
	for (const auto & e : fields_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_fields.PushBack(v, allocator);
	}
	v.AddMember("fields", v_fields, allocator);

}

void
InterfaceMessageType::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
InterfaceMessageType::from_json_value(const rapidjson::Value& d)
{
	if (d.HasMember("name") && d["name"].IsString()) {
		name_ = d["name"].GetString();
	}
	if (d.HasMember("fields") && d["fields"].IsArray()) {
		const rapidjson::Value& a = d["fields"];
		fields_ = std::vector<std::shared_ptr<InterfaceFieldType>>{};
;
		fields_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			std::shared_ptr<InterfaceFieldType> nv{new InterfaceFieldType()};
			nv->from_json_value(v);
			fields_.push_back(std::move(nv));
		}	
	}	

}

void
InterfaceMessageType::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! name_)  missing.push_back("name");
	for (size_t i = 0; i < fields_.size(); ++i) {
		if (! fields_[i]) {
			missing.push_back("fields[" + std::to_string(i) + "]");
		} else {
			try {
				fields_[i]->validate(true);
			} catch (std::vector<std::string> &subcall_missing) {
				for (const auto &s : subcall_missing) {
					missing.push_back("fields[" + std::to_string(i) + "]." + s);
				}
			}
		}
	}

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "InterfaceMessageType is missing field"
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