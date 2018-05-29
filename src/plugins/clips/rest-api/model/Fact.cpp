
/****************************************************************************
 *  Fact
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS REST API.
 *  Enables access to CLIPS environments.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "Fact.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

Fact::Fact()
{
}

Fact::Fact(const std::string &json)
{
	from_json(json);
}

Fact::Fact(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
Fact::to_json(bool pretty) const
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
Fact::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (index_) {
		rapidjson::Value v_index;
		v_index.SetInt64(*index_);
		v.AddMember("index", v_index, allocator);
	}
	if (template_name_) {
		rapidjson::Value v_template_name;
		v_template_name.SetString(*template_name_, allocator);
		v.AddMember("template_name", v_template_name, allocator);
	}
	if (formatted_) {
		rapidjson::Value v_formatted;
		v_formatted.SetString(*formatted_, allocator);
		v.AddMember("formatted", v_formatted, allocator);
	}
	rapidjson::Value v_slots(rapidjson::kArrayType);
	v_slots.Reserve(slots_.size(), allocator);
	for (const auto & e : slots_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_slots.PushBack(v, allocator);
	}
	v.AddMember("slots", v_slots, allocator);

}

void
Fact::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
Fact::from_json_value(const rapidjson::Value& d)
{
	if (d.HasMember("kind") && d["kind"].IsString()) {
		kind_ = d["kind"].GetString();
	}
	if (d.HasMember("apiVersion") && d["apiVersion"].IsString()) {
		apiVersion_ = d["apiVersion"].GetString();
	}
	if (d.HasMember("index") && d["index"].IsInt64()) {
		index_ = d["index"].GetInt64();
	}
	if (d.HasMember("template_name") && d["template_name"].IsString()) {
		template_name_ = d["template_name"].GetString();
	}
	if (d.HasMember("formatted") && d["formatted"].IsString()) {
		formatted_ = d["formatted"].GetString();
	}
	if (d.HasMember("slots") && d["slots"].IsArray()) {
		const rapidjson::Value& a = d["slots"];
		slots_ = std::vector<std::shared_ptr<SlotValue>>{};
;
		slots_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			std::shared_ptr<SlotValue> nv{new SlotValue()};
			nv->from_json_value(v);
			slots_.push_back(std::move(nv));
		}	
	}	

}

void
Fact::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! index_)  missing.push_back("index");
	if (! template_name_)  missing.push_back("template_name");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "Fact is missing field"
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