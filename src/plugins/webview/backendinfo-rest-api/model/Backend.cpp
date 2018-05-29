
/****************************************************************************
 *  Backend
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Backend Info REST API.
 *  Provides backend meta information to the frontend.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "Backend.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

Backend::Backend()
{
}

Backend::Backend(const std::string &json)
{
	from_json(json);
}

Backend::Backend(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
Backend::to_json(bool pretty) const
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
Backend::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (name_) {
		rapidjson::Value v_name;
		v_name.SetString(*name_, allocator);
		v.AddMember("name", v_name, allocator);
	}
	if (url_) {
		rapidjson::Value v_url;
		v_url.SetString(*url_, allocator);
		v.AddMember("url", v_url, allocator);
	}
	rapidjson::Value v_services(rapidjson::kArrayType);
	v_services.Reserve(services_.size(), allocator);
	for (const auto & e : services_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_services.PushBack(v, allocator);
	}
	v.AddMember("services", v_services, allocator);

}

void
Backend::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
Backend::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("name") && d["name"].IsString()) {
		name_ = d["name"].GetString();
	}
	if (d.HasMember("url") && d["url"].IsString()) {
		url_ = d["url"].GetString();
	}
	if (d.HasMember("services") && d["services"].IsArray()) {
		const rapidjson::Value& a = d["services"];
		services_ = std::vector<std::shared_ptr<Service>>{};
;
		services_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			std::shared_ptr<Service> nv{new Service()};
			nv->from_json_value(v);
			services_.push_back(std::move(nv));
		}	
	}	

}

void
Backend::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! id_)  missing.push_back("id");
	if (! name_)  missing.push_back("name");
	for (size_t i = 0; i < services_.size(); ++i) {
		if (! services_[i]) {
			missing.push_back("services[" + std::to_string(i) + "]");
		} else {
			try {
				services_[i]->validate(true);
			} catch (std::vector<std::string> &subcall_missing) {
				for (const auto &s : subcall_missing) {
					missing.push_back("services[" + std::to_string(i) + "]." + s);
				}
			}
		}
	}

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "Backend is missing field"
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