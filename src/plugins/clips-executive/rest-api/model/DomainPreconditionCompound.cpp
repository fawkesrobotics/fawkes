
/****************************************************************************
 *  DomainPreconditionCompound
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "DomainPreconditionCompound.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

DomainPreconditionCompound::DomainPreconditionCompound()
{
}

DomainPreconditionCompound::DomainPreconditionCompound(const std::string &json)
{
	from_json(json);
}

DomainPreconditionCompound::DomainPreconditionCompound(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
DomainPreconditionCompound::to_json(bool pretty) const
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
DomainPreconditionCompound::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
{
	rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
	v.SetObject();
	// Avoid unused variable warnings
	(void)allocator;

	DomainPrecondition::to_json_value(d, v);
	rapidjson::Value v_elements(rapidjson::kArrayType);
	v_elements.Reserve(elements_.size(), allocator);
	for (const auto & e : elements_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_elements.PushBack(v, allocator);
	}
	v.AddMember("elements", v_elements, allocator);

}

void
DomainPreconditionCompound::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
DomainPreconditionCompound::from_json_value(const rapidjson::Value& d)
{
	DomainPrecondition::from_json_value(d);
	if (d.HasMember("elements") && d["elements"].IsArray()) {
		const rapidjson::Value& a = d["elements"];
		elements_ = std::vector<std::shared_ptr<DomainPrecondition>>{};
;
		elements_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			std::shared_ptr<DomainPrecondition> nv{new DomainPrecondition()};
			nv->from_json_value(v);
			elements_.push_back(std::move(nv));
		}	
	}	

}

void
DomainPreconditionCompound::validate(bool subcall) const
{
  std::vector<std::string> missing;
	try {
  	DomainPrecondition::validate(true);
	} catch (std::vector<std::string> &supertype_missing) {
		missing.insert(missing.end(), supertype_missing.begin(), supertype_missing.end());
	}
	for (size_t i = 0; i < elements_.size(); ++i) {
		if (! elements_[i]) {
			missing.push_back("elements[" + std::to_string(i) + "]");
		} else {
			try {
				elements_[i]->validate(true);
			} catch (std::vector<std::string> &subcall_missing) {
				for (const auto &s : subcall_missing) {
					missing.push_back("elements[" + std::to_string(i) + "]." + s);
				}
			}
		}
	}


	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "DomainPreconditionCompound is missing field"
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