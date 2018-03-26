
/****************************************************************************
 *  InterfaceInfo
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "InterfaceInfo.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

InterfaceInfo::InterfaceInfo()
{
}

InterfaceInfo::InterfaceInfo(const std::string &json)
{
	from_json(json);
}

InterfaceInfo::InterfaceInfo(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
InterfaceInfo::to_json(bool pretty) const
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
InterfaceInfo::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (type_) {
		rapidjson::Value v_type;
		v_type.SetString(*type_, allocator);
		v.AddMember("type", v_type, allocator);
	}
	if (hash_) {
		rapidjson::Value v_hash;
		v_hash.SetString(*hash_, allocator);
		v.AddMember("hash", v_hash, allocator);
	}
	if (writer_) {
		rapidjson::Value v_writer;
		v_writer.SetString(*writer_, allocator);
		v.AddMember("writer", v_writer, allocator);
	}
	rapidjson::Value v_readers(rapidjson::kArrayType);
	v_readers.Reserve(readers_.size(), allocator);
	for (const auto & e : readers_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_readers.PushBack(v, allocator);
	}
	v.AddMember("readers", v_readers, allocator);
	rapidjson::Value v_fields(rapidjson::kArrayType);
	v_fields.Reserve(fields_.size(), allocator);
	for (const auto & e : fields_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_fields.PushBack(v, allocator);
	}
	v.AddMember("fields", v_fields, allocator);
	rapidjson::Value v_message_types(rapidjson::kArrayType);
	v_message_types.Reserve(message_types_.size(), allocator);
	for (const auto & e : message_types_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_message_types.PushBack(v, allocator);
	}
	v.AddMember("message_types", v_message_types, allocator);

}

void
InterfaceInfo::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
InterfaceInfo::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("type") && d["type"].IsString()) {
		type_ = d["type"].GetString();
	}
	if (d.HasMember("hash") && d["hash"].IsString()) {
		hash_ = d["hash"].GetString();
	}
	if (d.HasMember("writer") && d["writer"].IsString()) {
		writer_ = d["writer"].GetString();
	}
	if (d.HasMember("readers") && d["readers"].IsArray()) {
		const rapidjson::Value& a = d["readers"];
		readers_ = std::vector<std::string>{};
;
		readers_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			readers_.push_back(v.GetString());
		}	
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
	if (d.HasMember("message_types") && d["message_types"].IsArray()) {
		const rapidjson::Value& a = d["message_types"];
		message_types_ = std::vector<std::shared_ptr<InterfaceMessageType>>{};
;
		message_types_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			std::shared_ptr<InterfaceMessageType> nv{new InterfaceMessageType()};
			nv->from_json_value(v);
			message_types_.push_back(std::move(nv));
		}	
	}	

}

void
InterfaceInfo::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! id_)  missing.push_back("id");
	if (! type_)  missing.push_back("type");
	if (! hash_)  missing.push_back("hash");
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
	for (size_t i = 0; i < message_types_.size(); ++i) {
		if (! message_types_[i]) {
			missing.push_back("message_types[" + std::to_string(i) + "]");
		} else {
			try {
				message_types_[i]->validate(true);
			} catch (std::vector<std::string> &subcall_missing) {
				for (const auto &s : subcall_missing) {
					missing.push_back("message_types[" + std::to_string(i) + "]." + s);
				}
			}
		}
	}

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "InterfaceInfo is missing field"
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