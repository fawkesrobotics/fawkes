
/****************************************************************************
 *  InterfaceData
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "InterfaceData.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

InterfaceData::InterfaceData()
{
}

InterfaceData::InterfaceData(const std::string &json)
{
	from_json(json);
}

InterfaceData::InterfaceData(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
InterfaceData::to_json(bool pretty) const
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
InterfaceData::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (data_) {
		v.AddMember("data", *data_, allocator);
	}
	if (timestamp_) {
		rapidjson::Value v_timestamp;
		v_timestamp.SetString(*timestamp_, allocator);
		v.AddMember("timestamp", v_timestamp, allocator);
	}

}

void
InterfaceData::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
InterfaceData::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("data") && d["data"].IsObject()) {
		std::shared_ptr<rapidjson::Document> d_data =
		  std::make_shared<rapidjson::Document>();
		d_data->CopyFrom(d["data"], d_data->GetAllocator());
	}
	if (d.HasMember("timestamp") && d["timestamp"].IsString()) {
		timestamp_ = d["timestamp"].GetString();
	}

}

void
InterfaceData::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! id_)  missing.push_back("id");
	if (! type_)  missing.push_back("type");
	if (! data_)  missing.push_back("data");
	if (! timestamp_)  missing.push_back("timestamp");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "InterfaceData is missing field"
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