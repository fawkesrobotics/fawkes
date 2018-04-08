
/****************************************************************************
 *  ImageInfo
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Image REST API.
 *  Access images through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "ImageInfo.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

ImageInfo::ImageInfo()
{
}

ImageInfo::ImageInfo(const std::string &json)
{
	from_json(json);
}

ImageInfo::ImageInfo(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
ImageInfo::to_json(bool pretty) const
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
ImageInfo::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (colorspace_) {
		rapidjson::Value v_colorspace;
		v_colorspace.SetString(*colorspace_, allocator);
		v.AddMember("colorspace", v_colorspace, allocator);
	}
	if (frame_) {
		rapidjson::Value v_frame;
		v_frame.SetString(*frame_, allocator);
		v.AddMember("frame", v_frame, allocator);
	}
	if (width_) {
		rapidjson::Value v_width;
		v_width.SetInt64(*width_);
		v.AddMember("width", v_width, allocator);
	}
	if (height_) {
		rapidjson::Value v_height;
		v_height.SetInt64(*height_);
		v.AddMember("height", v_height, allocator);
	}
	if (mem_size_) {
		rapidjson::Value v_mem_size;
		v_mem_size.SetInt64(*mem_size_);
		v.AddMember("mem_size", v_mem_size, allocator);
	}

}

void
ImageInfo::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
ImageInfo::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("colorspace") && d["colorspace"].IsString()) {
		colorspace_ = d["colorspace"].GetString();
	}
	if (d.HasMember("frame") && d["frame"].IsString()) {
		frame_ = d["frame"].GetString();
	}
	if (d.HasMember("width") && d["width"].IsInt64()) {
		width_ = d["width"].GetInt64();
	}
	if (d.HasMember("height") && d["height"].IsInt64()) {
		height_ = d["height"].GetInt64();
	}
	if (d.HasMember("mem_size") && d["mem_size"].IsInt64()) {
		mem_size_ = d["mem_size"].GetInt64();
	}

}

void
ImageInfo::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! id_)  missing.push_back("id");
	if (! colorspace_)  missing.push_back("colorspace");
	if (! frame_)  missing.push_back("frame");
	if (! width_)  missing.push_back("width");
	if (! height_)  missing.push_back("height");
	if (! mem_size_)  missing.push_back("mem_size");

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "ImageInfo is missing field"
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