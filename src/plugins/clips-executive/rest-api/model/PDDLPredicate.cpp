
/****************************************************************************
 *  PDDLPredicate
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "PDDLPredicate.h"

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <numeric>

#include <sstream>

PDDLPredicate::PDDLPredicate()
{
}

PDDLPredicate::PDDLPredicate(const std::string &json)
{
	from_json(json);
}

PDDLPredicate::PDDLPredicate(const rapidjson::Value &v)
{
	from_json_value(v);
}

PDDLPredicate::~PDDLPredicate()
{
}

std::string
PDDLPredicate::to_json(bool pretty) const
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
PDDLPredicate::to_json_value(rapidjson::Document &d, rapidjson::Value &v) const
{
	rapidjson::Document::AllocatorType &allocator = d.GetAllocator();
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
	if (part_of_) {
		rapidjson::Value v_part_of;
		v_part_of.SetString(*part_of_, allocator);
		v.AddMember("part-of", v_part_of, allocator);
	}
	if (predicate_) {
		rapidjson::Value v_predicate;
		v_predicate.SetString(*predicate_, allocator);
		v.AddMember("predicate", v_predicate, allocator);
	}
	rapidjson::Value v_param_names(rapidjson::kArrayType);
	v_param_names.Reserve(param_names_.size(), allocator);
	for (const auto &e : param_names_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_param_names.PushBack(v, allocator);
	}
	v.AddMember("param-names", v_param_names, allocator);
	rapidjson::Value v_param_constants(rapidjson::kArrayType);
	v_param_constants.Reserve(param_constants_.size(), allocator);
	for (const auto &e : param_constants_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_param_constants.PushBack(v, allocator);
	}
	v.AddMember("param-constants", v_param_constants, allocator);
}

void
PDDLPredicate::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
PDDLPredicate::from_json_value(const rapidjson::Value &d)
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
	if (d.HasMember("part-of") && d["part-of"].IsString()) {
		part_of_ = d["part-of"].GetString();
	}
	if (d.HasMember("predicate") && d["predicate"].IsString()) {
		predicate_ = d["predicate"].GetString();
	}
	if (d.HasMember("param-names") && d["param-names"].IsArray()) {
		const rapidjson::Value &a = d["param-names"];
		param_names_              = std::vector<std::string>{};

		param_names_.reserve(a.Size());
		for (auto &v : a.GetArray()) {
			param_names_.push_back(v.GetString());
		}
	}
	if (d.HasMember("param-constants") && d["param-constants"].IsArray()) {
		const rapidjson::Value &a = d["param-constants"];
		param_constants_          = std::vector<std::string>{};

		param_constants_.reserve(a.Size());
		for (auto &v : a.GetArray()) {
			param_constants_.push_back(v.GetString());
		}
	}
}

void
PDDLPredicate::validate(bool subcall) const
{
	std::vector<std::string> missing;
	if (!kind_) {
		missing.push_back("kind");
	}
	if (!apiVersion_) {
		missing.push_back("apiVersion");
	}
	if (!id_) {
		missing.push_back("id");
	}
	if (!part_of_) {
		missing.push_back("part-of");
	}
	if (!predicate_) {
		missing.push_back("predicate");
	}

	if (!missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::string s =
			  std::accumulate(std::next(missing.begin()),
			                  missing.end(),
			                  missing.front(),
			                  [](std::string &s, const std::string &n) { return s + ", " + n; });
			throw std::runtime_error("PDDLPredicate is missing " + s);
		}
	}
}