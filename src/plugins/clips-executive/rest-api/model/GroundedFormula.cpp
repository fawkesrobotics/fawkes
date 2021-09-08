
/****************************************************************************
 *  GroundedFormula
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "GroundedFormula.h"

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <numeric>
#include <sstream>

GroundedFormula::GroundedFormula()
{
}

GroundedFormula::GroundedFormula(const std::string &json)
{
	from_json(json);
}

GroundedFormula::GroundedFormula(const rapidjson::Value &v)
{
	from_json_value(v);
}

GroundedFormula::~GroundedFormula()
{
}

std::string
GroundedFormula::to_json(bool pretty) const
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
GroundedFormula::to_json_value(rapidjson::Document &d, rapidjson::Value &v) const
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
	if (is_satisfied_) {
		rapidjson::Value v_is_satisfied;
		v_is_satisfied.SetBool(*is_satisfied_);
		v.AddMember("is-satisfied", v_is_satisfied, allocator);
	}
	rapidjson::Value v_param_names(rapidjson::kArrayType);
	v_param_names.Reserve(param_names_.size(), allocator);
	for (const auto &e : param_names_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_param_names.PushBack(v, allocator);
	}
	v.AddMember("param-names", v_param_names, allocator);
	rapidjson::Value v_param_values(rapidjson::kArrayType);
	v_param_values.Reserve(param_values_.size(), allocator);
	for (const auto &e : param_values_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_param_values.PushBack(v, allocator);
	}
	v.AddMember("param-values", v_param_values, allocator);
	rapidjson::Value v_param_constants(rapidjson::kArrayType);
	v_param_constants.Reserve(param_constants_.size(), allocator);
	for (const auto &e : param_constants_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_param_constants.PushBack(v, allocator);
	}
	v.AddMember("param-constants", v_param_constants, allocator);
	rapidjson::Value v_child(rapidjson::kArrayType);
	v_child.Reserve(child_.size(), allocator);
	for (const auto &e : child_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_child.PushBack(v, allocator);
	}
	v.AddMember("child", v_child, allocator);
}

void
GroundedFormula::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
GroundedFormula::from_json_value(const rapidjson::Value &d)
{
	if (d.HasMember("kind") && d["kind"].IsString()) {
		kind_ = d["kind"].GetString();
	}
	if (d.HasMember("apiVersion") && d["apiVersion"].IsString()) {
		apiVersion_ = d["apiVersion"].GetString();
	}
	if (d.HasMember("name") && d["name"].IsString()) {
		name_ = d["name"].GetString();
	}
	if (d.HasMember("type") && d["type"].IsString()) {
		type_ = d["type"].GetString();
	}
	if (d.HasMember("is-satisfied") && d["is-satisfied"].IsBool()) {
		is_satisfied_ = d["is-satisfied"].GetBool();
	}
	if (d.HasMember("param-names") && d["param-names"].IsArray()) {
		const rapidjson::Value &a = d["param-names"];
		param_names_              = std::vector<std::string>{};

		param_names_.reserve(a.Size());
		for (auto &v : a.GetArray()) {
			param_names_.push_back(v.GetString());
		}
	}
	if (d.HasMember("param-values") && d["param-values"].IsArray()) {
		const rapidjson::Value &a = d["param-values"];
		param_values_             = std::vector<std::string>{};

		param_values_.reserve(a.Size());
		for (auto &v : a.GetArray()) {
			param_values_.push_back(v.GetString());
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
	if (d.HasMember("child") && d["child"].IsArray()) {
		const rapidjson::Value &a = d["child"];
		child_                    = std::vector<std::shared_ptr<GroundedFormula>>{};

		child_.reserve(a.Size());
		for (auto &v : a.GetArray()) {
			std::shared_ptr<GroundedFormula> nv{new GroundedFormula()};
			nv->from_json_value(v);
			child_.push_back(std::move(nv));
		}
	}
}

void
GroundedFormula::validate(bool subcall) const
{
	std::vector<std::string> missing;
	if (!kind_) {
		missing.push_back("kind");
	}
	if (!apiVersion_) {
		missing.push_back("apiVersion");
	}
	if (!name_) {
		missing.push_back("name");
	}
	if (!type_) {
		missing.push_back("type");
	}
	if (!is_satisfied_) {
		missing.push_back("is-satisfied");
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
			throw std::runtime_error("GroundedFormula is missing " + s);
		}
	}
}
