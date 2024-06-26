
/****************************************************************************
 *  GroundedPDDLPredicate
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "GroundedPDDLPredicate.h"

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <numeric>
#include <sstream>

GroundedPDDLPredicate::GroundedPDDLPredicate()
{
}

GroundedPDDLPredicate::GroundedPDDLPredicate(const std::string &json)
{
	from_json(json);
}

GroundedPDDLPredicate::GroundedPDDLPredicate(const rapidjson::Value &v)
{
	from_json_value(v);
}

GroundedPDDLPredicate::~GroundedPDDLPredicate()
{
}

std::string
GroundedPDDLPredicate::to_json(bool pretty) const
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
GroundedPDDLPredicate::to_json_value(rapidjson::Document &d, rapidjson::Value &v) const
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
	if (predicate_id_) {
		rapidjson::Value v_predicate_id;
		v_predicate_id.SetString(*predicate_id_, allocator);
		v.AddMember("predicate-id", v_predicate_id, allocator);
	}
	if (grounding_) {
		rapidjson::Value v_grounding;
		v_grounding.SetString(*grounding_, allocator);
		v.AddMember("grounding", v_grounding, allocator);
	}
	if (is_satisfied_) {
		rapidjson::Value v_is_satisfied;
		v_is_satisfied.SetBool(*is_satisfied_);
		v.AddMember("is-satisfied", v_is_satisfied, allocator);
	}
}

void
GroundedPDDLPredicate::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
GroundedPDDLPredicate::from_json_value(const rapidjson::Value &d)
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
	if (d.HasMember("predicate-id") && d["predicate-id"].IsString()) {
		predicate_id_ = d["predicate-id"].GetString();
	}
	if (d.HasMember("grounding") && d["grounding"].IsString()) {
		grounding_ = d["grounding"].GetString();
	}
	if (d.HasMember("is-satisfied") && d["is-satisfied"].IsBool()) {
		is_satisfied_ = d["is-satisfied"].GetBool();
	}
}

void
GroundedPDDLPredicate::validate(bool subcall) const
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
	if (!predicate_id_) {
		missing.push_back("predicate-id");
	}
	if (!grounding_) {
		missing.push_back("grounding");
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
			throw std::runtime_error("GroundedPDDLPredicate is missing " + s);
		}
	}
}
