
/****************************************************************************
 *  PlanAction
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "PlanAction.h"

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <numeric>
#include <sstream>

PlanAction::PlanAction()
{
}

PlanAction::PlanAction(const std::string &json)
{
	from_json(json);
}

PlanAction::PlanAction(const rapidjson::Value &v)
{
	from_json_value(v);
}

PlanAction::~PlanAction()
{
}

std::string
PlanAction::to_json(bool pretty) const
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
PlanAction::to_json_value(rapidjson::Document &d, rapidjson::Value &v) const
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
		v_id.SetInt64(*id_);
		v.AddMember("id", v_id, allocator);
	}
	if (operator_name_) {
		rapidjson::Value v_operator_name;
		v_operator_name.SetString(*operator_name_, allocator);
		v.AddMember("operator-name", v_operator_name, allocator);
	}
	rapidjson::Value v_param_values(rapidjson::kArrayType);
	v_param_values.Reserve(param_values_.size(), allocator);
	for (const auto &e : param_values_) {
		rapidjson::Value v;
		v.SetString(e, allocator);
		v_param_values.PushBack(v, allocator);
	}
	v.AddMember("param-values", v_param_values, allocator);
	if (duration_) {
		rapidjson::Value v_duration;
		v_duration.SetFloat(*duration_);
		v.AddMember("duration", v_duration, allocator);
	}
	if (dispatch_time_) {
		rapidjson::Value v_dispatch_time;
		v_dispatch_time.SetFloat(*dispatch_time_);
		v.AddMember("dispatch-time", v_dispatch_time, allocator);
	}
	if (state_) {
		rapidjson::Value v_state;
		v_state.SetString(*state_, allocator);
		v.AddMember("state", v_state, allocator);
	}
	if (executable_) {
		rapidjson::Value v_executable;
		v_executable.SetBool(*executable_);
		v.AddMember("executable", v_executable, allocator);
	}
	if (_operator_) {
		rapidjson::Value v__operator(rapidjson::kObjectType);
		_operator_->to_json_value(d, v__operator);
		v.AddMember("operator", v__operator, allocator);
	}
	if (precondition_) {
		rapidjson::Value v_precondition(rapidjson::kObjectType);
		precondition_->to_json_value(d, v_precondition);
		v.AddMember("precondition", v_precondition, allocator);
	}
	rapidjson::Value v_effects(rapidjson::kArrayType);
	v_effects.Reserve(effects_.size(), allocator);
	for (const auto &e : effects_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_effects.PushBack(v, allocator);
	}
	v.AddMember("effects", v_effects, allocator);
	if (preconditions_) {
		rapidjson::Value v_preconditions(rapidjson::kObjectType);
		preconditions_->to_json_value(d, v_preconditions);
		v.AddMember("preconditions", v_preconditions, allocator);
	}
}

void
PlanAction::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
PlanAction::from_json_value(const rapidjson::Value &d)
{
	if (d.HasMember("kind") && d["kind"].IsString()) {
		kind_ = d["kind"].GetString();
	}
	if (d.HasMember("apiVersion") && d["apiVersion"].IsString()) {
		apiVersion_ = d["apiVersion"].GetString();
	}
	if (d.HasMember("id") && d["id"].IsInt64()) {
		id_ = d["id"].GetInt64();
	}
	if (d.HasMember("operator-name") && d["operator-name"].IsString()) {
		operator_name_ = d["operator-name"].GetString();
	}
	if (d.HasMember("param-values") && d["param-values"].IsArray()) {
		const rapidjson::Value &a = d["param-values"];
		param_values_             = std::vector<std::string>{};

		param_values_.reserve(a.Size());
		for (auto &v : a.GetArray()) {
			param_values_.push_back(v.GetString());
		}
	}
	if (d.HasMember("duration") && d["duration"].IsFloat()) {
		duration_ = d["duration"].GetFloat();
	}
	if (d.HasMember("dispatch-time") && d["dispatch-time"].IsFloat()) {
		dispatch_time_ = d["dispatch-time"].GetFloat();
	}
	if (d.HasMember("state") && d["state"].IsString()) {
		state_ = d["state"].GetString();
	}
	if (d.HasMember("executable") && d["executable"].IsBool()) {
		executable_ = d["executable"].GetBool();
	}
	if (d.HasMember("operator") && d["operator"].IsObject()) {
		std::shared_ptr<DomainOperator> nv{new DomainOperator(d["operator"])};
		_operator_ = std::move(nv);
	}
	if (d.HasMember("precondition") && d["precondition"].IsObject()) {
		std::shared_ptr<PDDLGrounding> nv{new PDDLGrounding(d["precondition"])};
		precondition_ = std::move(nv);
	}
	if (d.HasMember("effects") && d["effects"].IsArray()) {
		const rapidjson::Value &a = d["effects"];
		effects_                  = std::vector<std::shared_ptr<DomainEffect>>{};

		effects_.reserve(a.Size());
		for (auto &v : a.GetArray()) {
			std::shared_ptr<DomainEffect> nv{new DomainEffect()};
			nv->from_json_value(v);
			effects_.push_back(std::move(nv));
		}
	}
	if (d.HasMember("preconditions") && d["preconditions"].IsObject()) {
		std::shared_ptr<GroundedFormula> nv{new GroundedFormula(d["preconditions"])};
		preconditions_ = std::move(nv);
	}
}

void
PlanAction::validate(bool subcall) const
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
	if (!operator_name_) {
		missing.push_back("operator-name");
	}
	if (!state_) {
		missing.push_back("state");
	}
	if (!executable_) {
		missing.push_back("executable");
	}
	if (!preconditions_) {
		missing.push_back("preconditions");
	}
	for (size_t i = 0; i < effects_.size(); ++i) {
		if (!effects_[i]) {
			missing.push_back("effects[" + std::to_string(i) + "]");
		} else {
			try {
				effects_[i]->validate(true);
			} catch (std::vector<std::string> &subcall_missing) {
				for (const auto &s : subcall_missing) {
					missing.push_back("effects[" + std::to_string(i) + "]." + s);
				}
			}
		}
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
			throw std::runtime_error("PlanAction is missing " + s);
		}
	}
}
