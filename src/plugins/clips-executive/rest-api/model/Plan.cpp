
/****************************************************************************
 *  Plan
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "Plan.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <sstream>

Plan::Plan()
{
}

Plan::Plan(const std::string &json)
{
	from_json(json);
}

Plan::Plan(const rapidjson::Value& v)
{
	from_json_value(v);
}

std::string
Plan::to_json(bool pretty) const
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
Plan::to_json_value(rapidjson::Document& d, rapidjson::Value& v) const
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
	if (goal_id_) {
		rapidjson::Value v_goal_id;
		v_goal_id.SetString(*goal_id_, allocator);
		v.AddMember("goal-id", v_goal_id, allocator);
	}
	if (cost_) {
		rapidjson::Value v_cost;
		v_cost.SetFloat(*cost_);
		v.AddMember("cost", v_cost, allocator);
	}
	rapidjson::Value v_actions(rapidjson::kArrayType);
	v_actions.Reserve(actions_.size(), allocator);
	for (const auto & e : actions_) {
		rapidjson::Value v(rapidjson::kObjectType);
		e->to_json_value(d, v);
		v_actions.PushBack(v, allocator);
	}
	v.AddMember("actions", v_actions, allocator);

}

void
Plan::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
Plan::from_json_value(const rapidjson::Value& d)
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
	if (d.HasMember("goal-id") && d["goal-id"].IsString()) {
		goal_id_ = d["goal-id"].GetString();
	}
	if (d.HasMember("cost") && d["cost"].IsFloat()) {
		cost_ = d["cost"].GetFloat();
	}
	if (d.HasMember("actions") && d["actions"].IsArray()) {
		const rapidjson::Value& a = d["actions"];
		actions_ = std::vector<std::shared_ptr<PlanAction>>{};
;
		actions_.reserve(a.Size());
		for (auto& v : a.GetArray()) {
			std::shared_ptr<PlanAction> nv{new PlanAction()};
			nv->from_json_value(v);
			actions_.push_back(std::move(nv));
		}	
	}	

}

void
Plan::validate(bool subcall) const
{
  std::vector<std::string> missing;
	if (! kind_)  missing.push_back("kind");
	if (! apiVersion_)  missing.push_back("apiVersion");
	if (! id_)  missing.push_back("id");
	if (! goal_id_)  missing.push_back("goal-id");
	for (size_t i = 0; i < actions_.size(); ++i) {
		if (! actions_[i]) {
			missing.push_back("actions[" + std::to_string(i) + "]");
		} else {
			try {
				actions_[i]->validate(true);
			} catch (std::vector<std::string> &subcall_missing) {
				for (const auto &s : subcall_missing) {
					missing.push_back("actions[" + std::to_string(i) + "]." + s);
				}
			}
		}
	}

	if (! missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "Plan is missing field"
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