
/****************************************************************************
 *  ClipsExecutive -- Schema Plan
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#pragma once

#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/fwd.h>

#include <string>
#include <cstdint>
#include <vector>
#include <memory>
#include <optional>

#include "PlanAction.h"


/** Plan representation for JSON transfer. */
class Plan

{
 public:
	/** Constructor. */
	Plan();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	Plan(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	Plan(const rapidjson::Value& v);

	/** Get version of implemented API.
	 * @return string representation of version
	 */
	static std::string api_version()
	{
	  return "v1beta1";
	}

	/** Render object to JSON.
	 * @param pretty true to enable pretty printing (readable spacing)
	 * @return JSON string
	 */
	virtual std::string to_json(bool pretty = false) const;
	/** Render object to JSON.
	 * @param d RapidJSON document to retrieve allocator from
	 * @param v RapidJSON value to add data to
	 */
	virtual void        to_json_value(rapidjson::Document& d, rapidjson::Value& v) const;
	/** Retrieve data from JSON string.
	 * @param json JSON representation suitable for this object.
	 * Will allow partial assignment and not validate automaticaly.
	 * @see validate()
	 */
	virtual void        from_json(const std::string& json);
	/** Retrieve data from JSON string.
	 * @param v RapidJSON value suitable for this object.
	 * Will allow partial assignment and not validate automaticaly.
	 * @see validate()
	 */
	virtual void        from_json_value(const rapidjson::Value& v);

	/** Validate if all required fields have been set.
	 * @param subcall true if this is called from another class, e.g.,
	 * a sub-class or array holder. Will modify the kind of exception thrown.
	 * @exception std::vector<std::string> thrown if required information is
	 * missing and @p subcall is set to true. Contains a list of missing fields.
	 * @exception std::runtime_error informative message describing the missing
	 * fields
	 */
	virtual void validate(bool subcall = false) const;

	// Schema: Plan
 public:
  /** Get kind value.
   * @return kind value
   */
	std::optional<std::string>
 kind() const
	{
		return kind_;
	}

	/** Set kind value.
	 * @param kind new value
	 */
	void set_kind(const std::string& kind)
	{
		kind_ = kind;
	}
  /** Get apiVersion value.
   * @return apiVersion value
   */
	std::optional<std::string>
 apiVersion() const
	{
		return apiVersion_;
	}

	/** Set apiVersion value.
	 * @param apiVersion new value
	 */
	void set_apiVersion(const std::string& apiVersion)
	{
		apiVersion_ = apiVersion;
	}
  /** Get id value.
   * @return id value
   */
	std::optional<std::string>
 id() const
	{
		return id_;
	}

	/** Set id value.
	 * @param id new value
	 */
	void set_id(const std::string& id)
	{
		id_ = id;
	}
  /** Get goal-id value.
   * @return goal-id value
   */
	std::optional<std::string>
 goal_id() const
	{
		return goal_id_;
	}

	/** Set goal-id value.
	 * @param goal_id new value
	 */
	void set_goal_id(const std::string& goal_id)
	{
		goal_id_ = goal_id;
	}
  /** Get cost value.
   * @return cost value
   */
	std::optional<float>
 cost() const
	{
		return cost_;
	}

	/** Set cost value.
	 * @param cost new value
	 */
	void set_cost(const float& cost)
	{
		cost_ = cost;
	}
  /** Get actions value.
   * @return actions value
   */
	std::vector<std::shared_ptr<PlanAction>>
 actions() const
	{
		return actions_;
	}

	/** Set actions value.
	 * @param actions new value
	 */
	void set_actions(const std::vector<std::shared_ptr<PlanAction>>& actions)
	{
		actions_ = actions;
	}
	/** Add element to actions array.
	 * @param actions new value
	 */
	void addto_actions(const std::shared_ptr<PlanAction>&& actions)
	{
		actions_.push_back(std::move(actions));
	}

	/** Add element to actions array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param actions new value
	 */
	void addto_actions(const std::shared_ptr<PlanAction>& actions)
	{
		actions_.push_back(actions);
	}
	/** Add element to actions array.
	 * @param actions new value
	 */
	void addto_actions(const PlanAction&& actions)
	{
		actions_.push_back(std::make_shared<PlanAction>(std::move(actions)));
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 id_;
	std::optional<std::string>
 goal_id_;
	std::optional<float>
 cost_;
	std::vector<std::shared_ptr<PlanAction>>
 actions_;

};