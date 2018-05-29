
/****************************************************************************
 *  ClipsExecutive -- Schema PlanAction
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

#include "DomainOperator.h"
#include "DomainPrecondition.h"
#include "DomainEffect.h"


/** PlanAction representation for JSON transfer. */
class PlanAction

{
 public:
	/** Constructor. */
	PlanAction();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	PlanAction(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	PlanAction(const rapidjson::Value& v);

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

	// Schema: PlanAction
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
	std::optional<int64_t>
 id() const
	{
		return id_;
	}

	/** Set id value.
	 * @param id new value
	 */
	void set_id(const int64_t& id)
	{
		id_ = id;
	}
  /** Get operator-name value.
   * @return operator-name value
   */
	std::optional<std::string>
 operator_name() const
	{
		return operator_name_;
	}

	/** Set operator-name value.
	 * @param operator_name new value
	 */
	void set_operator_name(const std::string& operator_name)
	{
		operator_name_ = operator_name;
	}
  /** Get param-values value.
   * @return param-values value
   */
	std::vector<std::string>
 param_values() const
	{
		return param_values_;
	}

	/** Set param-values value.
	 * @param param_values new value
	 */
	void set_param_values(const std::vector<std::string>& param_values)
	{
		param_values_ = param_values;
	}
	/** Add element to param-values array.
	 * @param param_values new value
	 */
	void addto_param_values(const std::string&& param_values)
	{
		param_values_.push_back(std::move(param_values));
	}

	/** Add element to param-values array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param param_values new value
	 */
	void addto_param_values(const std::string& param_values)
	{
		param_values_.push_back(param_values);
	}
  /** Get duration value.
   * @return duration value
   */
	std::optional<float>
 duration() const
	{
		return duration_;
	}

	/** Set duration value.
	 * @param duration new value
	 */
	void set_duration(const float& duration)
	{
		duration_ = duration;
	}
  /** Get dispatch-time value.
   * @return dispatch-time value
   */
	std::optional<float>
 dispatch_time() const
	{
		return dispatch_time_;
	}

	/** Set dispatch-time value.
	 * @param dispatch_time new value
	 */
	void set_dispatch_time(const float& dispatch_time)
	{
		dispatch_time_ = dispatch_time;
	}
  /** Get status value.
   * @return status value
   */
	std::optional<std::string>
 status() const
	{
		return status_;
	}

	/** Set status value.
	 * @param status new value
	 */
	void set_status(const std::string& status)
	{
		status_ = status;
	}
  /** Get executable value.
   * @return executable value
   */
	std::optional<bool>
 executable() const
	{
		return executable_;
	}

	/** Set executable value.
	 * @param executable new value
	 */
	void set_executable(const bool& executable)
	{
		executable_ = executable;
	}
  /** Get operator value.
   * @return operator value
   */
	std::shared_ptr<DomainOperator>
 _operator() const
	{
		return _operator_;
	}

	/** Set operator value.
	 * @param _operator new value
	 */
	void set__operator(const std::shared_ptr<DomainOperator>& _operator)
	{
		_operator_ = _operator;
	}
  /** Get preconditions value.
   * @return preconditions value
   */
	std::vector<std::shared_ptr<DomainPrecondition>>
 preconditions() const
	{
		return preconditions_;
	}

	/** Set preconditions value.
	 * @param preconditions new value
	 */
	void set_preconditions(const std::vector<std::shared_ptr<DomainPrecondition>>& preconditions)
	{
		preconditions_ = preconditions;
	}
	/** Add element to preconditions array.
	 * @param preconditions new value
	 */
	void addto_preconditions(const std::shared_ptr<DomainPrecondition>&& preconditions)
	{
		preconditions_.push_back(std::move(preconditions));
	}

	/** Add element to preconditions array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param preconditions new value
	 */
	void addto_preconditions(const std::shared_ptr<DomainPrecondition>& preconditions)
	{
		preconditions_.push_back(preconditions);
	}
	/** Add element to preconditions array.
	 * @param preconditions new value
	 */
	void addto_preconditions(const DomainPrecondition&& preconditions)
	{
		preconditions_.push_back(std::make_shared<DomainPrecondition>(std::move(preconditions)));
	}
  /** Get effects value.
   * @return effects value
   */
	std::vector<std::shared_ptr<DomainEffect>>
 effects() const
	{
		return effects_;
	}

	/** Set effects value.
	 * @param effects new value
	 */
	void set_effects(const std::vector<std::shared_ptr<DomainEffect>>& effects)
	{
		effects_ = effects;
	}
	/** Add element to effects array.
	 * @param effects new value
	 */
	void addto_effects(const std::shared_ptr<DomainEffect>&& effects)
	{
		effects_.push_back(std::move(effects));
	}

	/** Add element to effects array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param effects new value
	 */
	void addto_effects(const std::shared_ptr<DomainEffect>& effects)
	{
		effects_.push_back(effects);
	}
	/** Add element to effects array.
	 * @param effects new value
	 */
	void addto_effects(const DomainEffect&& effects)
	{
		effects_.push_back(std::make_shared<DomainEffect>(std::move(effects)));
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<int64_t>
 id_;
	std::optional<std::string>
 operator_name_;
	std::vector<std::string>
 param_values_;
	std::optional<float>
 duration_;
	std::optional<float>
 dispatch_time_;
	std::optional<std::string>
 status_;
	std::optional<bool>
 executable_;
	std::shared_ptr<DomainOperator>
 _operator_;
	std::vector<std::shared_ptr<DomainPrecondition>>
 preconditions_;
	std::vector<std::shared_ptr<DomainEffect>>
 effects_;

};