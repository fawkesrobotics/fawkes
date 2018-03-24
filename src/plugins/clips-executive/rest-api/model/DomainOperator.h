
/****************************************************************************
 *  ClipsExecutive -- Schema DomainOperator
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

#include "DomainOperatorParameter.h"


/** DomainOperator representation for JSON transfer. */
class DomainOperator

{
 public:
	/** Constructor. */
	DomainOperator();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	DomainOperator(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	DomainOperator(const rapidjson::Value& v);

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

	// Schema: DomainOperator
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
  /** Get name value.
   * @return name value
   */
	std::optional<std::string>
 name() const
	{
		return name_;
	}

	/** Set name value.
	 * @param name new value
	 */
	void set_name(const std::string& name)
	{
		name_ = name;
	}
  /** Get wait-sensed value.
   * @return wait-sensed value
   */
	std::optional<bool>
 wait_sensed() const
	{
		return wait_sensed_;
	}

	/** Set wait-sensed value.
	 * @param wait_sensed new value
	 */
	void set_wait_sensed(const bool& wait_sensed)
	{
		wait_sensed_ = wait_sensed;
	}
  /** Get parameters value.
   * @return parameters value
   */
	std::vector<std::shared_ptr<DomainOperatorParameter>>
 parameters() const
	{
		return parameters_;
	}

	/** Set parameters value.
	 * @param parameters new value
	 */
	void set_parameters(const std::vector<std::shared_ptr<DomainOperatorParameter>>& parameters)
	{
		parameters_ = parameters;
	}
	/** Add element to parameters array.
	 * @param parameters new value
	 */
	void addto_parameters(const std::shared_ptr<DomainOperatorParameter>&& parameters)
	{
		parameters_.push_back(std::move(parameters));
	}

	/** Add element to parameters array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param parameters new value
	 */
	void addto_parameters(const std::shared_ptr<DomainOperatorParameter>& parameters)
	{
		parameters_.push_back(parameters);
	}
	/** Add element to parameters array.
	 * @param parameters new value
	 */
	void addto_parameters(const DomainOperatorParameter&& parameters)
	{
		parameters_.push_back(std::make_shared<DomainOperatorParameter>(std::move(parameters)));
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 name_;
	std::optional<bool>
 wait_sensed_;
	std::vector<std::shared_ptr<DomainOperatorParameter>>
 parameters_;

};