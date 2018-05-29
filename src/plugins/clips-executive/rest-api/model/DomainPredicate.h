
/****************************************************************************
 *  ClipsExecutive -- Schema DomainPredicate
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



/** DomainPredicate representation for JSON transfer. */
class DomainPredicate

{
 public:
	/** Constructor. */
	DomainPredicate();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	DomainPredicate(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	DomainPredicate(const rapidjson::Value& v);

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

	// Schema: DomainPredicate
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
  /** Get sensed value.
   * @return sensed value
   */
	std::optional<bool>
 sensed() const
	{
		return sensed_;
	}

	/** Set sensed value.
	 * @param sensed new value
	 */
	void set_sensed(const bool& sensed)
	{
		sensed_ = sensed;
	}
  /** Get param-names value.
   * @return param-names value
   */
	std::vector<std::string>
 param_names() const
	{
		return param_names_;
	}

	/** Set param-names value.
	 * @param param_names new value
	 */
	void set_param_names(const std::vector<std::string>& param_names)
	{
		param_names_ = param_names;
	}
	/** Add element to param-names array.
	 * @param param_names new value
	 */
	void addto_param_names(const std::string&& param_names)
	{
		param_names_.push_back(std::move(param_names));
	}

	/** Add element to param-names array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param param_names new value
	 */
	void addto_param_names(const std::string& param_names)
	{
		param_names_.push_back(param_names);
	}
  /** Get param-types value.
   * @return param-types value
   */
	std::vector<std::string>
 param_types() const
	{
		return param_types_;
	}

	/** Set param-types value.
	 * @param param_types new value
	 */
	void set_param_types(const std::vector<std::string>& param_types)
	{
		param_types_ = param_types;
	}
	/** Add element to param-types array.
	 * @param param_types new value
	 */
	void addto_param_types(const std::string&& param_types)
	{
		param_types_.push_back(std::move(param_types));
	}

	/** Add element to param-types array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param param_types new value
	 */
	void addto_param_types(const std::string& param_types)
	{
		param_types_.push_back(param_types);
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 name_;
	std::optional<bool>
 sensed_;
	std::vector<std::string>
 param_names_;
	std::vector<std::string>
 param_types_;

};