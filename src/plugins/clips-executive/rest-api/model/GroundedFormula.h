
/****************************************************************************
 *  ClipsExecutive -- Schema GroundedFormula
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
#include "GroundedFormula.h"

#include <rapidjson/fwd.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/** GroundedFormula representation for JSON transfer. */
class GroundedFormula
{
public:
	/** Constructor. */
	GroundedFormula();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	GroundedFormula(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	GroundedFormula(const rapidjson::Value &v);

	/** Destructor. */
	virtual ~GroundedFormula();

	/** Get version of implemented API.
	 * @return string representation of version
	 */
	static std::string
	api_version()
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
	virtual void to_json_value(rapidjson::Document &d, rapidjson::Value &v) const;
	/** Retrieve data from JSON string.
	 * @param json JSON representation suitable for this object.
	 * Will allow partial assignment and not validate automaticaly.
	 * @see validate()
	 */
	virtual void from_json(const std::string &json);
	/** Retrieve data from JSON string.
	 * @param v RapidJSON value suitable for this object.
	 * Will allow partial assignment and not validate automaticaly.
	 * @see validate()
	 */
	virtual void from_json_value(const rapidjson::Value &v);

	/** Validate if all required fields have been set.
	 * @param subcall true if this is called from another class, e.g.,
	 * a sub-class or array holder. Will modify the kind of exception thrown.
	 * @exception std::vector<std::string> thrown if required information is
	 * missing and @p subcall is set to true. Contains a list of missing fields.
	 * @exception std::runtime_error informative message describing the missing
	 * fields
	 */
	virtual void validate(bool subcall = false) const;

	// Schema: GroundedFormula
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
	void
	set_kind(const std::string &kind)
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
	void
	set_apiVersion(const std::string &apiVersion)
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
	void
	set_name(const std::string &name)
	{
		name_ = name;
	}
	/** Get type value.
   * @return type value
   */
	std::optional<std::string>
	type() const
	{
		return type_;
	}

	/** Set type value.
	 * @param type new value
	 */
	void
	set_type(const std::string &type)
	{
		type_ = type;
	}
	/** Get is-satisfied value.
   * @return is-satisfied value
   */
	std::optional<bool>
	is_satisfied() const
	{
		return is_satisfied_;
	}

	/** Set is-satisfied value.
	 * @param is_satisfied new value
	 */
	void
	set_is_satisfied(const bool &is_satisfied)
	{
		is_satisfied_ = is_satisfied;
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
	void
	set_param_names(const std::vector<std::string> &param_names)
	{
		param_names_ = param_names;
	}
	/** Add element to param-names array.
	 * @param param_names new value
	 */
	void
	addto_param_names(const std::string &&param_names)
	{
		param_names_.push_back(std::move(param_names));
	}

	/** Add element to param-names array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param param_names new value
	 */
	void
	addto_param_names(const std::string &param_names)
	{
		param_names_.push_back(param_names);
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
	void
	set_param_values(const std::vector<std::string> &param_values)
	{
		param_values_ = param_values;
	}
	/** Add element to param-values array.
	 * @param param_values new value
	 */
	void
	addto_param_values(const std::string &&param_values)
	{
		param_values_.push_back(std::move(param_values));
	}

	/** Add element to param-values array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param param_values new value
	 */
	void
	addto_param_values(const std::string &param_values)
	{
		param_values_.push_back(param_values);
	}
	/** Get param-constants value.
   * @return param-constants value
   */
	std::vector<std::string>
	param_constants() const
	{
		return param_constants_;
	}

	/** Set param-constants value.
	 * @param param_constants new value
	 */
	void
	set_param_constants(const std::vector<std::string> &param_constants)
	{
		param_constants_ = param_constants;
	}
	/** Add element to param-constants array.
	 * @param param_constants new value
	 */
	void
	addto_param_constants(const std::string &&param_constants)
	{
		param_constants_.push_back(std::move(param_constants));
	}

	/** Add element to param-constants array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param param_constants new value
	 */
	void
	addto_param_constants(const std::string &param_constants)
	{
		param_constants_.push_back(param_constants);
	}
	/** Get child value.
   * @return child value
   */
	std::vector<std::shared_ptr<GroundedFormula>>
	child() const
	{
		return child_;
	}

	/** Set child value.
	 * @param child new value
	 */
	void
	set_child(const std::vector<std::shared_ptr<GroundedFormula>> &child)
	{
		child_ = child;
	}
	/** Add element to child array.
	 * @param child new value
	 */
	void
	addto_child(const std::shared_ptr<GroundedFormula> &&child)
	{
		child_.push_back(std::move(child));
	}

	/** Add element to child array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param child new value
	 */
	void
	addto_child(const std::shared_ptr<GroundedFormula> &child)
	{
		child_.push_back(child);
	}
	/** Add element to child array.
	 * @param child new value
	 */
	void
	addto_child(const GroundedFormula &&child)
	{
		child_.push_back(std::make_shared<GroundedFormula>(std::move(child)));
	}

private:
	std::optional<std::string>                    kind_;
	std::optional<std::string>                    apiVersion_;
	std::optional<std::string>                    name_;
	std::optional<std::string>                    type_;
	std::optional<bool>                           is_satisfied_;
	std::vector<std::string>                      param_names_;
	std::vector<std::string>                      param_values_;
	std::vector<std::string>                      param_constants_;
	std::vector<std::shared_ptr<GroundedFormula>> child_;
};
