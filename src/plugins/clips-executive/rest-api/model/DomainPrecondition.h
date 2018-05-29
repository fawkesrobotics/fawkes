
/****************************************************************************
 *  ClipsExecutive -- Schema DomainPrecondition
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



/** DomainPrecondition representation for JSON transfer. */
class DomainPrecondition

{
 public:
	/** Constructor. */
	DomainPrecondition();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	DomainPrecondition(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	DomainPrecondition(const rapidjson::Value& v);

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

	// Schema: DomainPrecondition
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
	void set_type(const std::string& type)
	{
		type_ = type;
	}
  /** Get grounded value.
   * @return grounded value
   */
	std::optional<bool>
 grounded() const
	{
		return grounded_;
	}

	/** Set grounded value.
	 * @param grounded new value
	 */
	void set_grounded(const bool& grounded)
	{
		grounded_ = grounded;
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
	void set_is_satisfied(const bool& is_satisfied)
	{
		is_satisfied_ = is_satisfied;
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 name_;
	std::optional<std::string>
 type_;
	std::optional<bool>
 grounded_;
	std::optional<bool>
 is_satisfied_;

};