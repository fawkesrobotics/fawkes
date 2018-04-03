
/****************************************************************************
 *  BehaviorEngine -- Schema SkillCall
 *  (auto-generated, do not modify directly)
 *
 *  Behavior Engine REST API.
 *  Visualize, monitor, and instruct the Skill Execution Run-Time of
 *  the Lua-based Behavior Engine.
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



/** SkillCall representation for JSON transfer. */
class SkillCall

{
 public:
	/** Constructor. */
	SkillCall();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	SkillCall(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	SkillCall(const rapidjson::Value& v);

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

	// Schema: SkillCall
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
  /** Get skill_string value.
   * @return skill_string value
   */
	std::optional<std::string>
 skill_string() const
	{
		return skill_string_;
	}

	/** Set skill_string value.
	 * @param skill_string new value
	 */
	void set_skill_string(const std::string& skill_string)
	{
		skill_string_ = skill_string;
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 skill_string_;

};