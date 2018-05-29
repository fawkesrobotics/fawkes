
/****************************************************************************
 *  BehaviorEngine -- Schema Skill
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



/** Skill representation for JSON transfer. */
class Skill

{
 public:
	/** Constructor. */
	Skill();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	Skill(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	Skill(const rapidjson::Value& v);

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

	// Schema: Skill
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
  /** Get graph value.
   * @return graph value
   */
	std::optional<std::string>
 graph() const
	{
		return graph_;
	}

	/** Set graph value.
	 * @param graph new value
	 */
	void set_graph(const std::string& graph)
	{
		graph_ = graph;
	}
	/** The skill string is given only for the active skill.

   * @return skill-string value
   */
	std::optional<std::string>
 skill_string() const
	{
		return skill_string_;
	}

	/** Set skill-string value.
	 * @param skill_string new value
	 */
	void set_skill_string(const std::string& skill_string)
	{
		skill_string_ = skill_string;
	}
	/** An error is presented for the active skill if it has FAILED.

   * @return error value
   */
	std::optional<std::string>
 error() const
	{
		return error_;
	}

	/** Set error value.
	 * @param error new value
	 */
	void set_error(const std::string& error)
	{
		error_ = error;
	}
  /** Get msg_id value.
   * @return msg_id value
   */
	std::optional<int64_t>
 msg_id() const
	{
		return msg_id_;
	}

	/** Set msg_id value.
	 * @param msg_id new value
	 */
	void set_msg_id(const int64_t& msg_id)
	{
		msg_id_ = msg_id;
	}
  /** Get exclusive_controller value.
   * @return exclusive_controller value
   */
	std::optional<int64_t>
 exclusive_controller() const
	{
		return exclusive_controller_;
	}

	/** Set exclusive_controller value.
	 * @param exclusive_controller new value
	 */
	void set_exclusive_controller(const int64_t& exclusive_controller)
	{
		exclusive_controller_ = exclusive_controller;
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
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 name_;
	std::optional<std::string>
 graph_;
	std::optional<std::string>
 skill_string_;
	std::optional<std::string>
 error_;
	std::optional<int64_t>
 msg_id_;
	std::optional<int64_t>
 exclusive_controller_;
	std::optional<std::string>
 status_;

};