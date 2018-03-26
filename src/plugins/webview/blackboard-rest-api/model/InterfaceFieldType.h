
/****************************************************************************
 *  Blackboard -- Schema InterfaceFieldType
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
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



/** InterfaceFieldType representation for JSON transfer. */
class InterfaceFieldType

{
 public:
	/** Constructor. */
	InterfaceFieldType();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	InterfaceFieldType(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	InterfaceFieldType(const rapidjson::Value& v);

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

	// Schema: InterfaceFieldType
 public:
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
  /** Get is_array value.
   * @return is_array value
   */
	std::optional<bool>
 is_array() const
	{
		return is_array_;
	}

	/** Set is_array value.
	 * @param is_array new value
	 */
	void set_is_array(const bool& is_array)
	{
		is_array_ = is_array;
	}
	/** possible enum values for this field
   * @return enums value
   */
	std::vector<std::string>
 enums() const
	{
		return enums_;
	}

	/** Set enums value.
	 * @param enums new value
	 */
	void set_enums(const std::vector<std::string>& enums)
	{
		enums_ = enums;
	}
	/** Add element to enums array.
	 * @param enums new value
	 */
	void addto_enums(const std::string&& enums)
	{
		enums_.push_back(std::move(enums));
	}

	/** Add element to enums array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param enums new value
	 */
	void addto_enums(const std::string& enums)
	{
		enums_.push_back(enums);
	}
 private:
	std::optional<std::string>
 name_;
	std::optional<std::string>
 type_;
	std::optional<bool>
 is_array_;
	std::vector<std::string>
 enums_;

};