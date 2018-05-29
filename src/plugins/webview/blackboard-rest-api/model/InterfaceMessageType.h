
/****************************************************************************
 *  Blackboard -- Schema InterfaceMessageType
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

#include "InterfaceFieldType.h"


/** InterfaceMessageType representation for JSON transfer. */
class InterfaceMessageType

{
 public:
	/** Constructor. */
	InterfaceMessageType();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	InterfaceMessageType(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	InterfaceMessageType(const rapidjson::Value& v);

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

	// Schema: InterfaceMessageType
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
  /** Get fields value.
   * @return fields value
   */
	std::vector<std::shared_ptr<InterfaceFieldType>>
 fields() const
	{
		return fields_;
	}

	/** Set fields value.
	 * @param fields new value
	 */
	void set_fields(const std::vector<std::shared_ptr<InterfaceFieldType>>& fields)
	{
		fields_ = fields;
	}
	/** Add element to fields array.
	 * @param fields new value
	 */
	void addto_fields(const std::shared_ptr<InterfaceFieldType>&& fields)
	{
		fields_.push_back(std::move(fields));
	}

	/** Add element to fields array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param fields new value
	 */
	void addto_fields(const std::shared_ptr<InterfaceFieldType>& fields)
	{
		fields_.push_back(fields);
	}
	/** Add element to fields array.
	 * @param fields new value
	 */
	void addto_fields(const InterfaceFieldType&& fields)
	{
		fields_.push_back(std::make_shared<InterfaceFieldType>(std::move(fields)));
	}
 private:
	std::optional<std::string>
 name_;
	std::vector<std::shared_ptr<InterfaceFieldType>>
 fields_;

};