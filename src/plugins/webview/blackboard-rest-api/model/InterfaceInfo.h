
/****************************************************************************
 *  Blackboard -- Schema InterfaceInfo
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
#include "InterfaceMessageType.h"


/** InterfaceInfo representation for JSON transfer. */
class InterfaceInfo

{
 public:
	/** Constructor. */
	InterfaceInfo();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	InterfaceInfo(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	InterfaceInfo(const rapidjson::Value& v);

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

	// Schema: InterfaceInfo
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
  /** Get hash value.
   * @return hash value
   */
	std::optional<std::string>
 hash() const
	{
		return hash_;
	}

	/** Set hash value.
	 * @param hash new value
	 */
	void set_hash(const std::string& hash)
	{
		hash_ = hash;
	}
  /** Get writer value.
   * @return writer value
   */
	std::optional<std::string>
 writer() const
	{
		return writer_;
	}

	/** Set writer value.
	 * @param writer new value
	 */
	void set_writer(const std::string& writer)
	{
		writer_ = writer;
	}
  /** Get readers value.
   * @return readers value
   */
	std::vector<std::string>
 readers() const
	{
		return readers_;
	}

	/** Set readers value.
	 * @param readers new value
	 */
	void set_readers(const std::vector<std::string>& readers)
	{
		readers_ = readers;
	}
	/** Add element to readers array.
	 * @param readers new value
	 */
	void addto_readers(const std::string&& readers)
	{
		readers_.push_back(std::move(readers));
	}

	/** Add element to readers array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param readers new value
	 */
	void addto_readers(const std::string& readers)
	{
		readers_.push_back(readers);
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
  /** Get message_types value.
   * @return message_types value
   */
	std::vector<std::shared_ptr<InterfaceMessageType>>
 message_types() const
	{
		return message_types_;
	}

	/** Set message_types value.
	 * @param message_types new value
	 */
	void set_message_types(const std::vector<std::shared_ptr<InterfaceMessageType>>& message_types)
	{
		message_types_ = message_types;
	}
	/** Add element to message_types array.
	 * @param message_types new value
	 */
	void addto_message_types(const std::shared_ptr<InterfaceMessageType>&& message_types)
	{
		message_types_.push_back(std::move(message_types));
	}

	/** Add element to message_types array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param message_types new value
	 */
	void addto_message_types(const std::shared_ptr<InterfaceMessageType>& message_types)
	{
		message_types_.push_back(message_types);
	}
	/** Add element to message_types array.
	 * @param message_types new value
	 */
	void addto_message_types(const InterfaceMessageType&& message_types)
	{
		message_types_.push_back(std::make_shared<InterfaceMessageType>(std::move(message_types)));
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 id_;
	std::optional<std::string>
 type_;
	std::optional<std::string>
 hash_;
	std::optional<std::string>
 writer_;
	std::vector<std::string>
 readers_;
	std::vector<std::shared_ptr<InterfaceFieldType>>
 fields_;
	std::vector<std::shared_ptr<InterfaceMessageType>>
 message_types_;

};