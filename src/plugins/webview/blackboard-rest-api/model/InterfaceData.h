
/****************************************************************************
 *  Blackboard -- Schema InterfaceData
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



/** InterfaceData representation for JSON transfer. */
class InterfaceData

{
 public:
	/** Constructor. */
	InterfaceData();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	InterfaceData(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	InterfaceData(const rapidjson::Value& v);

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

	// Schema: InterfaceData
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
  /** Get data value.
   * @return data value
   */
	std::shared_ptr<rapidjson::Document>
 data() const
	{
		return data_;
	}

	/** Set data value.
	 * @param data new value
	 */
	void set_data(const std::shared_ptr<rapidjson::Document>& data)
	{
		data_ = data;
	}
  /** Get timestamp value.
   * @return timestamp value
   */
	std::optional<std::string>
 timestamp() const
	{
		return timestamp_;
	}

	/** Set timestamp value.
	 * @param timestamp new value
	 */
	void set_timestamp(const std::string& timestamp)
	{
		timestamp_ = timestamp;
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
 writer_;
	std::vector<std::string>
 readers_;
	std::shared_ptr<rapidjson::Document>
 data_;
	std::optional<std::string>
 timestamp_;

};