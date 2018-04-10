
/****************************************************************************
 *  Plugin -- Schema Plugin
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Plugin REST API.
 *  List, load, and unload plugins.
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



/** Plugin representation for JSON transfer. */
class Plugin

{
 public:
	/** Constructor. */
	Plugin();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	Plugin(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	Plugin(const rapidjson::Value& v);

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

	// Schema: Plugin
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
  /** Get description value.
   * @return description value
   */
	std::optional<std::string>
 description() const
	{
		return description_;
	}

	/** Set description value.
	 * @param description new value
	 */
	void set_description(const std::string& description)
	{
		description_ = description;
	}
  /** Get is_meta value.
   * @return is_meta value
   */
	std::optional<bool>
 is_meta() const
	{
		return is_meta_;
	}

	/** Set is_meta value.
	 * @param is_meta new value
	 */
	void set_is_meta(const bool& is_meta)
	{
		is_meta_ = is_meta;
	}
  /** Get meta_children value.
   * @return meta_children value
   */
	std::vector<std::string>
 meta_children() const
	{
		return meta_children_;
	}

	/** Set meta_children value.
	 * @param meta_children new value
	 */
	void set_meta_children(const std::vector<std::string>& meta_children)
	{
		meta_children_ = meta_children;
	}
	/** Add element to meta_children array.
	 * @param meta_children new value
	 */
	void addto_meta_children(const std::string&& meta_children)
	{
		meta_children_.push_back(std::move(meta_children));
	}

	/** Add element to meta_children array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param meta_children new value
	 */
	void addto_meta_children(const std::string& meta_children)
	{
		meta_children_.push_back(meta_children);
	}
  /** Get is_loaded value.
   * @return is_loaded value
   */
	std::optional<bool>
 is_loaded() const
	{
		return is_loaded_;
	}

	/** Set is_loaded value.
	 * @param is_loaded new value
	 */
	void set_is_loaded(const bool& is_loaded)
	{
		is_loaded_ = is_loaded;
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 name_;
	std::optional<std::string>
 description_;
	std::optional<bool>
 is_meta_;
	std::vector<std::string>
 meta_children_;
	std::optional<bool>
 is_loaded_;

};