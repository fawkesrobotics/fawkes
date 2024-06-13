
/****************************************************************************
 *  Clips -- Schema Fact
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS REST API.
 *  Enables access to CLIPS environments.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#pragma once

#define RAPIDJSON_HAS_STDSTRING 1
#include "SlotValue.h"

#include <rapidjson/fwd.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/** Fact representation for JSON transfer. */
class Fact

{
public:
	/** Constructor. */
	Fact();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	Fact(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	Fact(const rapidjson::Value &v);

	/** Destructor. */
	virtual ~Fact();

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

	// Schema: Fact
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
	/** Get index value.
   * @return index value
   */
	std::optional<int64_t>
	index() const
	{
		return index_;
	}

	/** Set index value.
	 * @param index new value
	 */
	void
	set_index(const int64_t &index)
	{
		index_ = index;
	}
	/** Get template_name value.
   * @return template_name value
   */
	std::optional<std::string>
	template_name() const
	{
		return template_name_;
	}

	/** Set template_name value.
	 * @param template_name new value
	 */
	void
	set_template_name(const std::string &template_name)
	{
		template_name_ = template_name;
	}
	/** Get formatted value.
   * @return formatted value
   */
	std::optional<std::string>
	formatted() const
	{
		return formatted_;
	}

	/** Set formatted value.
	 * @param formatted new value
	 */
	void
	set_formatted(const std::string &formatted)
	{
		formatted_ = formatted;
	}
	/** Get slots value.
   * @return slots value
   */
	std::vector<std::shared_ptr<SlotValue>>
	slots() const
	{
		return slots_;
	}

	/** Set slots value.
	 * @param slots new value
	 */
	void
	set_slots(const std::vector<std::shared_ptr<SlotValue>> &slots)
	{
		slots_ = slots;
	}
	/** Add element to slots array.
	 * @param slots new value
	 */
	void
	addto_slots(const std::shared_ptr<SlotValue> &&slots)
	{
		slots_.push_back(std::move(slots));
	}

	/** Add element to slots array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param slots new value
	 */
	void
	addto_slots(const std::shared_ptr<SlotValue> &slots)
	{
		slots_.push_back(slots);
	}
	/** Add element to slots array.
	 * @param slots new value
	 */
	void
	addto_slots(const SlotValue &&slots)
	{
		slots_.push_back(std::make_shared<SlotValue>(std::move(slots)));
	}

private:
	std::optional<std::string>              kind_;
	std::optional<std::string>              apiVersion_;
	std::optional<int64_t>                  index_;
	std::optional<std::string>              template_name_;
	std::optional<std::string>              formatted_;
	std::vector<std::shared_ptr<SlotValue>> slots_;
};
