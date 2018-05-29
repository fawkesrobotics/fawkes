
/****************************************************************************
 *  ClipsExecutive -- Schema Goal
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



/** Goal representation for JSON transfer. */
class Goal

{
 public:
	/** Constructor. */
	Goal();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	Goal(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	Goal(const rapidjson::Value& v);

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

	// Schema: Goal
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
  /** Get class value.
   * @return class value
   */
	std::optional<std::string>
 _class() const
	{
		return _class_;
	}

	/** Set class value.
	 * @param _class new value
	 */
	void set__class(const std::string& _class)
	{
		_class_ = _class;
	}
  /** Get mode value.
   * @return mode value
   */
	std::optional<std::string>
 mode() const
	{
		return mode_;
	}

	/** Set mode value.
	 * @param mode new value
	 */
	void set_mode(const std::string& mode)
	{
		mode_ = mode;
	}
  /** Get outcome value.
   * @return outcome value
   */
	std::optional<std::string>
 outcome() const
	{
		return outcome_;
	}

	/** Set outcome value.
	 * @param outcome new value
	 */
	void set_outcome(const std::string& outcome)
	{
		outcome_ = outcome;
	}
  /** Get message value.
   * @return message value
   */
	std::optional<std::string>
 message() const
	{
		return message_;
	}

	/** Set message value.
	 * @param message new value
	 */
	void set_message(const std::string& message)
	{
		message_ = message;
	}
  /** Get parent value.
   * @return parent value
   */
	std::optional<std::string>
 parent() const
	{
		return parent_;
	}

	/** Set parent value.
	 * @param parent new value
	 */
	void set_parent(const std::string& parent)
	{
		parent_ = parent;
	}
  /** Get priority value.
   * @return priority value
   */
	std::optional<int64_t>
 priority() const
	{
		return priority_;
	}

	/** Set priority value.
	 * @param priority new value
	 */
	void set_priority(const int64_t& priority)
	{
		priority_ = priority;
	}
  /** Get parameters value.
   * @return parameters value
   */
	std::vector<std::string>
 parameters() const
	{
		return parameters_;
	}

	/** Set parameters value.
	 * @param parameters new value
	 */
	void set_parameters(const std::vector<std::string>& parameters)
	{
		parameters_ = parameters;
	}
	/** Add element to parameters array.
	 * @param parameters new value
	 */
	void addto_parameters(const std::string&& parameters)
	{
		parameters_.push_back(std::move(parameters));
	}

	/** Add element to parameters array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param parameters new value
	 */
	void addto_parameters(const std::string& parameters)
	{
		parameters_.push_back(parameters);
	}
  /** Get plans value.
   * @return plans value
   */
	std::vector<std::string>
 plans() const
	{
		return plans_;
	}

	/** Set plans value.
	 * @param plans new value
	 */
	void set_plans(const std::vector<std::string>& plans)
	{
		plans_ = plans;
	}
	/** Add element to plans array.
	 * @param plans new value
	 */
	void addto_plans(const std::string&& plans)
	{
		plans_.push_back(std::move(plans));
	}

	/** Add element to plans array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param plans new value
	 */
	void addto_plans(const std::string& plans)
	{
		plans_.push_back(plans);
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
 _class_;
	std::optional<std::string>
 mode_;
	std::optional<std::string>
 outcome_;
	std::optional<std::string>
 message_;
	std::optional<std::string>
 parent_;
	std::optional<int64_t>
 priority_;
	std::vector<std::string>
 parameters_;
	std::vector<std::string>
 plans_;

};