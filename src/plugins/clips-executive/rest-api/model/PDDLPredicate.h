
/****************************************************************************
 *  ClipsExecutive -- Schema PDDLPredicate
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

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/** PDDLPredicate representation for JSON transfer. */
class PDDLPredicate
{
public:
	/** Constructor. */
	PDDLPredicate();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	PDDLPredicate(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	PDDLPredicate(const rapidjson::Value &v);

	/** Destructor. */
	virtual ~PDDLPredicate();

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

	// Schema: PDDLPredicate
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
	void
	set_id(const std::string &id)
	{
		id_ = id;
	}
	/** Get part-of value.
   * @return part-of value
   */
	std::optional<std::string>
	part_of() const
	{
		return part_of_;
	}

	/** Set part-of value.
	 * @param part_of new value
	 */
	void
	set_part_of(const std::string &part_of)
	{
		part_of_ = part_of;
	}
	/** Get predicate value.
   * @return predicate value
   */
	std::optional<std::string>
	predicate() const
	{
		return predicate_;
	}

	/** Set predicate value.
	 * @param predicate new value
	 */
	void
	set_predicate(const std::string &predicate)
	{
		predicate_ = predicate;
	}
	/** Get param-names value.
   * @return param-names value
   */
	std::vector<std::string>
	param_names() const
	{
		return param_names_;
	}

	/** Set param-names value.
	 * @param param_names new value
	 */
	void
	set_param_names(const std::vector<std::string> &param_names)
	{
		param_names_ = param_names;
	}
	/** Add element to param-names array.
	 * @param param_names new value
	 */
	void
	addto_param_names(const std::string &&param_names)
	{
		param_names_.push_back(std::move(param_names));
	}

	/** Add element to param-names array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param param_names new value
	 */
	void
	addto_param_names(const std::string &param_names)
	{
		param_names_.push_back(param_names);
	}
	/** Get param-constants value.
   * @return param-constants value
   */
	std::vector<std::string>
	param_constants() const
	{
		return param_constants_;
	}

	/** Set param-constants value.
	 * @param param_constants new value
	 */
	void
	set_param_constants(const std::vector<std::string> &param_constants)
	{
		param_constants_ = param_constants;
	}
	/** Add element to param-constants array.
	 * @param param_constants new value
	 */
	void
	addto_param_constants(const std::string &&param_constants)
	{
		param_constants_.push_back(std::move(param_constants));
	}

	/** Add element to param-constants array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param param_constants new value
	 */
	void
	addto_param_constants(const std::string &param_constants)
	{
		param_constants_.push_back(param_constants);
	}

private:
	std::optional<std::string> kind_;
	std::optional<std::string> apiVersion_;
	std::optional<std::string> id_;
	std::optional<std::string> part_of_;
	std::optional<std::string> predicate_;
	std::vector<std::string>   param_names_;
	std::vector<std::string>   param_constants_;
};
