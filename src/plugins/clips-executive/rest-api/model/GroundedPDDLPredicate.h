
/****************************************************************************
 *  ClipsExecutive -- Schema GroundedPDDLPredicate
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

/** GroundedPDDLPredicate representation for JSON transfer. */
class GroundedPDDLPredicate
{
public:
	/** Constructor. */
	GroundedPDDLPredicate();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	GroundedPDDLPredicate(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	GroundedPDDLPredicate(const rapidjson::Value &v);

	/** Destructor. */
	virtual ~GroundedPDDLPredicate();

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

	// Schema: GroundedPDDLPredicate
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
	/** Get predicate-id value.
   * @return predicate-id value
   */
	std::optional<std::string>
	predicate_id() const
	{
		return predicate_id_;
	}

	/** Set predicate-id value.
	 * @param predicate_id new value
	 */
	void
	set_predicate_id(const std::string &predicate_id)
	{
		predicate_id_ = predicate_id;
	}
	/** Get grounding value.
   * @return grounding value
   */
	std::optional<std::string>
	grounding() const
	{
		return grounding_;
	}

	/** Set grounding value.
	 * @param grounding new value
	 */
	void
	set_grounding(const std::string &grounding)
	{
		grounding_ = grounding;
	}
	/** Get is-satisfied value.
   * @return is-satisfied value
   */
	std::optional<bool>
	is_satisfied() const
	{
		return is_satisfied_;
	}

	/** Set is-satisfied value.
	 * @param is_satisfied new value
	 */
	void
	set_is_satisfied(const bool &is_satisfied)
	{
		is_satisfied_ = is_satisfied;
	}

private:
	std::optional<std::string> kind_;
	std::optional<std::string> apiVersion_;
	std::optional<std::string> id_;
	std::optional<std::string> predicate_id_;
	std::optional<std::string> grounding_;
	std::optional<bool>        is_satisfied_;
};
