
/****************************************************************************
 *  ClipsExecutive -- Schema DomainPreconditionCompound
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

#include "DomainPrecondition.h"


/** DomainPreconditionCompound representation for JSON transfer. */
class DomainPreconditionCompound
: public DomainPrecondition

{
 public:
	/** Constructor. */
	DomainPreconditionCompound();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	DomainPreconditionCompound(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	DomainPreconditionCompound(const rapidjson::Value& v);

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

	// allOf
	// Schema: DomainPreconditionCompound[2]
 public:
  /** Get elements value.
   * @return elements value
   */
	std::vector<std::shared_ptr<DomainPrecondition>>
 elements() const
	{
		return elements_;
	}

	/** Set elements value.
	 * @param elements new value
	 */
	void set_elements(const std::vector<std::shared_ptr<DomainPrecondition>>& elements)
	{
		elements_ = elements;
	}
	/** Add element to elements array.
	 * @param elements new value
	 */
	void addto_elements(const std::shared_ptr<DomainPrecondition>&& elements)
	{
		elements_.push_back(std::move(elements));
	}

	/** Add element to elements array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param elements new value
	 */
	void addto_elements(const std::shared_ptr<DomainPrecondition>& elements)
	{
		elements_.push_back(elements);
	}
	/** Add element to elements array.
	 * @param elements new value
	 */
	void addto_elements(const DomainPrecondition&& elements)
	{
		elements_.push_back(std::make_shared<DomainPrecondition>(std::move(elements)));
	}
 private:
	std::vector<std::shared_ptr<DomainPrecondition>>
 elements_;

};