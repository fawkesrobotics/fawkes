
/****************************************************************************
 *  BackendInfo -- Schema Backend
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Backend Info REST API.
 *  Provides backend meta information to the frontend.
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

#include "Service.h"


/** Backend representation for JSON transfer. */
class Backend

{
 public:
	/** Constructor. */
	Backend();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	Backend(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	Backend(const rapidjson::Value& v);

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

	// Schema: Backend
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
  /** Get url value.
   * @return url value
   */
	std::optional<std::string>
 url() const
	{
		return url_;
	}

	/** Set url value.
	 * @param url new value
	 */
	void set_url(const std::string& url)
	{
		url_ = url;
	}
  /** Get services value.
   * @return services value
   */
	std::vector<std::shared_ptr<Service>>
 services() const
	{
		return services_;
	}

	/** Set services value.
	 * @param services new value
	 */
	void set_services(const std::vector<std::shared_ptr<Service>>& services)
	{
		services_ = services;
	}
	/** Add element to services array.
	 * @param services new value
	 */
	void addto_services(const std::shared_ptr<Service>&& services)
	{
		services_.push_back(std::move(services));
	}

	/** Add element to services array.
	 * The move-semantics version (std::move) should be preferred.
	 * @param services new value
	 */
	void addto_services(const std::shared_ptr<Service>& services)
	{
		services_.push_back(services);
	}
	/** Add element to services array.
	 * @param services new value
	 */
	void addto_services(const Service&& services)
	{
		services_.push_back(std::make_shared<Service>(std::move(services)));
	}
 private:
	std::optional<std::string>
 kind_;
	std::optional<std::string>
 apiVersion_;
	std::optional<std::string>
 id_;
	std::optional<std::string>
 name_;
	std::optional<std::string>
 url_;
	std::vector<std::shared_ptr<Service>>
 services_;

};