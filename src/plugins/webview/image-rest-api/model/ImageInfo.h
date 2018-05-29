
/****************************************************************************
 *  Image -- Schema ImageInfo
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Image REST API.
 *  Access images through a REST API.
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



/** ImageInfo representation for JSON transfer. */
class ImageInfo

{
 public:
	/** Constructor. */
	ImageInfo();
	/** Constructor from JSON.
	 * @param json JSON string to initialize from
	 */
	ImageInfo(const std::string &json);
	/** Constructor from JSON.
	 * @param v RapidJSON value object to initialize from.
	 */
	ImageInfo(const rapidjson::Value& v);

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

	// Schema: ImageInfo
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
  /** Get colorspace value.
   * @return colorspace value
   */
	std::optional<std::string>
 colorspace() const
	{
		return colorspace_;
	}

	/** Set colorspace value.
	 * @param colorspace new value
	 */
	void set_colorspace(const std::string& colorspace)
	{
		colorspace_ = colorspace;
	}
  /** Get frame value.
   * @return frame value
   */
	std::optional<std::string>
 frame() const
	{
		return frame_;
	}

	/** Set frame value.
	 * @param frame new value
	 */
	void set_frame(const std::string& frame)
	{
		frame_ = frame;
	}
  /** Get width value.
   * @return width value
   */
	std::optional<int64_t>
 width() const
	{
		return width_;
	}

	/** Set width value.
	 * @param width new value
	 */
	void set_width(const int64_t& width)
	{
		width_ = width;
	}
  /** Get height value.
   * @return height value
   */
	std::optional<int64_t>
 height() const
	{
		return height_;
	}

	/** Set height value.
	 * @param height new value
	 */
	void set_height(const int64_t& height)
	{
		height_ = height;
	}
  /** Get mem_size value.
   * @return mem_size value
   */
	std::optional<int64_t>
 mem_size() const
	{
		return mem_size_;
	}

	/** Set mem_size value.
	 * @param mem_size new value
	 */
	void set_mem_size(const int64_t& mem_size)
	{
		mem_size_ = mem_size;
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
 colorspace_;
	std::optional<std::string>
 frame_;
	std::optional<int64_t>
 width_;
	std::optional<int64_t>
 height_;
	std::optional<int64_t>
 mem_size_;

};