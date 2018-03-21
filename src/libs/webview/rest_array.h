
/***************************************************************************
 *  reply.h - Web request reply
 *
 *  Created: Fri Mar 16 17:39:54 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __LIBS_WEBVIEW_REST_ARRAY_H_
#define __LIBS_WEBVIEW_REST_ARRAY_H_

#include <string>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

/** Container to return array via REST.
 * @author Tim Niemueller
 */
template<class M>
class WebviewRestArray
{
 public:
	/** Empty array constructor. */
	WebviewRestArray() {}
	/** Constructor.
	 * @param items vector of values to copy
	 */
	WebviewRestArray(std::vector<M> &  items)
		: items_(items)
	{}

	/** Constructor.
	 * @param items vector of values to move to this array.
	 */
	WebviewRestArray(std::vector<M> &&  items)
	  : items_(std::move(items))
	{}

	/** Render object to JSON.
	 * @param pretty true to enable pretty printing (readable spacing)
	 * @return JSON string
	 */
	std::string
	to_json(bool pretty = false) const
	{
		std::string rv = "[\n";
		for (size_t i = 0; i < items_.size(); ++i) {
			rv += items_[i].to_json(pretty);
			if (i < items_.size() - 1) {
				rv += ",";
			}
		}
		rv += "]";
		return rv;
	}

	/** Retrieve data from JSON string.
	 * @param json JSON representation suitable for this object.
	 * Will allow partial assignment and not validate automaticaly.
	 * @see validate()
	 */
	void from_json(const std::string& json)
	{
		std::stringstream ss(json);
		boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    for (auto& c : pt.get_child("")) {
	    std::stringstream os;
	    boost::property_tree::write_json(os, c.second);
	    M m;
	    m.from_json(os.str());
	    items_.push_back(std::move(m));
    }
	}

	/** Validate if all required fields have been set.
	 * @param subcall true if this is called from another class, e.g.,
	 * a sub-class or array holder. Will modify the kind of exception thrown.
	 * @exception std::vector<std::string> thrown if required information is
	 * missing and @p subcall is set to true. Contains a list of missing fields.
	 * @exception std::runtime_error informative message describing the missing
	 * fields
	 */
	void validate(bool subcall = false)
	{
		for (const auto &i : items_) {
			i.validate(subcall);
		}
	}

	/** Accessor for items.
	 * @return item vector
	 */
	std::vector<M> & items()
	{
		return items_;
	}

	/** Add item at the back of the container.
	 * @param m element to copy
	 */
	void push_back(M& m)
	{
		items_.push_back(m);
	}

	/** Add item at the back of the container.
	 * @param m element to move
	 */
	void push_back(M&& m)
	{
		items_.push_back(std::move(m));
	}

 private:
	std::vector<M> items_;
};

#endif
