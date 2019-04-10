/***************************************************************************
 *  computable.cpp - Class holding information for a single computable
 *    
 *
 *  Created: 6:57:45 PM 2016
 *  Copyright  2016  Frederik Zwilling
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

#include "computable.h"

#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/document/value.hpp>
#include <chrono>

using namespace bsoncxx;

/** @class Computable  computable.h
 * Class holding information for a single computable
 * this class also enhances computed documents by additional information, such as the caching time
 * @author Frederik Zwilling
 */

/**
 * Constructor for object holding information about a computable
 * @param query_to_compute Computable specification. Queries matching to this spec invoke the computable
 * @param collection Collection covered
 * @param compute_function Reference to the function providing the computation
 * @param caching_time How long should computed results for a query be cached and be used for identical queries in that time?
 * @param priority Computable priority ordering the evaluation
 */
Computable::Computable(
  bsoncxx::document::value query_to_compute,
  std::string              collection,
  const boost::function<std::list<document::value>(bsoncxx::document::view, std::string)>
    &    compute_function,
  double caching_time,
  int    priority)
: compute_function(compute_function), query_to_compute(query_to_compute), collection(collection)
{
	//convert caching time to milliseconds
	this->caching_time = (int)(caching_time * 1000.0);
	this->priority     = priority;
}

Computable::~Computable()
{
}

/**
 * Compute demanded information and insert it into the robot memory
 * @param query The query demanding the computable information
 * @return Documents to insert extended with computable meta information (e.g. caching time)
 */
std::list<bsoncxx::document::value>
Computable::compute(bsoncxx::document::view query)
{
	// use provided function to compute demanded documents
	std::list<bsoncxx::document::value> docs = compute_function(query, collection);
	int64_t                             milliseconds_since_epoch =
	  std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
	int64_t cached_until = milliseconds_since_epoch + caching_time;
	//add metainformation for each document
	for (auto obj : docs) {
		using namespace bsoncxx::builder;
		auto info = basic::document{};
		info.append(basic::kvp("computed", true));
		info.append(basic::kvp("cached_until", cached_until));

		basic::document doc;
		doc.append(concatenate(obj.view()));
		doc.append(basic::kvp("_robmem_info", info));
		//override
		obj = doc.extract();
	}
	return docs;
}

/**
 * Gets the query that defines what information is computed by the Computable
 * @return The query
 */
bsoncxx::document::value
Computable::get_query()
{
	return query_to_compute;
}

/**
 * Gets the collection the computable adds information to
 * @return The query
 */
std::string
Computable::get_collection()
{
	return collection;
}

/**
 * Gets the priority of the computable
 * @return The query
 */
int
Computable::get_priority()
{
	return priority;
}
