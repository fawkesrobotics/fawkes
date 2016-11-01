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

using namespace mongo;

/** @class Computable  computable.h
 * Class holding information for a single computable
 * @author Frederik Zwilling
 */

Computable::Computable(Query query_to_compute, std::string collection, const boost::function<std::list<BSONObj> (BSONObj)> &compute_function)
{
  this->compute_function = compute_function;
  this->query_to_compute = query_to_compute;
  this->collection = collection;
}

Computable::~Computable()
{
}

/**
 * Compute demanded information and insert it into the robot memory
 * @param query The query demanding the computable information
 * @param mongodb_client Client to use to insert computed documents
 * @return Documents to insert extended with computable meta information (e.g. caching time)
 */
std::list<BSONObj> Computable::compute(BSONObj query)
{
  // use provided function to compute demanded documents
  std::list<BSONObj> docs = compute_function(query);
  //add metainformation for each document
  for(BSONObj &obj : docs)
  {
    BSONObjBuilder info_b;
    info_b.append("computed", true);
    BSONObjBuilder obj_b;
    obj_b.appendElements(obj);
    obj_b.append("_robmem_info", info_b.obj());
    //override
    obj = obj_b.obj();
  }
  return docs;
}

/**
 * Gets the query that defines what information is computed by the Computable
 * @return The query
 */
mongo::Query Computable::get_query()
{
  return query_to_compute;
}

/**
 * Gets the collection the computable adds information to
 * @return The query
 */
std::string Computable::get_collection()
{
  return collection;
}


