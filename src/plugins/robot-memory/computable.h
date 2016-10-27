/***************************************************************************
 *  computable.h - Class holding information for a single computable
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

#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLE_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLE_H_

#include <mongo/client/dbclient.h>
#include <boost/function.hpp>

class Computable
{
  public:
    Computable(mongo::Query query_to_compute, std::string collection, const boost::function<std::list<mongo::BSONObj> (mongo::BSONObj)> &compute_function);
    virtual ~Computable();

    std::list<mongo::BSONObj> compute(mongo::BSONObj query);
    mongo::Query get_query();
    std::string get_collection();

  private:
    boost::function<std::list<mongo::BSONObj> (mongo::BSONObj)> compute_function;
    mongo::Query query_to_compute;
    std::string collection;
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLE_H_ */
