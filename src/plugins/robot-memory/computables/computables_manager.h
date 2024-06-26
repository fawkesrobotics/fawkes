/***************************************************************************
 *  computables_manager.h - Class managing registered computables and
 *                   checking if any computables are invoced by a query
 *
 *  Created: 6:37:44 PM 2016
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

#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_COMPUTABLES_MANAGER_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_COMPUTABLES_MANAGER_H_

#include "computable.h"

#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>

#include <boost/bind/bind.hpp>
#include <map>
#include <mongocxx/client.hpp>
#include <tuple>
#include <utility>

//forward declaration
class RobotMemory;

namespace fawkes {
#ifdef USE_TIMETRACKER
class TimeTracker;
#endif
} // namespace fawkes

using namespace boost::placeholders;

class ComputablesManager
{
public:
	ComputablesManager(fawkes::Configuration *config, RobotMemory *robot_memory);
	virtual ~ComputablesManager();

	bool check_and_compute(const bsoncxx::document::view &query, std::string collection);
	void remove_computable(Computable *computable);
	void cleanup_computed_docs();

	/**
     * Registers a Computable which provides information in the robot memory that is computed on demand.
     * @param query_to_compute Query describing what the function computes. Yor computable is called when an new query matches query_to_compute.
     * @param collection db.collection to fill with computed information
     * @param compute_func Callback function that computes the information and retruns a list of computed documents
     * @param obj Pointer to class the callback is a function of (usaually this)
     * @param caching_time How long should computed results for a query be cached and be used for identical queries in that time?
     * @param priority Computable priority ordering the evaluation
     * @return Computable Object pointer used for removing it
     */
	template <typename T>
	Computable *
	register_computable(bsoncxx::document::value &&query_to_compute,
	                    const std::string         &collection,
	                    std::list<bsoncxx::document::value> (
	                      T::*compute_func)(const bsoncxx::document::view &, const std::string &),
	                    T     *obj,
	                    double caching_time = 0.0,
	                    int    priority     = 0)
	{
		Computable *comp = new Computable(
		  query_to_compute, collection, boost::bind(compute_func, obj, _1, _2), caching_time, priority);
		//sort it into the right position
		std::list<Computable *>::iterator pos = computables.begin();
		while (pos != computables.end() && priority < (*pos)->get_priority())
			pos++;
		computables.insert(pos, comp);
		return comp;
	}

private:
	ComputablesManager(const ComputablesManager &other);

private:
	std::string            name = "RobotMemory ComputablesManager";
	fawkes::Configuration *config_;
	RobotMemory           *robot_memory_;

	std::list<Computable *> computables;
	std::string             matching_test_collection_;
	//cached querries as ((collection, querry), cached_until)
	std::map<std::tuple<std::string, std::string>, long long> cached_querries_;
#ifdef USE_TIMETRACKER
	fawkes::TimeTracker *tt_;
	unsigned int         tt_loopcount_;
	unsigned int         ttc_cleanup_;
	unsigned int         ttc_cleanup_inner_loop_;
	unsigned int         ttc_cleanup_remove_query_;
#endif
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_COMPUTABLES_MANAGER_H_ */
