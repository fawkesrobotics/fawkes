/***************************************************************************
 *  computables_manager.cpp - Class managing registered computables and
 *                   checking if any computables are invoced by a query
 *
 *  Created: 6:37:45 PM 2016
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

#include "computables_manager.h"

#include <core/exception.h>
#include <plugins/robot-memory/robot_memory.h>

#include <chrono>

/** @class ComputablesManager  computables_manager.h
 *  This class manages registering computables and can check
 *  if any computables are invoced by a query.
 * @author Frederik Zwilling
 */

using namespace fawkes;
using namespace mongocxx;
using namespace bsoncxx;

/**
 * Constructor for class managing computables with refereces to plugin objects
 * @param config Configuration
 * @param robot_memory Robot Memory
 */
ComputablesManager::ComputablesManager(fawkes::Configuration *config, RobotMemory *robot_memory)
: config_(config),
  robot_memory_(robot_memory),
  matching_test_collection_("robmem.computables_matching")
{
	try {
		matching_test_collection_ =
		  config_->get_string("/plugins/robot-memory/database") + ".computables_matching";
	} catch (Exception &e) {
	}

	srand(time(NULL));
}

ComputablesManager::~ComputablesManager()
{
}

/**
 * Remove previously registered computable
 * @param computable The computable to remove
 */
void
ComputablesManager::remove_computable(Computable *computable)
{
	for (std::list<Computable *>::iterator it = computables.begin(); it != computables.end(); ++it) {
		if ((*it) == computable) {
			Computable *comp = *it;
			computables.erase(it);
			delete comp;
			return;
		}
	}
}

/**
 * Checks if computable knowledge is queried and calls the compute functions in this case
 * @param query The query that might ask for computable knowledge
 * @param collection The collection that is querried
 * @return Were computed documents added?
 */
bool
ComputablesManager::check_and_compute(const document::view &query, std::string collection)
{
	//check if computation result of the query is already cached
	for (std::map<std::tuple<std::string, std::string>, long long>::iterator it =
	       cached_querries_.begin();
	     it != cached_querries_.end();
	     ++it) {
		if (collection == std::get<0>(it->first) && to_json(query) == std::get<1>(it->first)) {
			return false;
		}
	}
	if (collection.find(matching_test_collection_) != std::string::npos)
		return false; //not necessary for matching test itself
	bool added_computed_docs = false;
	//check if the query is matched by the computable identifyer
	//to do that we just insert the query as if it would be a document and query for it with the computable identifiers
	std::string current_test_collection = matching_test_collection_ + std::to_string(rand());
	robot_memory_->insert(query, current_test_collection);
	for (std::list<Computable *>::iterator it = computables.begin(); it != computables.end(); ++it) {
		auto cursor = robot_memory_->query((*it)->get_query(), current_test_collection);
		if (collection == (*it)->get_collection() && cursor.begin() != cursor.end()) {
			std::list<document::value> computed_docs_list = (*it)->compute(query);
			if (!computed_docs_list.empty()) {
				//move list into vector
				std::vector<document::view> computed_docs_vector{
				  std::make_move_iterator(std::begin(computed_docs_list)),
				  std::make_move_iterator(std::end(computed_docs_list))};
				//remember how long a query is cached:
				long long cached_until =
				  computed_docs_vector[0]["_robmem_info"]["cached_until"].get_int64();
				cached_querries_[std::make_tuple(collection, to_json(query))] = cached_until;
				//TODO: fix minor problem: equivalent queries in different order jield unequal strings
				robot_memory_->insert(computed_docs_vector, (*it)->get_collection());
				added_computed_docs = true;
			}
		}
	}
	robot_memory_->drop_collection(current_test_collection);
	return added_computed_docs;
}

/**
 * Clean up all collections containing documents computed on demand
 */
void
ComputablesManager::cleanup_computed_docs()
{
	long long current_time_ms =
	  std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
	for (std::map<std::tuple<std::string, std::string>, long long>::iterator it =
	       cached_querries_.begin();
	     it != cached_querries_.end();
	     ++it) {
		if (current_time_ms > it->second) {
			using namespace bsoncxx::builder;
			basic::document doc;
			doc.append(basic::kvp("_robmem_info.computed", true));
			doc.append(
			  basic::kvp("_robmem_info.cached_until", [current_time_ms](basic::sub_document subdoc) {
				  subdoc.append(basic::kvp("$lt", static_cast<std::int64_t>(current_time_ms)));
			  }));
			robot_memory_->remove(doc, std::get<0>(it->first));
			cached_querries_.erase(it->first);
		}
	}
}
