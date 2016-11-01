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

/** @class ComputablesManager  computables_manager.h
 *  This class manages registering computables and can check
 *  if any computables are invoced by a query.
 * @author Frederik Zwilling
 */

using namespace fawkes;
using namespace mongo;

ComputablesManager::ComputablesManager(fawkes::Logger* logger, fawkes::Configuration* config,
  mongo::DBClientBase* mongodb_client, fawkes::Clock* clock)
{
  logger_ = logger;
  config_ = config;
  mongodb_client_ = mongodb_client;
  clock_ = clock;

  matching_test_collection_ = "robmem.computables_matching";
  try {
    matching_test_collection_ = config_->get_string("/plugins/robot-memory/database") + ".computables_matching";
  } catch (Exception &e) {}
}

ComputablesManager::~ComputablesManager()
{
}


/**
 * Remove previously registered computable
 * @param computable The computable to remove
 */
void ComputablesManager::remove_computable(Computable* computable)
{
  for(std::list<Computable*>::iterator it = computables.begin(); it != computables.end(); it++)
  {
    if((*it) == computable)
    {
      Computable* comp = *it;
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
bool ComputablesManager::check_and_compute(mongo::Query query, std::string collection)
{
  //logger_->log_info(name.c_str(), "checking query: %s", query.toString().c_str());
  bool added_computed_docs = false;
  //check if the query is matched by the computable identifyer
  //to do that we just insert the query as if it would be a document and query for it with the computable identifiers
  mongodb_client_->dropCollection(matching_test_collection_);
  mongodb_client_->insert(matching_test_collection_, query.obj);
  for(std::list<Computable*>::iterator it = computables.begin(); it != computables.end(); it++)
  {
    if(collection == (*it)->get_collection() &&  mongodb_client_->query(matching_test_collection_, (*it)->get_query())->more())
    {
      std::list<BSONObj> computed_docs_list = (*it)->compute(query.obj);
      if(computed_docs_list.size() > 0)
      {
        //move list into vector
        std::vector<BSONObj> computed_docs_vector{ std::make_move_iterator(std::begin(computed_docs_list)),
          std::make_move_iterator(std::end(computed_docs_list))};
        mongodb_client_->insert((*it)->get_collection(), computed_docs_vector);
        added_computed_docs = true;
      }
    }
  }
  if(added_computed_docs)
  {
    collections_to_cleanup.push_back(collection);
  }
  return added_computed_docs;
}

/**
 * Clean up all collections containing documents computed on demand
 */
void ComputablesManager::cleanup_computed_docs()
{
  for(std::string collection : collections_to_cleanup)
  {
    mongodb_client_->remove(collection, fromjson("{'_robmem_info.computed':true}"));
  }
  collections_to_cleanup.clear();
}
