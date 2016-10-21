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

/** @class ComputablesManager  computables_manager.h
 *  This class manages registering computables and can check
 *  if any computables are invoced by a query.
 * @author Frederik Zwilling
 */

ComputablesManager::ComputablesManager(fawkes::Logger* logger, fawkes::Configuration* config,
  mongo::DBClientBase* mongodb_client, fawkes::Clock* clock)
{
  logger_ = logger;
  config_ = config;
  mongodb_client_ = mongodb_client;
  clock_ = clock;
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

}
