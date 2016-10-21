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

#include <mongo/client/dbclient.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include "computable.h"

class ComputablesManager
{
  public:
    ComputablesManager(fawkes::Logger* logger, fawkes::Configuration* config,
      mongo::DBClientBase* mongodb_client, fawkes::Clock* clock);
    virtual ~ComputablesManager();

    void check_and_compute(mongo::Query query);
    void remove_computable(Computable* computable);

    /**
     * Registers a Computable which provides information in the robot memory that is computed on demand.
     *
     * @param identifyer BSONObj describing what the function computes. Yor computable is called when an new query matches the key value fields in the identifiyer.
     * @param compute_func Callback function that computes the information
     * @param obj Pointer to class the callback is a function of (usaually this)
     * @return Computable Object pointer used for removing it
     */
    template<typename T>
    Computable* register_computable(mongo::BSONObj identifyer, void(T::*compute_func)(mongo::BSONObj), T *obj)
    {
      return NULL;
    }

  private:
    std::string name = "RobotMemory ComputablesManager";
    fawkes::Logger* logger_;
    fawkes::Configuration* config_;
    mongo::DBClientBase* mongodb_client_;
    fawkes::Clock* clock_;

    std::map<mongo::BSONObj, Computable*> computables;
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_COMPUTABLES_MANAGER_H_ */
