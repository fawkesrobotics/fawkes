/***************************************************************************
 *  execution_time_estimator.cpp - Aspect for a execution time provider
 *
 *  Created: Thu 12 Dec 2019 19:03:19 CET 19:03
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "execution_time_estimator.h"

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <algorithm>

namespace fawkes {

/** @class ExecutionTimeEstimatorManager
 * A manager for execution time providers.
 * It stores prioritized providers, where the provider with the maximal
 * priority is considered first.
 */

/** Get the execution time provider for the given skill string.
 * @param skill_string The string to get the execution time for
 * @return a pointer to the provider
 * @throws IllegalArgumentException if no provider for the given skill exists
 */
std::shared_ptr<ExecutionTimeEstimator>
ExecutionTimeEstimatorManager::get_provider(const std::string &skill_string) const
{
	for (const auto &pair : execution_time_estimators_) {
		const auto &provider = pair.second;
		if (provider->can_execute(skill_string)) {
			return provider;
		}
	}
	throw IllegalArgumentException("No provider found for %s", skill_string.c_str());
}

/** Add an execution time provider.
 * @param provider The provider to add
 * @param priority The priority of the new provider
 */
void
ExecutionTimeEstimatorManager::register_provider(std::shared_ptr<ExecutionTimeEstimator> provider,
                                                 int                                     priority)
{
	execution_time_estimators_.insert(std::make_pair(priority, provider));
}

/** Remove an execution time estimate provider.
 * @param provider The provider to remove
 */
void
ExecutionTimeEstimatorManager::unregister_provider(std::shared_ptr<ExecutionTimeEstimator> provider)
{
#if __cplusplus >= 202002L
	std::erase_if(execution_time_estimators_, [&](auto &pair) { return provider == pair.second; });
#else
	for (auto it = execution_time_estimators_.begin(); it != execution_time_estimators_.end();) {
		if (it->second == provider) {
			it = execution_time_estimators_.erase(it);
		} else {
			it++;
		}
	}
#endif
}

/** @class ExecutionTimeEstimatorsAspect
 * An aspect to give access to the execution time estimator manager.
 * Use this aspect to add an execution time provider.
 *
 * @var ExecutionTimeEstimatorsAspect::execution_time_estimator_manager_
 * The ExecutionTimeEstimatorManager that is used to manage the estimators.
 * @see ExecutionTimeEstimatorManager
 */

/** Constructor. */
ExecutionTimeEstimatorsAspect::ExecutionTimeEstimatorsAspect()
: execution_time_estimator_manager_(nullptr)
{
	add_aspect("SkillExecutionTimeEstimatorAspect");
}

/** Initialize the aspect with a provider manager.
 * @param provider_manager The manager of the execution time providers
 */
void
ExecutionTimeEstimatorsAspect::init_ExecutionTimeEstimatorsAspect(
  ExecutionTimeEstimatorManager *provider_manager)
{
	execution_time_estimator_manager_ = provider_manager;
}

/** Finalize the aspect. */
void
ExecutionTimeEstimatorsAspect::finalize_ExecutionTimeEstimatorsAspect()
{
}

} // namespace fawkes
