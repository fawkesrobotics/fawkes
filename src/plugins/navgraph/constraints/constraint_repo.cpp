/***************************************************************************
 *  constraint_repo.cpp - navgraph constraint repository
 *
 *  Created: Fr Mar 14 10:47:35 2014
 *  Copyright  2014  Sebastian Reuter
 *             2014  Tim Niemueller
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
#include <plugins/navgraph/constraints/constraint_repo.h>

#include <logging/logger.h>
#include <algorithm>

using namespace std;

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ConstraintRepo <plugins/navgraph/constraints/constraint_repo.h>
 * Constraint repository to maintain blocks on nodes.
 * @author Sebastian Reuter
 * @author Tim Niemueller
 */


/** Constructor.
 * @param logger logger used for debug output
 */
ConstraintRepo::ConstraintRepo(Logger *logger)
{
  logger_ = logger;
  modified_ = false;
}

/** Destructor. */
ConstraintRepo::~ConstraintRepo()
{
}


/** Register a constraint.
 * @param constraint constraint to register
 */
void
ConstraintRepo::register_constraint(NavGraphNodeConstraint* constraint)
{
  modified_ = true;

  constraints_.push_back(constraint);

  logger_->log_debug("Constraint Repo", "New Constraint %s registered.",
		     constraint->name().c_str());
}


/** Unregister a constraint by name.
 * @param name name of constraint to remove.
 */
void
ConstraintRepo::unregister_constraint(std::string name)
{
  modified_ = true;

  ConstraintList::iterator it =
    std::find_if(constraints_.begin(), constraints_.end(),
		 [&name](const NavGraphNodeConstraint *c) {
		   return *c == name;
		 });
  if (it != constraints_.end()) {
    logger_->log_debug("ConstraintRepo", "Unregistering constraint %s",
		       (*it)->name().c_str());
    constraints_.erase(it);
  }
}


/** Check by name if a constraint has been registered.
 * @param name name of constraint to look for
 * @return true if a constraint with the given name has been registered,
 * false otherwise
 */
bool
ConstraintRepo::has_constraint(std::string &name)
{
  ConstraintList::iterator it =
    std::find_if(constraints_.begin(), constraints_.end(),
		 [&name](const NavGraphNodeConstraint *c) {
		   return *c == name;
		 });
  return (it != constraints_.end());
}


/** Get a constraint by name.
 * @param name name of constraint to retrieve
 * @return if found returns a pointer to the constrained, NULL if not found
 */
fawkes::NavGraphNodeConstraint *
ConstraintRepo::get_constraint(std::string &name)
{
  ConstraintList::iterator it =
    std::find_if(constraints_.begin(), constraints_.end(),
		 [&name](const NavGraphNodeConstraint *c) {
		   return *c == name;
		 });
  if (it != constraints_.end()) {
    return *it;
  }

  return NULL;
}


/** Get a list of registered constraints.
 * @return list of constraints
 */
const ConstraintRepo::ConstraintList &
ConstraintRepo::constraints() const
{
  return constraints_;
}


/** Check if there are any constraints at all.
 * @return true if constraints have been registered, false otherwise
 */
bool
ConstraintRepo::has_constraints() const
{
  return (! constraints_.empty());
}


/** Call compute method on all registered constraints. */
void
ConstraintRepo::compute()
{
  modified_ = true;
  for (fawkes::NavGraphNodeConstraint *c : constraints_) {
    c->compute();
  }
}

bool
ConstraintRepo::modified(bool reset_modified)
{
  if (reset_modified) {
    bool rv = modified_;
    modified_ =false;
    return rv;
  } else {
    return modified_;
  }
}

} // namespace
