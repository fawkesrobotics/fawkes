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

using namespace std;

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

ConstraintRepo::ConstraintRepo(Logger *logger)
{
  logger_ = logger;
}

ConstraintRepo::~ConstraintRepo()
{
}

void
ConstraintRepo::register_constraint(NavGraphNodeConstraint* constraint)
{
  constraints_.push_back( constraint );

  logger_->log_info("Constraint Repo", "New Constraint %s registered.",
		    constraint->name().c_str());
}

void
ConstraintRepo::unregister_constraint(std::string name)
{

  std::vector<fawkes::NavGraphNodeConstraint*>::iterator it = constraints_.begin();
  bool found = false;

  while( it != constraints_.end() ) {
    if( (*it)->name() == name ) {
      logger_->log_info( "Constraint Repo", "Unregistering %s from ConstraintList.",
			 (*it)->name().c_str() );
      found = true;
      it = constraints_.erase(it);
      break;
    } else {
      ++it;
    }
  }

  if(!found) {
    logger_->log_error( "Constraint Repo", "Trying to unregister constraint %s "
			"from ConstraintList which is not in list", name.c_str() );
  }
}

bool
ConstraintRepo::has_constraint(std::string name)
{

  for(unsigned int i=0; i< constraints_.size(); i++) {
    if( constraints_[i]->name() == name ) {
      return true;
    }
  }
  return false;

}

void
ConstraintRepo::add_node(std::string constraint_name, fawkes::TopologicalMapNode node)
{

  for(unsigned int i=0; i< constraints_.size(); i++) {
    if( constraints_[i]->name() == constraint_name ) {
      constraints_[i]->add_node(node);
      break;
    }
  }
}

void
ConstraintRepo::add_nodes(std::string constraint_name,
			  std::vector<fawkes::TopologicalMapNode> nodes)
{

  for(unsigned int i=0; i< constraints_.size(); i++) {
    if( constraints_[i]->name() == constraint_name ) {
      constraints_[i]->add_nodes(nodes);
      break;
    }
  }
}


fawkes::NavGraphNodeConstraint*
ConstraintRepo::get_constraint(std::string name)
{

  for(unsigned int i=0; i< constraints_.size(); i++) {
    if( constraints_[i]->name() == name ) {
      return constraints_[i];
    }
  }
  return NULL;
}

void
ConstraintRepo::override_nodes(std::string constraint_name,
			       std::vector<fawkes::TopologicalMapNode> &nodes)
{

  if( has_constraint(constraint_name) ) {
    logger_->log_error( "Constraint Repo", "Updating nodeList of constraint '%s'",
			constraint_name.c_str() );

    fawkes::NavGraphNodeConstraint *constraint = get_constraint(constraint_name);

    constraint->clear_nodes();
    for(unsigned int i=0; i< nodes.size(); i++) {
      constraint->add_node(nodes[i]);
    }
  } else {
    logger_->log_error( "Constraint Repo", "Trying to override constraint '%s' "
			"which is not registered yet", constraint_name.c_str() );
  }
}

const std::vector<fawkes::NavGraphNodeConstraint*> &
ConstraintRepo::constraints() const
{
  return constraints_;
}

} // namespace
