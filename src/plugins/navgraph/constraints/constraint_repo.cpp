/***************************************************************************
 *  constraint_repo.h
 *
 *  Created: Fr Mar 14 10:47:35 2014
 *  Copyright  2014  Sebastian Reuter
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
#include "constraint_repo.h"

using namespace std;

namespace fawkes{

ConstraintRepo::ConstraintRepo(Logger *logger){
	this->logger_ = logger;
}

ConstraintRepo::~ConstraintRepo(){
	delete logger_;
}

void
ConstraintRepo::register_constraint(AbstractNodeConstraint* constraint){

	constraintList_.push_back( constraint );

	logger_->log_info("Constraint Repo", "New Constraint %s registered.", constraint->get_constraint_name().c_str());
}

void
ConstraintRepo::unregister_constraint(std::string name){

	std::vector<fawkes::AbstractNodeConstraint*>::iterator it = constraintList_.begin();
	bool found = false;

	while( it != constraintList_.end() ){
		if( (*it)->get_constraint_name() == name ){
			logger_->log_info( "Constraint Repo", "Unregistering %s from ConstraintList.", (*it)->get_constraint_name().c_str() );
			found = true;
			it = constraintList_.erase(it);
			break;
		}
		else{
			++it;
		}
	}

	if(!found){
		logger_->log_error( "Constraint Repo", "Trying to unregister constraint %s from ConstraintList which is not in list", name.c_str() );
	}
}

fawkes::AbstractNodeConstraint*
ConstraintRepo::get_constraint(std::string name){

	for(unsigned int i=0; i< constraintList_.size(); i++){
		if( constraintList_[i]->get_constraint_name() == name ){
			return constraintList_[i];
		}
	}
	return NULL;
}

const std::vector<fawkes::AbstractNodeConstraint*> &
ConstraintRepo::constraints() const{

	return constraintList_;
}

} // namespace
