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

#ifndef __CONSTRAINT_REPO_H_
#define __CONSTRAINT_REPO_H_

#include <vector>

#include "abstract_node_constraint.h"
#include "reserved_node_constraint.h"


namespace fawkes{

class Logger;

class ConstraintRepo {

public:
  ConstraintRepo(Logger *logger);
  ~ConstraintRepo();

public:
  void register_constraint(AbstractNodeConstraint* constraint);
  void unregister_constraint(std::string name);
  bool has_constraint(std::string name);

  void add_node(std::string constraint_name, fawkes::TopologicalMapNode node);
  void add_nodes(std::string constraint_name, std::vector<fawkes::TopologicalMapNode> node);
  fawkes::AbstractNodeConstraint* get_constraint(std::string name);

  void override_nodes(std::string constraint_name, std::vector<fawkes::TopologicalMapNode> nodes);

  const std::vector<fawkes::AbstractNodeConstraint*> &constraints() const;

private:
  std::vector<fawkes::AbstractNodeConstraint*> constraintList_;
  Logger *logger_;

};
} // namespace

#endif
