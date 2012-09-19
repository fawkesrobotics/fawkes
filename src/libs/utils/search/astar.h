
/***************************************************************************
 *  astar.h - Implementation of A*
 *
 *  Created: Mon Sep 15 18:39:00 2002
 *  Copyright  2007  Martin Liebenberg
 *             2002  Stefan Jacobs
 *             2012  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef _ABSTRACT_ASTAR_H_
#define _ABSTRACT_ASTAR_H_

#include <utils/search/astar_state.h>

#include <vector>
#include <map>
#include <queue>

namespace fawkes {

class AStar
{
 public:
  AStar ();
  ~AStar();

  std::vector<AStarState *> solve( AStarState * initialState );

 private:
  struct CmpSearchStateCost {
    bool operator() ( AStarState * a1, AStarState * a2 ) const
    { return (a1->total_estimated_cost >= a2->total_estimated_cost); }
  };
  
  std::priority_queue<AStarState *, std::vector<AStarState *>, CmpSearchStateCost> open_list;
  std::map<const size_t, AStarState*> closed_list;

  AStarState * search();
  
  std::vector<AStarState *> solution_sequence(AStarState * node);
  std::vector<AStarState *> solution;
};


} // end namespace fawkes

#endif
