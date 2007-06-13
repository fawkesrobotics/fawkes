
/***************************************************************************
 *  astar.h - Implementation of A*
 *
 *  Generated: Mon Sep 15 18:39:00 2002
 *  Copyright  2002-2007  Stefan Jacobs, Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef _ABSTRACT_ASTAR_H_
#define _ABSTRACT_ASTAR_H_

#include <utils/search/astar_state.h>

#include <vector>
#include <map>
#include <queue>

using namespace std;


class AStar
{
 public:

  AStar ();

  ~AStar();

  std::vector< AStarState * > solve( AStarState * initialState );

 private:

  struct Cmp {
    bool operator() ( AStarState * a1, AStarState * a2 ) const
    { return (a1->totalEstimatedCost >= a2->totalEstimatedCost); }
  };
  
  std::priority_queue< AStarState *, std::vector< AStarState * >, Cmp > openList;
  
  std::map< const long, AStarState*> closedList;

  AStarState * search();
  
  std::vector< AStarState * > getSolutionSequence( AStarState * node );

  std::vector< AStarState * > solution;
  
};

#endif
