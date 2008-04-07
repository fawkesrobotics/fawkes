
/***************************************************************************
 *  astar.cpp - Implementation of A*
 *
 *  Generated: Mon Sep 15 18:38:00 2002
 *  Copyright  2002-2007  Stefan Jacobs, Martin Liebenberg
 *
 *  $Id$
 *
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

#include <utils/search/astar.h>


/** @class AStar <utils/search/astar.h>
 *  This is an implementation of the A* search algorithm.
 * 
 * @author Stefan Jacobs, Martin Liebenberg
 */
/** @var AStar::closedList
 * 	 This is AStars closedList.	
 */
/** @var AStar::solution
 * 	This is the final solution vector.
 */
/** @var AStar::openList
 *   	This is AStars openlist.
 */
/** @struct AStar::Cmp <utils/search/astar.h>
 * Comparison structure to be used for the ordering on AStar::openList.
 * @fn AStar::Cmp::operator() ( AStarState * a1, AStarState * a2 ) const
 * 	The relation >= of this ordering.
 * @param a1 the left operand
 * @param a2 the right operand
 * @return true, if a1 <= b1, else false
 */
 
 
  /** Constructor.
   *  This is the constructor for the AStar Object.
   */
AStar::AStar()
{
  while ( openList.size() > 0 )
    {
      openList.pop();
    }
  closedList.clear();
  solution.clear();
}

  /** Destructor.
   *  This destructs the AStarObject.
   */
AStar::~AStar()
{
  
   AStarState * best = 0;
  while ( openList.size() > 0 )
    {
      best = openList.top();
      openList.pop();
      delete best;
    }
  closedList.clear();
}

  /** Solves a situation given by the initial state with AStar, and
   *  returns a vector of AStarStates that solve the problem.
   * @param initialState pointer of AStarState to the initial state
   * @return a vector of pointers of AStarState with the solution sequence
   */
std::vector< AStarState * > AStar::solve( AStarState * initialState )
{
  AStarState * best = 0;
  while ( openList.size() > 0 )
    {
      best = openList.top();
      openList.pop();
      delete best;
    }
  closedList.clear();

  openList.push( initialState );
  return getSolutionSequence( search() );
}


/** Search with astar. */
AStarState * AStar::search( )
{
  AStarState * best = 0;
  long key = 0;
  std::vector< AStarState * > children;

  // while the openlist not is empty
  while ( openList.size() > 0 )
    {
      // take the best state, and check if it is on closed list
      do
	{
	  if ( openList.size() > 0 )
	    {
	      best = openList.top();
	      openList.pop( );
	    }
	  else
	    return 0;
	  key = best->calculateKey( );
	}
      while ( closedList.find( key ) != closedList.end() );
      
      // put best state on closed list
      closedList[key] = best;
      
      // check if its a goal.
      if ( best->isGoal( ) ) 
       {
	    return best;
       }
      // generate all its children
      children = best->generateChildren( );
      for ( unsigned int i = 0; i < children.size(); i++ )
	   openList.push( children[i] );
    }
  return 0;
}


  /** Generates a solution sequence for a given state
   * Initial solution is in solution[0]!
   * @param node a pointer of AStarState to the goal
   * @return the path from solution to initial solution
   */
std::vector< AStarState * > AStar::getSolutionSequence( AStarState * node )
{
  solution.clear();
  AStarState * state = node;
  
  while ( state != 0 )
    {
      closedList.erase(state->key);
      solution.insert( solution.begin(), state );
      state = state->father;
    }
   
   //delete the states, which are not part of the solution
   while ( closedList.size() > 0 )
    {
      state = closedList.begin()->second;
      closedList.erase(state->key);
      delete state;
     }
  return solution;
}
