/* ***************************************************************************
 *                                                                           
 *                                            ####   ####           .-""-.   
 *       # #                             #   #    # #    #         /[] _ _\  
 *       # #                                 #    # #             _|_o_LII|_ 
 * ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \
 * #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_|
 * #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  || 
 * #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||'----'|| 
 * '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###   /__|    |__\
 *                                                                           
 * ***************************************************************************
 *
 *           $Id$
 *
 *   description: definition of object for hungarian method
 *
 *     re-author: Stefan Schiffer <mailto:schiffer[at]cs.rwth-aachen.de>
 *
 * ***************************************************************************
 *
 *   based on: libhungarian by Cyrill Stachniss, 2004
 *
 *   Solving the Minimum Assignment Problem using the 
 *   Hungarian Method.
 *
 *   ** This file may be freely copied and distributed! **
 *
 *   Parts of the used code was originally provided by the 
 *   "Stanford GraphGase", but I made changes to this code.
 *   As asked by  the copyright node of the "Stanford GraphGase", 
 *   I hereby proclaim that this file are *NOT* part of the
 *   "Stanford GraphGase" distrubition!
 *
 *   This file is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied 
 *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *   PURPOSE.  
 *
 * ***************************************************************************
 *
 * last modified: $Date$
 *            by: $Author$
 *
 * **************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "hungarian.h"

int** array_to_matrix(int* m, int rows, int cols) {
  int i,j;
  int** r;
  r = (int**)calloc(rows,sizeof(int*));
  for(i=0;i<rows;i++)
  {
    r[i] = (int*)calloc(cols,sizeof(int));
    for(j=0;j<cols;j++)
      r[i][j] = m[i*cols+j];
  }
  return r;
}


int 
main( int argc, char * argv[] ) 
{
  std::cout << "QAHungarian: creating HungarianMethod object" << std::endl;
  HungarianMethod * hungarian = new HungarianMethod();

  /* an example cost matrix */
  int r[4*3] =  {  100, 1, 1, 
		   100, 2, 2, 
		   1, 0, 0, 
		   0, 2, 0 };
  int** m = array_to_matrix(r,4,3);

  std::cout << "QAHungarian: init HungarianMethod object" << std::endl;
  /* initialize the hungarian_problem using the cost matrix*/
  int matrix_size = hungarian->Init( m , 4, 3, HUNGARIAN_MODE_MINIMIZE_COST );

  std::cout << "QAHungarian: Assignement matrix has a now a size '" << matrix_size << "' rows " 
	    << "and '" << matrix_size << "' columns." << std::endl;

  hungarian->PrintCostmatrix();

  std::cout << "QAHungarian: calling solve on HungarianMethod object" << std::endl;
  /* solve the assignement problem */
  hungarian->Solve();

  hungarian->PrintAssignment();

  std::cout << "QAHungarian: calling free on HungarianMethod object" << std::endl;
  /* free used memory */
  hungarian->Free();

  free(m);

  return 0;
}

