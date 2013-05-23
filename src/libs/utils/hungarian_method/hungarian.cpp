
/***************************************************************************
 *  hungarian.cpp - Hungarian Method
 *
 *  Created: Thu Apr 18 17:09:58 2013
 *  Copyright  2004  Cyrill Stachniss
 *             2008  Masrur Doostdar
 *             2008  Stefan Schiffer
 *             2013  Tim Niemueller [www.niemueller.de]
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

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <utils/hungarian_method/hungarian.h>

#define INF (0x7FFFFFFF)
#define verbose (0)

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#define hungarian_test_alloc(X) do {if ((void *)(X) == NULL)		\
      fprintf(stderr, "Out of memory in %s, (%s, line %d).\n",		\
	      __FUNCTION__, __FILE__, __LINE__); } while (0)

// 'faked' class member
//hungarian_problem_t hp;

/** @class HungarianMethod <utils/hungarian_method/hungarian.h>
 * Hungarian method assignment solver.
 * @author Stefan Schiffer
 */

/** Constructor. */
HungarianMethod::HungarianMethod()
{
  p            = (hungarian_problem_t *)malloc(sizeof(hungarian_problem_t));
  num_cols_   = 0;
  num_rows_   = 0;
  available_ = false;
}

/** Destructor. */
HungarianMethod::~HungarianMethod()
{
  ::free(p);
}

/** Print matrix to stdout.
 * @param C values
 * @param rows number of rows
 * @param cols number of columns
 */
void 
HungarianMethod::print_matrix( int** C, int rows, int cols ) 
{
  int i,j;
  std::cerr << std::endl;
  for(i=0; i<rows; i++) {
    std::cerr << " " << i << "'th row [" << std::flush;
    for(j=0; j<cols; j++) {
      std::cerr << C[i][j] << " ";
//       char s[5] = "\0";
//       sprintf( s, "%5d", C[i][j]);
//       std::cerr << s << " " << std::flush;
    }
    std::cerr << " ]" << std::endl << std::flush;
  }
  std::cerr << std::endl;
}

/** Convert an array to a matrix.
 * @param m array to convert
 * @param rows number of rows in array
 * @param cols number of columns in array
 * @return matrix from array
 */
int** 
HungarianMethod::array_to_matrix(int* m, int rows, int cols) 
{
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


/** Print the assignment matrix. */
void 
HungarianMethod::print_assignment() 
{
  if( available_ ) {
    std::cerr << "HungarianMethod: == Assignment ==" << std::endl;
    print_matrix(p->assignment, p->num_rows, p->num_cols) ;
  }
}

/** Print the cost matrix. */
void
HungarianMethod::print_cost_matrix() 
{
  if( available_ ) {
    std::cerr << "HungarianMethod: == CostMatrix ==" << std::endl;
    print_matrix(p->cost, p->num_rows, p->num_cols) ;
  }
}

/** Print the current status.
 * Prints cost matrix followed by assignment. */
void
HungarianMethod::print_status()
{
  print_cost_matrix();
  print_assignment();
}

/** Initialize hungarian method.
 * @param cost_matrix initial cost matrix
 * @param rows number of rows in matrix
 * @param cols number of columns in matrix
 * @param mode One of HUNGARIAN_MODE_MINIMIZE_COST and HUNGARIAN_MODE_MAXIMIZE_UTIL
 * @return number of rows in quadratic matrix
 */
int
HungarianMethod::init( int** cost_matrix, int rows, int cols, int mode ) 
{
//   std::cout << "HungarianMethod(init): entering ..." << std::endl;

  int i,j, org_cols, org_rows;
  int max_cost;
  max_cost = 0;
  
  org_cols = cols;
  org_rows = rows;

  // is the number of cols  not equal to number of rows ? 
  // if yes, expand with 0-cols / 0-cols
  rows = std::max(cols, rows);
  cols = rows;
  
  p->num_rows = rows;
  p->num_cols = cols;

  p->cost = (int**)calloc(rows,sizeof(int*));
  hungarian_test_alloc(p->cost);
  p->assignment = (int**)calloc(rows,sizeof(int*));
  hungarian_test_alloc(p->assignment);

  //std::cout << "HungarianMethod(init): loop rows" << std::endl;
  for(i=0; i<p->num_rows; i++) {
    p->cost[i] = (int*)calloc(cols,sizeof(int));
    hungarian_test_alloc(p->cost[i]);
    p->assignment[i] = (int*)calloc(cols,sizeof(int));
    hungarian_test_alloc(p->assignment[i]);
    for(j=0; j<p->num_cols; j++) {
      p->cost[i][j] =  (i < org_rows && j < org_cols) ? cost_matrix[i][j] : 0;
      p->assignment[i][j] = 0;

      if (max_cost < p->cost[i][j])
	max_cost = p->cost[i][j];
    }
  }


  if (mode == HUNGARIAN_MODE_MAXIMIZE_UTIL) {
    for(i=0; i<p->num_rows; i++) {
      for(j=0; j<p->num_cols; j++) {
	p->cost[i][j] =  max_cost - p->cost[i][j];
      }
    }
  }
  else if (mode == HUNGARIAN_MODE_MINIMIZE_COST) {
    // nothing to do
  }
  else 
    fprintf(stderr,"%s: unknown mode. Mode was set to HUNGARIAN_MODE_MINIMIZE_COST !\n", __FUNCTION__);

  // /////////////////////////////////////
  //std::cout << "HungarianMethod(init): init assignment save" << std::endl;
  //
  num_cols_ = cols;
  col_mates_ = (int*)calloc(cols,sizeof(int));
  hungarian_test_alloc(col_mates_);
  for( int j = 0; j < num_cols_; ++j ) {
    col_mates_[j] = -1;
  }
  //
  num_rows_ = rows;
  row_mates_ = (int*)calloc(rows,sizeof(int));
  hungarian_test_alloc(row_mates_);
  for( int i = 0; i < num_rows_; ++i ) {
    row_mates_[i] = -1;
  }
  // /////////////////////////////////////
  
//   std::cout << "HungarianMethod(init): ... leaving." << std::endl;
  return rows;
}

/** Free space alloacted by method. */
void
HungarianMethod::free() 
{
//   std::cout << "HungarianMethod(free): entering ..." << std::endl;
  int i;
  for(i=0; i<p->num_rows; i++) {
    ::free(p->cost[i]);
    ::free(p->assignment[i]);
  }
  ::free(p->cost);
  ::free(p->assignment);
  p->cost = NULL;
  p->assignment = NULL;
  //
  num_cols_ = 0;
  num_rows_ = 0;
  ::free( col_mates_ );
  ::free( row_mates_ );
  available_ = false;
//   std::cout << "HungarianMethod(free): ... leaving." << std::endl;
}


/** Solve the assignment problem.
 * This method computes the optimal assignment.
 */
void
HungarianMethod::solve()
{
  int i, j, m, n, k, l, s, t, q, unmatched, cost;
  int* col_mate;
  int* row_mate;
  int* parent_row;
  int* unchosen_row;
  int* row_dec;
  int* col_inc;
  int* slack;
  int* slack_row;

  cost=0;
  m =p->num_rows;
  n =p->num_cols;
  
  col_mate = (int*)calloc(p->num_rows,sizeof(int));
  hungarian_test_alloc(col_mate);
  unchosen_row = (int*)calloc(p->num_rows,sizeof(int));
  hungarian_test_alloc(unchosen_row);
  row_dec  = (int*)calloc(p->num_rows,sizeof(int));
  hungarian_test_alloc(row_dec);
  slack_row  = (int*)calloc(p->num_rows,sizeof(int));
  hungarian_test_alloc(slack_row);

  row_mate = (int*)calloc(p->num_cols,sizeof(int));
  hungarian_test_alloc(row_mate);
  parent_row = (int*)calloc(p->num_cols,sizeof(int));
  hungarian_test_alloc(parent_row);
  col_inc = (int*)calloc(p->num_cols,sizeof(int));
  hungarian_test_alloc(col_inc);
  slack = (int*)calloc(p->num_cols,sizeof(int));
  hungarian_test_alloc(slack);

  for (i=0;i<p->num_rows;i++) {
    col_mate[i]=0;
    unchosen_row[i]=0;
    row_dec[i]=0;
    slack_row[i]=0;
  }
  for (j=0;j<p->num_cols;j++) {
    row_mate[j]=0;
    parent_row[j] = 0;
    col_inc[j]=0;
    slack[j]=0;
  }

  for (i=0;i<p->num_rows;++i)
    for (j=0;j<p->num_cols;++j)
      p->assignment[i][j]=HUNGARIAN_NOT_ASSIGNED;

  // Begin subtract column minima in order to start with lots of zeroes 12
  if (verbose)
    fprintf(stderr, "Using heuristic\n");
  for (l=0;l<n;l++)
    {
      s=p->cost[0][l];
      for (k=1;k<m;k++) 
	if (p->cost[k][l]<s)
	  s=p->cost[k][l];
      cost+=s;
      if (s!=0)
	for (k=0;k<m;k++)
	  p->cost[k][l]-=s;
    }
  // End subtract column minima in order to start with lots of zeroes 12

  // Begin initial state 16
  t=0;
  for (l=0;l<n;l++)
    {
      row_mate[l]= -1;
      parent_row[l]= -1;
      col_inc[l]=0;
      slack[l]=INF;
    }
  for (k=0;k<m;k++)
    {
      s=p->cost[k][0];
      for (l=1;l<n;l++)
	if (p->cost[k][l]<s)
	  s=p->cost[k][l];
      row_dec[k]=s;
      for (l=0;l<n;l++)
	if (s==p->cost[k][l] && row_mate[l]<0)
	  {
	    col_mate[k]=l;
	    row_mate[l]=k;
	    if (verbose)
	      fprintf(stderr, "matching col %d==row %d\n",l,k);
	    goto row_done;
	  }
      col_mate[k]= -1;
      if (verbose)
	fprintf(stderr, "node %d: unmatched row %d\n",t,k);
      unchosen_row[t++]=k;
    row_done:
      ;
    }
  // End initial state 16
 
  // Begin Hungarian algorithm 18
  if (t==0)
    goto done;
  unmatched=t;
  while (1)
    {
      if (verbose)
	fprintf(stderr, "Matched %d rows.\n",m-t);
      q=0;
      while (1)
	{
	  while (q<t)
	    {
	      // Begin explore node q of the forest 19
	      {
		k=unchosen_row[q];
		s=row_dec[k];
		for (l=0;l<n;l++)
		  if (slack[l])
		    {
		      int del;
		      del=p->cost[k][l]-s+col_inc[l];
		      if (del<slack[l])
			{
			  if (del==0)
			    {
			      if (row_mate[l]<0)
				goto breakthru;
			      slack[l]=0;
			      parent_row[l]=k;
			      if (verbose)
				fprintf(stderr, "node %d: row %d==col %d--row %d\n",
				       t,row_mate[l],l,k);
			      unchosen_row[t++]=row_mate[l];
			    }
			  else
			    {
			      slack[l]=del;
			      slack_row[l]=k;
			    }
			}
		    }
	      }
	      // End explore node q of the forest 19
	      q++;
	    }
 
	  // Begin introduce a new zero into the matrix 21
	  s=INF;
	  for (l=0;l<n;l++)
	    if (slack[l] && slack[l]<s)
	      s=slack[l];
	  for (q=0;q<t;q++)
	    row_dec[unchosen_row[q]]+=s;
	  for (l=0;l<n;l++)
	    if (slack[l])
	      {
		slack[l]-=s;
		if (slack[l]==0)
		  {
		    // Begin look at a new zero 22
		    k=slack_row[l];
		    if (verbose)
		      fprintf(stderr, 
			     "Decreasing uncovered elements by %d produces zero at [%d,%d]\n",
			     s,k,l);
		    if (row_mate[l]<0)
		      {
			for (j=l+1;j<n;j++)
			  if (slack[j]==0)
			    col_inc[j]+=s;
			goto breakthru;
		      }
		    else
		      {
			parent_row[l]=k;
			if (verbose)
			  fprintf(stderr, "node %d: row %d==col %d--row %d\n",t,row_mate[l],l,k);
			unchosen_row[t++]=row_mate[l];
		      }
		    // End look at a new zero 22
		  }
	      }
	    else
	      col_inc[l]+=s;
	  // End introduce a new zero into the matrix 21
	}
    breakthru:
      // Begin update the matching 20
      if (verbose)
	fprintf(stderr, "Breakthrough at node %d of %d!\n",q,t);
      while (1)
	{
	  j=col_mate[k];
	  col_mate[k]=l;
	  row_mate[l]=k;
	  if (verbose)
	    fprintf(stderr, "rematching col %d==row %d\n",l,k);
	  if (j<0)
	    break;
	  k=parent_row[j];
	  l=j;
	}
      // End update the matching 20
      if (--unmatched==0)
	goto done;
      // Begin get ready for another stage 17
      t=0;
      for (l=0;l<n;l++)
	{
	  parent_row[l]= -1;
	  slack[l]=INF;
	}
      for (k=0;k<m;k++)
	if (col_mate[k]<0)
	  {
	    if (verbose)
	      fprintf(stderr, "node %d: unmatched row %d\n",t,k);
	    unchosen_row[t++]=k;
	  }
      // End get ready for another stage 17
    }
 done:

  // Begin doublecheck the solution 23
  for (k=0;k<m;k++)
    for (l=0;l<n;l++)
      if (p->cost[k][l]<row_dec[k]-col_inc[l]) {
        printf("boom1:  p->cost[%i][%i]=%i < row_dec[%i]-col_inc[%i]=%i\n", k, l, p->cost[k][l], k, l, row_dec[k]-col_inc[l]);
//	exit(0);
      }
  for (k=0;k<m;k++)
    {
      l=col_mate[k];
      if (l<0 || p->cost[k][l]!=row_dec[k]-col_inc[l]) {
        printf("boom2: %i<0 || p->cost[%i][%i]=%i != row_dec[%i]-col_inc[%i]=%i\n", l, k, l, p->cost[k][l], k, l, row_dec[k]-col_inc[l]);
//	exit(0);
      }
    }
  k=0;
  for (l=0;l<n;l++)
    if (col_inc[l])
      k++;
  if (k>m) {
    printf("boom3: %i > %i\n", k, m);
  //  exit(0);
  }
  // End doublecheck the solution 23
  // End Hungarian algorithm 18

  for (i=0;i<m;++i)
    {
      p->assignment[i][col_mate[i]]=HUNGARIAN_ASSIGNED;
      /*TRACE("%d - %d\n", i, col_mate[i]);*/
    }
  for (k=0;k<m;++k)
    {
      for (l=0;l<n;++l)
	{
	  /*TRACE("%d ",p->cost[k][l]-row_dec[k]+col_inc[l]);*/
	  p->cost[k][l]=p->cost[k][l]-row_dec[k]+col_inc[l];
	}
      /*TRACE("\n");*/
    }
  for (i=0;i<m;i++)
    cost+=row_dec[i];
  for (i=0;i<n;i++)
    cost-=col_inc[i];
  if (verbose)
    fprintf(stderr, "Cost is %d\n",cost);

  // /////////////////////////////////////
  // Save Assignment
  //
  for( int i = 0; i < num_rows_; ++i ) {
    row_mates_[i] = row_mate[i];
  }
  for( int j = 0; j < num_cols_; ++j ) {
    col_mates_[j] = col_mate[j];
  }
  // /////////////////////////////////////

  ::free(slack);
  ::free(col_inc);
  ::free(parent_row);
  ::free(row_mate);
  ::free(slack_row);
  ::free(row_dec);
  ::free(unchosen_row);
  ::free(col_mate);

  available_ = true;
}


/** Get column assignment.
 * @param col column index
 * @return column assignment, or -1 if @p col is out of bounds.
 */
int
HungarianMethod::get_column_assignment( const int & col )
{
  //std::cout << "HungarianMethod(get_column_assignment): for col '" << col << "'" << std::endl;
  if( col < num_cols_ ) {
    return (int)col_mates_[col];
  }
  return -1;
}


/** Get row assignment.
 * @param row row index
 * @return row assignment, or -1 if @p row is out of bounds.
 */
int
HungarianMethod::get_row_assignment( const int & row )
{
  //std::cout << "HungarianMethod(get_row_assignment): for row '" << row << "'" << std::endl;
  if( row < num_rows_ ) {
    return (int)row_mates_[row];
  }
  return -1;
}

/** Check if data is available.
 * solve done and not freed yet.
 * @return true if data is available, false otherwise
 */
bool
HungarianMethod::is_available()
{
  return available_;
}


/** Get assignment and size.
 * @param size number of rows/columns in quadratic matrix
 * @return pointer to columns.
 */
int *
HungarianMethod::get_assignment(int & size)
{
  size=p->num_rows;
  return  col_mates_;
}


} // end of namespace fawkes
