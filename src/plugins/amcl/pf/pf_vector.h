
/***************************************************************************
 *  pf_vector.h: Vector functions
 *
 *  Created: Thu May 24 18:41:48 2012
 *  Copyright  2000  Brian Gerkey
 *             2000  Kasper Stoy
 *             2012  Tim Niemueller [www.niemueller.de]
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

/*  From:
 *  Player - One Hell of a Robot Server (LGPL)
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 */
/**************************************************************************
 * Desc: Vector functions
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 *************************************************************************/

#ifndef PF_VECTOR_H
#define PF_VECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

/// @cond EXTERNAL
  
// The basic vector
typedef struct
{
  double v[3];
} pf_vector_t;


// The basic matrix
typedef struct
{
  double m[3][3];
} pf_matrix_t;


// Return a zero vector
pf_vector_t pf_vector_zero();

// Check for NAN or INF in any component
int pf_vector_finite(pf_vector_t a);

// Print a vector
void pf_vector_fprintf(pf_vector_t s, FILE *file, const char *fmt);

// Simple vector addition
pf_vector_t pf_vector_add(pf_vector_t a, pf_vector_t b);

// Simple vector subtraction
pf_vector_t pf_vector_sub(pf_vector_t a, pf_vector_t b);

// Transform from local to global coords (a + b)
pf_vector_t pf_vector_coord_add(pf_vector_t a, pf_vector_t b);

// Transform from global to local coords (a - b)
pf_vector_t pf_vector_coord_sub(pf_vector_t a, pf_vector_t b);


// Return a zero matrix
pf_matrix_t pf_matrix_zero();

// Check for NAN or INF in any component
int pf_matrix_finite(pf_matrix_t a);

// Print a matrix
void pf_matrix_fprintf(pf_matrix_t s, FILE *file, const char *fmt);

// Compute the matrix inverse.  Will also return the determinant,
// which should be checked for underflow (indicated singular matrix).
//pf_matrix_t pf_matrix_inverse(pf_matrix_t a, double *det);

// Decompose a covariance matrix [a] into a rotation matrix [r] and a
// diagonal matrix [d] such that a = r * d * r^T.
void pf_matrix_unitary(pf_matrix_t *r, pf_matrix_t *d, pf_matrix_t a);

/// @endcond

#ifdef __cplusplus
}
#endif

#endif
