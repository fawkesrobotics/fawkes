/***************************************************************************
 *  fc_accum.h - Header for 'fitted circle' accumulator
 *               used by Randomized Stable Circle Fitting Algorithm
 *
 *  Generated: Fri Sep 09 2005 22:47:55
 *  Copyright  2005  Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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
                                                                                
#ifndef __FIREVISION_FC_ACCUM_H_
#define __FIREVISION_FC_ACCUM_H_

#include <fvutils/base/types.h>
#include <models/shape/circle.h>

class FittedCircle
{
private:
	static const float TOO_SMALL_DELTA;

private:
	int count;
	struct circle_matrix
	{
		float A00, A01, A02;
		float A10, A11, A12;
		float A20, A21, A22;

		float  b0,  b1,  b2;
	} circle_matrices[2];
	int current_circle;
	bool point_added;

public:
	FittedCircle(void);
	~FittedCircle(void);

	void reset(void);
	float addPoint(const point_t&); // add a new point
		// and return the distance from it to the fitted circle
	void removePoint(const point_t&); // remove a point

	float distanceTo(const point_t&, bool current = true);

	void commit(void);
	int getCount(void) const;
	Circle* getCircle(void) const;

private:
	Circle* fitCircle(circle_matrix* p) const;
};

#endif // __FIREVISION_FC_ACCUM_H_
