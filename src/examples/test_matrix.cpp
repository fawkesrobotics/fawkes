
/***************************************************************************
 *  example_matrix.cpp - matrix test program
 *
 *  Created: Tue Jun 3 09:48:11 2006
 *  Copyright  2006-2008 Tobias Kellner
 *
 *  $Id$
 *
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

// Do not mention in API doc
/// @cond EXAMPLES

#include <libs/geometry/matrix.h>

int main(int argc, char *argv[])
{
	Matrix *a = new Matrix(4, 6);
	a->print_info("a1", "|", "\n");
	a->id();
	a->print_info("a2", "|", "\n");
	a->transpose();
	a->print_info("a3", "|", "\n");

	static float b_v[] = {1, 2, 3, 4, 5, 6, 7, 8};
	Matrix *b = new Matrix(4, 2, b_v);
	b->print_info("b1", "|", "\n");

	Matrix c = *a * *b;

	delete a;
	delete b;

	c.print_info("c=a*b", "|", "\n");

	Matrix d(c);

	c.id();

	d.print_info("d", "|", "\n");

}


/// @endcond
