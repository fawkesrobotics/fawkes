
/***************************************************************************
 *  binomial coefficient.h - a function for computing the binomial coefficient
 *
 *  Generated: Sun Nov 04 17:29:46 2007 
 *  Copyright  2007  Martin Liebenberg
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_MATH_BINOMIAL_COEFFICIENT_H_
#define __UTILS_MATH_BINOMIAL_COEFFICIENT_H_

class BinomialCoefficient
  {
  public:

    static unsigned int binoc(unsigned int n, unsigned int k);
  };

#endif
