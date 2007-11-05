
/***************************************************************************
 *  binomial coefficient.cpp - a function for computing the binomial coefficient
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

#include <utils/math/binomial_coefficient.h>

/** @class BinomialCoefficient utils/math/binomialcoefficient.h
 *  Contains the methode to compute the binomial coefficient.
 * 
 *   @author Martin Liebenberg
 */


/** Calculates the binomial coefficient.
 * @param n upper value
 * @param k lower value
 * @return the binomial coefficient of n and k
 */
inline unsigned int BinomialCoefficient::binoc(unsigned int n, unsigned int k)
{
  unsigned int result;
  if(k == 0)
    return 1;
  if(2 * k > n)
    result = binoc(n, n - k);
  else
    {
      result = n;
      for(unsigned int i = 2; i <= k; i++)
        {
          result = result * ((n + 1 - i) / i);
        }
    }
  return result;
}
