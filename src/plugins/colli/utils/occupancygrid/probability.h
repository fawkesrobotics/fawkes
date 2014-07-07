
/***************************************************************************
 *  probability.h - Probability typedef
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  AllemaniACs
 *             2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_UTILS_OCCUPANCYGRID_PROBABILITY_H_
#define __PLUGINS_COLLI_UTILS_OCCUPANCYGRID_PROBABILITY_H_

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** A probability type. */
typedef float Probability;

/** Check if the probability value is valid.
 * @param p The probablity
 * @return true if valid, false otherwise
 */
inline bool
isProb(Probability p)
{
  return ((p >= 0) && (p <= 1));
}

} // namespace fawkes

#endif
