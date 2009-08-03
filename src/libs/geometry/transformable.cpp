
/***************************************************************************
 *  transformable.h - Transformable interface
 *
 *  Created: Thu Oct 02 16:53:27 2008
 *  Copyright  2008  Daniel Beck
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

#include <geometry/transformable.h>

namespace fawkes {

/** @class fawkes::Transformable <geometry/transformable.h> 
 * Interface class for all transformable objects. In order to be
 * tranformable by multiplying the geometric object with a
 * fawkes::HomTransform it should be derived from this class (although
 * it doesn't have to).
 *
 * @author Daniel Beck
 */

/** @fn void fawkes::Transformable::register_primitives()
 * Here, a derived class should register its primitives (HomPoints and
 * HomVectors) by calling add_primitive for each of those.
 */

/** @fn void fawkes::Transformable::post_transform()
 * This method is called after the primitives are transformed. Any
 * additional updates that need to be done should be done here.
 */

/** Constructor. */
Transformable::Transformable()
{
}


/** Destructor. */
Transformable::~Transformable()
{
}

/** Add a primitive to the list of primitives that is transformed.
* @param c a primitive (a HomCoord or an object of a derived class)
*/
void
Transformable::add_primitive(HomCoord* c)
{
  m_primitives.push_back(c);
}

/** Clear the list of primitives. */
void
Transformable::clear_primitives()
{
  m_primitives.clear();
}

/** Apply the transform to all registered primitives and call the
 * post_transform() method.
 * @param t a transform
 */
void
Transformable::transform(const HomTransform& t)
{
  std::vector<HomCoord*>::const_iterator iter;
  for ( iter  = m_primitives.begin();
	iter != m_primitives.end();
	++iter )
    { 
      HomCoord* c = *iter;
      c->transform(t); 
    }
  
  post_transform();
}

} // end namespace fawkes
