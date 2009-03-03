
/***************************************************************************
 *  transformable.h - Transformable interface
 *
 *  Created: Thu Oct 02 16:53:27 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __GEOMETRY_TRANSFORMABLE_H_
#define __GEOMETRY_TRANSFORMABLE_H_

#include <geometry/hom_transform.h>
#include <geometry/hom_coord.h>

#include <vector>

namespace fawkes {

class Transformable
{
  friend class HomTransform;

 public:
  Transformable();
  virtual ~Transformable();
  
 protected:
  void transform(const HomTransform& t);
  void add_primitive(HomCoord* c);
  void clear_primitives();

  virtual void register_primitives() =0;
  virtual void post_transform()      =0;

 private:
  std::vector<HomCoord*> m_primitives;
};

}

#endif /* __GEOMETRY_TRANSFORMABLE_H_ */
