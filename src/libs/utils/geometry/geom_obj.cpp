
/***************************************************************************
 *  geom_obj.cpp - Geometric Object
 *
 *  Created: Fri Sep 28 11:50:26 2007
 *  Copyright  2007  Daniel Beck
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <utils/geometry/geom_obj.h>
#include <utils/geometry/transform.h>

using namespace std;


/** @class GeomObj libs/utils/geometry/geom_obj.h
 * Base class of all geometric objects (e.g. line, circle, cube, etc.).
 * Such objects are defined by means of a reference point and several
 * vectors which are specified wrt. the object's own CS. The transform
 * into the object's CS can be specified.
 */

/** @var GeomObj::mToRefCS
 * The transform into the reference CS.
 */

/** @var GeomObj::mRefPoint
 * The reference point allows to specify an offset in the object's CS
 * in order to move the object away from the origin.
 */

/** @var GeomObj::mVectorsLocal
 * The list of vectors specifying the object. Specified wrt. the
 * object's CS. An additional offest (mRefPoint) is be added.
 */

/** @var GeomObj::mVectorsRef
 * All the object-defining vectors transformed into the reference CS.
 */

/** @var GeomObj::mChanged
 * Is set to true everytime a new transform is applied to the object and
 * the vectors in mVectorsRef need to be recomputed.
 */


/**Constructor.
 * Just a reference point specified which means that the object's CS
 * is aligned in the same way as the reference CS. the objects's CS
 * origin is at the point specified as the reference point.
 * @param t the transform to the reference CS
 * @param ref_point the reference point of the object
 */
GeomObj::GeomObj(const Transform& t, const Point& ref_point)
{
  mToRefCS = t;
  mRefPoint = ref_point;

  mChanged = true;
}


/**Destructor */
GeomObj::~GeomObj()
{
}


/**Applies a transform to the object.
 * @param t the transform
 * @return a reference to *this
 */
GeomObj&
GeomObj::_apply_transform(const Transform& t)
{
  mToRefCS *= t;
  cout << mToRefCS << endl;
  mChanged = true;

  return *this;
}


/**Applies a transform to the object wrt. the reference CS.
 * @param t the transform
 * @return a reference to *this
 */
GeomObj&
GeomObj::_apply_transform_ref(const Transform& t)
{
  mToRefCS = t * mToRefCS;
  cout << mToRefCS << endl;
  mChanged = true;

  return *this;
}


/**Rotate the object around the x-axis of its CS.
 * @param angle the angle
 * @return a reference to *this
 */
GeomObj&
GeomObj::_rotate_x(float angle)
{
  Transform t;
  t.rotate_x(angle);
  mToRefCS *= t;

  mChanged = true;

  return *this;
}


/**Rotate the object around the y-axis of its CS.
 * @param angle the angle
 * @return a reference to *this
 */
GeomObj&
GeomObj::_rotate_y(float angle)
{
  Transform t;
  t.rotate_y(angle);
  mToRefCS *= t;

  mChanged = true;

  return *this;
}


/**Rotate the object around the z-axis of its CS.
 * @param angle the angle
 * @return a reference to *this
 */
GeomObj&
GeomObj::_rotate_z(float angle)
{
  Transform t;
  t.rotate_z(angle);
  mToRefCS *= t;

  mChanged = true;

  return *this;
}


/**Rotate the object around the x-axis of the reference CS.
 * @param angle the angle
 * @return a reference to *this
 */
GeomObj&
GeomObj::_rotate_x_ref(float angle)
{
  Transform t;
  t.rotate_x(angle);
  mToRefCS = t * mToRefCS;

  mChanged = true;

  return *this;
}


/**Rotate the object around the y-axis of the reference CS.
 * @param angle the angle
 * @return a reference to *this
 */
GeomObj&
GeomObj::_rotate_y_ref(float angle)
{
  Transform t;
  t.rotate_y(angle);
  mToRefCS = t * mToRefCS;

  mChanged = true;

  return *this;
}


/**Rotate the object around the z-axis of the reference CS.
 * @param angle the angle
 * @return a reference to *this
 */
GeomObj&
GeomObj::_rotate_z_ref(float angle)
{
  Transform t;
  t.rotate_z(angle);
  mToRefCS = t * mToRefCS;

  mChanged = true;

  return *this;
}


/**Translate the object wrt. its local CS.
 * @param trans_x translation along the x-axis
 * @param trans_y translation along the y-axis
 * @param trans_z translation along the z-axis
 * @return a reference to *this
 */
GeomObj&
GeomObj::_trans(float trans_x, float trans_y, float trans_z)
{
  mRefPoint.trans(trans_x, trans_y, trans_z);

  mChanged = true;

  return *this;
}


/**Translate the object wrt. the reference CS.
 * @param trans_x translation along the x-axis
 * @param trans_y translation along the y-axis
 * @param trans_z translation along the z-axis
 * @return a reference to *this
 */
GeomObj&
GeomObj::_trans_ref(float trans_x, float trans_y, float trans_z)
{
  Transform t;
  t.trans(trans_x, trans_y, trans_z);
  mToRefCS = mToRefCS * t;
  mChanged = true;

  return *this;
}


/**Returns the reference point wrt. the reference CS.
 * @return the reference point
 */
Point
GeomObj::_get_refpoint_ref() const
{
  return mToRefCS * mRefPoint;
}


/**Returns all the object's vector wrt. the local CS.
 * @return a list of vectors
 */
vector<Vector>
GeomObj::_get_vectors_local() const
{
  return mVectorsLocal;
}


/**Returns all the object's vectors wrt. the reference CS.
 * @return a list of vectors
 */
vector<Vector>
GeomObj::_get_vectors_ref()
{
  if (mChanged)
    {
      mVectorsRef.clear();

      vector<Vector>::iterator iter;
      for (iter = mVectorsLocal.begin(); iter != mVectorsLocal.end(); iter++)
	{
	  Vector v;
	  v = mToRefCS * (*iter);
	  mVectorsRef.push_back(v);
	}
    }

  return mVectorsRef;
}
