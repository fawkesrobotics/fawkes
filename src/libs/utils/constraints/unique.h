
/***************************************************************************
 *  unique.h - Uniqueness constraint
 *
 *  Created: Sun Feb 24 13:07:25 2008
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_CONSTRAINTS_UNIQUE_H_
#define __UTILS_CONSTRAINTS_UNIQUE_H_

#include <core/exception.h>
#include <cstddef>

namespace fawkes {


/** @class UniquenessViolationException <utils/constraints/unique.h>
 * Uniqueness violation exception.
 * Thrown if an operation is tried which would violate the uniqueness
 * constraint.
 * @see UniquenessConstraint
 * @ingroup Exceptions
 * @author Tim Niemueller
 */

class UniquenessViolationException : public Exception
{
 public:
  /** Contructor.
   * @param msg message
   */
  UniquenessViolationException(const char *msg) : Exception(msg) {}
};


/** @class UniquenessConstraint <utils/constraints/unique.h>
 * Uniqueness constraint.
 * This constraint keeps track of a resource that may exist at most once.
 *
 * The resource can only be added if no resource has been added and not been
 * removed before. A resource can always be removed.
 *
 * @author Tim Niemueller
 */

template <class ResourceType>
  class UniquenessConstraint
{
 public:
  UniquenessConstraint();

  void add(ResourceType *r);
  void remove(ResourceType *p);

  ResourceType *             resource();

 private:
  ResourceType *_resource;
};


/** Constructor. */
template <class ResourceType>
  UniquenessConstraint<ResourceType>::UniquenessConstraint()
{
  _resource = NULL;
}


/** Add resource.
 * This will add the resources or throw an exception if there is already a resource.
 * @param r resource object to add
 * @exception UniquenessViolationException thrown, if a second resource is added
 */
template <class ResourceType>
void
UniquenessConstraint<ResourceType>::add(ResourceType *r)
{
  if ( (_resource != NULL) && (r != _resource) ) {
    throw UniquenessViolationException("Different resource has already been added.");
  } else {
    _resource = r;
  }
}


/** Remove resource.
 * @param r resource object to remove
 */
template <class ResourceType>
void
UniquenessConstraint<ResourceType>::remove(ResourceType *r)
{
  if ( r == _resource )  _resource = NULL;
}

/** Get resource.
 * @return resource if set, NULL otherwise
 */
template <class ResourceType>
ResourceType *
UniquenessConstraint<ResourceType>::resource()
{
  return _resource;
}


} // end namespace fawkes

#endif
