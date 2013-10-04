
/***************************************************************************
 *  aspect_provider.h - Aspect to provider a new aspect for Fawkes
 *
 *  Created: Thu Nov 25 12:08:21 2010 (Thanksgiving)
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include <aspect/aspect_provider.h>
#include <aspect/inifins/inifin.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class AspectProviderAspect <aspect/aspect_provider.h>
 * Thread aspect provide a new aspect.
 * Aspects in Fawkes are used to provide access to framework features.
 * More generally speaking they are used to provide access to features on the
 * C++ programming level. In some situations, it might be useful to provide
 * a custom aspect to allow for access to a resource on this level, e.g.
 * bypassing the blackboard for communication. In this case the
 * AspectProviderAspect can be used.
 *
 * Use this rarely! Be absolutely certain, that there is no better or equally
 * good way to implement a feature without a new aspect. If used it allows
 * for a well-defined way to exchange resources between threads (and even
 * plugins). Make sure that you define strong guarantees and keep them by
 * means of your aspect initializer/finalizer. For example if you share a
 * (pointer to a) resource, you <i>must</i> make sure, that this
 * resource stays as long as there is any user!
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** Constructor.
 * Add a single aspect.
 * @param inifin intializer/finalizer for the aspect. The inifin
 * must exist for the whole lifetime of this AspectProviderAspect instance!
 */
AspectProviderAspect::AspectProviderAspect(AspectIniFin *inifin)
{
  add_aspect("AspectProviderAspect");
  __aspect_provider_aspects.push_back(inifin);
}

/** Constructor.
 * Add multiple aspects.
 * @param aspects Map from aspect name to initializer/finalizer
 */
AspectProviderAspect::AspectProviderAspect(const std::list<AspectIniFin *> aspects)
{
  add_aspect("AspectProviderAspect");
  __aspect_provider_aspects = aspects;
}


/** Virtual empty destructor. */
AspectProviderAspect::~AspectProviderAspect()
{
}


/** Get name of the provided aspect.
 * @return name of the provided aspect
 */
const std::list<AspectIniFin *> &
AspectProviderAspect::aspect_provider_aspects() const
{
  return __aspect_provider_aspects;
}

} // end namespace fawkes
