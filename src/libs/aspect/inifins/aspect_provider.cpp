
/***************************************************************************
 *  aspect_provider.cpp - Fawkes Aspect Provider initializer/finalizer
 *
 *  Created: Thu Nov 25 12:20:43 2010 (Thanksgiving, Pittsburgh)
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/inifins/aspect_provider.h>
#include <aspect/aspect_provider.h>
#include <aspect/manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class AspectProviderAspectIniFin <aspect/inifins/aspect_provider.h>
 * Initializer/finalizer for the AspectProviderAspect.
 * This initializer/finalizer will register the AspectIniFin instance with
 * the main aspect manager on init, and unregister it on finalization. it
 * will deny unloading if there are still threads using the provided aspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param manager aspect manager to register new aspects to
 */
AspectProviderAspectIniFin::AspectProviderAspectIniFin(AspectManager *manager)
  : AspectIniFin("AspectProviderAspect")
{
  __aspect_manager = manager;
}


void
AspectProviderAspectIniFin::init(Thread *thread)
{
  AspectProviderAspect *provider_thread;
  provider_thread = dynamic_cast<AspectProviderAspect *>(thread);

  if (provider_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "AspectProviderAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  const std::list<AspectIniFin *> &aspects =
    provider_thread->aspect_provider_aspects();
  std::list<AspectIniFin *>::const_iterator a;
  for (a = aspects.begin(); a != aspects.end(); ++a) {
    __aspect_manager->register_inifin(*a);
  }
}


bool
AspectProviderAspectIniFin::prepare_finalize(Thread *thread)
{
  AspectProviderAspect *p_thr;
  p_thr = dynamic_cast<AspectProviderAspect *>(thread);

  if (p_thr == NULL)  return true;

  const std::list<AspectIniFin *> &aspects = p_thr->aspect_provider_aspects();
  std::list<AspectIniFin *>::const_iterator a;
  for (a = aspects.begin(); a != aspects.end(); ++a) {
    if (__aspect_manager->has_threads_for_aspect((*a)->get_aspect_name())) {
      return false;
    }
  }

  return true;
}


void
AspectProviderAspectIniFin::finalize(Thread *thread)
{
  AspectProviderAspect *provider_thread;
  provider_thread = dynamic_cast<AspectProviderAspect *>(thread);

  if (provider_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"AspectProviderAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  const std::list<AspectIniFin *> &aspects =
    provider_thread->aspect_provider_aspects();
  std::list<AspectIniFin *>::const_iterator a;
  for (a = aspects.begin(); a != aspects.end(); ++a) {
    __aspect_manager->unregister_inifin(*a);  
  }
}


} // end namespace fawkes
