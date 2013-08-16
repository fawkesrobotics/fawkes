
/***************************************************************************
 *  clips_feature.cpp - CLIPS feature aspect for Fawkes
 *
 *  Created: Thu Jul 25 17:37:58 2013
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

#include <plugins/clips/aspect/clips_feature.h>
#include <clipsmm.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CLIPSFeatureAspect <plugins/clips/aspect/clips_feature.h>
 * Thread aspect to provide a feature to CLIPS environments.
 * Give this aspect to your thread if you want to provide a CLIPS
 * feature (library) to other threads which have the CLIPSAspect.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @fn void CLIPSFeatureAspect::clips_context_init(const std::string &env_name, fawkes::LockPtr<CLIPS::Environment> &clips) = 0
 * Initialize a CLIPS context to use the provided feature.
 * This method must be implemented by threads with the
 * CLIPSFeatureAspect. It is called to initialize a particular CLIPS
 * environment that requests to use the provided feature.
 * @param env_name name of CLIPS environment to initialized.
 * @param clips CLIPS environment to initialize
 */

/** @fn void CLIPSFeatureAspect::clips_context_destroyed(const std::string &env_name) = 0
 * Notification that a CLIPS environment has been destroyed.
 * At this time the CLIPS environment can no longer be accessed. But the
 * event can be used to free internal resources that were associated with
 * the environment.
 * @param env_name name of destroyed CLIPS environment
 */

/** Constructor.
 * @param feature_name CLIPS feature name by which threads can request
 * access to the feature.
 */
CLIPSFeatureAspect::CLIPSFeatureAspect(const char *feature_name)
  : clips_feature_name(feature_name)
{
  add_aspect("CLIPSFeatureAspect");
}


/** Virtual empty destructor. */
CLIPSFeatureAspect::~CLIPSFeatureAspect()
{
}

} // end namespace fawkes
