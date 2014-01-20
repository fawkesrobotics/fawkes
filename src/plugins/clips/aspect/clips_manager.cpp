
/***************************************************************************
 *  clips_manager.cpp - CLIPS manager aspect for Fawkes
 *
 *  Created: Thu Aug 15 18:52:36 2013
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

#include <plugins/clips/aspect/clips_manager.h>
#include <clipsmm.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CLIPSManagerAspect <plugins/clips/aspect/clips_manager.h>
 * Thread aspect access the CLIPS environment manager.

 * Give this aspect to your thread if you want to access the CLIPS
 * environment manager. Use this with extreme care and only if you
 * know what you are doing. If you want to create a CLIPS environment
 * to work with use the CLIPSAspect. If you want to provide a CLIPS
 * feature to other environments use the CLIPSFeatureAspect.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes::CLIPSEnvManager CLIPSManagerAspect::clips_env_mgr
 * CLIPS environment manager.
 */

/** Constructor. */
CLIPSManagerAspect::CLIPSManagerAspect()
{
  add_aspect("CLIPSManagerAspect");
}


/** Virtual empty destructor. */
CLIPSManagerAspect::~CLIPSManagerAspect()
{
}

} // end namespace fawkes
