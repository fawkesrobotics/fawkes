
/***************************************************************************
 *  clips.cpp - CLIPS aspect for Fawkes
 *
 *  Created: Sat Jun 16 14:30:44 2012
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include <plugins/clips/aspect/clips.h>
#include <clipsmm.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CLIPSAspect <plugins/clips/aspect/clips.h>
 * Thread aspect to get access to a CLIPS environment.
 * Give this aspect to your thread to get a CLIPS environment for exclusive
 * usage.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes:LockPtr<CLIPS::Environment> CLIPSAspect::clips
 * CLIPS environment for exclusive usage.
 */

/** Constructor. */
CLIPSAspect::CLIPSAspect()
{
  add_aspect("CLIPSAspect");
}


/** Virtual empty destructor. */
CLIPSAspect::~CLIPSAspect()
{
}


/** Init CLIPS aspect.
 * This set the CLIPS environment.
 * It is guaranteed that this is called for a CLIPS Thread before start
 * is called (when running regularly inside Fawkes).
 * @param clips CLIPS environment
 */
void
CLIPSAspect::init_CLIPSAspect(LockPtr<CLIPS::Environment> clips)
{
  this->clips = clips;
}

/** Finalize CLIPS aspect.
 * This clears the CLIPS environment.
 */
void
CLIPSAspect::finalize_CLIPSAspect()
{
  clips.clear();
}

} // end namespace fawkes
