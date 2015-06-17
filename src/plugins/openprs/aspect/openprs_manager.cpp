
/***************************************************************************
 *  openprs_manager.cpp - OpenPRS manager aspect for Fawkes
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

#include <plugins/openprs/aspect/openprs_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenPRSManagerAspect <plugins/openprs/aspect/openprs_manager.h>
 * Thread aspect access the OpenPRS kernel manager.

 * Give this aspect to your thread if you want to access the OpenPRS
 * kernel manager. Use this with extreme care and only if you
 * know what you are doing. If you want to create a OpenPRS kernel
 * to work with use the OpenPRSAspect. If you want to provide a OpenPRS
 * feature to other kernel use the OpenPRSFeatureAspect.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes::OpenPRSKernelManager OpenPRSManagerAspect::openprs_kernel_mgr
 * OpenPRS kenerl manager.
 */

/** Constructor. */
OpenPRSManagerAspect::OpenPRSManagerAspect()
{
  add_aspect("OpenPRSManagerAspect");
}


/** Virtual empty destructor. */
OpenPRSManagerAspect::~OpenPRSManagerAspect()
{
}

} // end namespace fawkes
