
/***************************************************************************
 *  asp_inifin.h - Fawkes ASPAspect initializer/finalizer
 *
 *  Created: Thu Oct 20 15:49:31 2016
 *  Copyright  2016 Björn Schäpers
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

#ifndef __PLUGINS_ASP_ASPECT_ASP_INIFIN_H_
#define __PLUGINS_ASP_ASPECT_ASP_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include "asp.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ASPAspectIniFin : public AspectIniFin
{
	public:
	ASPAspectIniFin(void);

	void init(Thread *thread) override;
	void finalize(Thread *thread) override;
};

} // end namespace fawkes

#endif
