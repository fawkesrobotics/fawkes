
/***************************************************************************
 *  motion_thread.h - Katana one-time thread interface for motions
 *
 *  Created: Wed Jun 10 11:20:18 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *             2012-2014  Bahram Maleki-Fard
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_KATANA_MOTION_THREAD_H_
#define __PLUGINS_KATANA_MOTION_THREAD_H_

#include <core/threading/thread.h>
#include <logging/logger.h>
#include <core/utils/refptr.h>

namespace fawkes {
  class KatanaController;
}

class KatanaMotionThread
: public fawkes::Thread
{
 public:
  KatanaMotionThread(const char *thread_name,
		     fawkes::RefPtr<fawkes::KatanaController> katana, fawkes::Logger *logger);

  bool finished() const;
  virtual void reset();
  unsigned int error_code() const;

 protected:
  /** Katana object for interaction with the arm */
  fawkes::RefPtr<fawkes::KatanaController>  _katana;
  /** Set to true when motion is finished, to false on reset */
  bool                     _finished;
  /** Logger */
  fawkes::Logger          *_logger;
  /** Set to the desired error code on error */
  unsigned int             _error_code;

};


#endif
