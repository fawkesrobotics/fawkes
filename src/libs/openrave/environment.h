
/***************************************************************************
 *  environment.h - Fawkes to OpenRAVE Environment
 *
 *  Created: Sun Sep 19 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef __OPENRAVE_ENVIRONMEN_H_
#define __OPENRAVE_ENVIRONMEN_H_

#include <boost/shared_ptr.hpp>
#include <string>

namespace OpenRAVE {
  class EnvironmentBase;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

typedef boost::shared_ptr< OpenRAVE::EnvironmentBase > EnvironmentBasePtr;
void SetViewer(EnvironmentBasePtr penv, const std::string& viewername);

class Logger;

class OpenRAVEEnvironment
{
 public:
  OpenRAVEEnvironment(fawkes::Logger* logger = 0);
  ~OpenRAVEEnvironment();

  virtual void create();
  virtual void destroy();
  virtual void lock();

  virtual void enableDebug();
  virtual void disableDebug();

  virtual void startViewer();

  virtual bool addRobot(const std::string& filename);


 private:
  const char*		__name;
  fawkes::Logger*	__logger;
  EnvironmentBasePtr	__env;
};
} // end of namespace fawkes

#endif