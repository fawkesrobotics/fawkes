
/***************************************************************************
 *  mongodb.h - MongoDB aspect for Fawkes
 *
 *  Created: Mon Dec 06 00:24:43 2010
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

#ifndef __PLUGINS_MONGODB_ASPECT_MONGODB_H_
#define __PLUGINS_MONGODB_ASPECT_MONGODB_H_

#include <aspect/aspect.h>

#include <memory>

namespace mongo {
  class DBClientBase;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MongoDBAspect : public virtual Aspect
{
 public:
  MongoDBAspect();
  MongoDBAspect(const char *dbname, const char *user, const char *clearpwd);
  virtual ~MongoDBAspect();

  void init_MongoDBAspect(mongo::DBClientBase *mongodb_client);

 protected:
  std::auto_ptr<mongo::DBClientBase> mongodb_client;

 private:
  char *__mongodb_name;
  char *__mongodb_user;
  char *__mongodb_pass;
};

} // end namespace fawkes

#endif
