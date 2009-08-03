 
/***************************************************************************
 *  instance_factory.h - BlackBoard interface instance factory
 *
 *  Created: Mon Mar 03 17:59:59 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_INSTANCE_FACTORY_H_
#define __BLACKBOARD_INSTANCE_FACTORY_H_

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Interface;
class Module;
class ModuleManager;

class BlackBoardInstanceFactory
{
 public:

  BlackBoardInstanceFactory();
  ~BlackBoardInstanceFactory();

  Interface *  new_interface_instance(const char *type, const char *identifier);
  void         delete_interface_instance(Interface *interface);

 private:
  ModuleManager  *__mm;
};

} // end namespace fawkes

#endif
