
/***************************************************************************
 *  ownership.h - BlackBoard with traced ownership
 *
 *  Created: Thu Jan 22 15:16:15 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_OWNERSHIP_H_
#define __BLACKBOARD_OWNERSHIP_H_

#include <blackboard/blackboard.h>


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BlackBoardWithOwnership : public BlackBoard
{
 public:
  BlackBoardWithOwnership(BlackBoard *parent, const char *owner);
  virtual ~BlackBoardWithOwnership();

  virtual Interface *  open_for_reading(const char *interface_type,
					const char *identifier,
					const char *owner = NULL);
  virtual Interface *  open_for_writing(const char *interface_type,
					const char *identifier,
					const char *owner = NULL);
  virtual void         close(Interface *interface);

  virtual InterfaceInfoList *  list_all();
  virtual InterfaceInfoList *  list(const char *type_pattern,
				    const char *id_pattern);
  virtual bool                 is_alive() const throw();
  virtual bool                 try_aliveness_restore() throw();

  virtual std::list<Interface *>
    open_multiple_for_reading(const char *type_pattern,
			      const char *id_pattern = "*",
			      const char *owner = NULL);

  virtual void register_listener(BlackBoardInterfaceListener *listener,
                                 ListenerRegisterFlag flag = BBIL_FLAG_ALL);
  virtual void update_listener(BlackBoardInterfaceListener *listener,
                                 ListenerRegisterFlag flag = BBIL_FLAG_ALL);
  virtual void unregister_listener(BlackBoardInterfaceListener *listener);

  virtual void register_observer(BlackBoardInterfaceObserver *observer);
  virtual void unregister_observer(BlackBoardInterfaceObserver *observer);

 private: /* members */
  BlackBoard  *blackboard_;
  std::string  owner_;
};


} // end namespace fawkes

#endif
