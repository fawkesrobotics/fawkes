
/***************************************************************************
 *  ownership.cpp - BlackBoard with traced ownership
 *
 *  Created: Thu Jan 22 15:19:03 2015
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

#include <blackboard/ownership.h>

#include <string>
#include <cstring>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BlackBoardWithOwnership <blackboard/ownership.h>
 * BlackBoard that traces interface ownership.
 *
 * @see Interface
 * @see Message
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param parent parent blackboard to use as actual blackboard
 * @param owner owner name to record in newly created interfaces
 */
BlackBoardWithOwnership::BlackBoardWithOwnership(fawkes::BlackBoard *parent, const char *owner)
  : BlackBoard(/* create notifier */ false), blackboard_(parent), owner_(owner)
{
	BlackBoardWithOwnership *bbo = dynamic_cast<BlackBoardWithOwnership *>(blackboard_);
	if (bbo) {
		// we are wrapping another ownership, remove indirection and make sure
		// we do use the outer wrapper's ownership info
		blackboard_ = bbo->blackboard_;
	}
}


/** Destructor. */
BlackBoardWithOwnership::~BlackBoardWithOwnership()
{
}


Interface *
BlackBoardWithOwnership::open_for_reading(const char *type, const char *identifier, const char *owner)
{
  return blackboard_->open_for_reading(type, identifier,
				       owner ? owner : owner_.c_str());
}


Interface *
BlackBoardWithOwnership::open_for_writing(const char *type, const char *identifier, const char *owner)
{
  return blackboard_->open_for_writing(type, identifier,
				       owner ? owner : owner_.c_str());
}


std::list<Interface *>
BlackBoardWithOwnership::open_multiple_for_reading(const char *type_pattern,
							 const char *id_pattern,
							 const char *owner)
{
  return blackboard_->open_multiple_for_reading(type_pattern, id_pattern,
						owner ? owner : owner_.c_str());
}


void
BlackBoardWithOwnership::close(Interface *interface)
{
  blackboard_->close(interface);
}


InterfaceInfoList *
BlackBoardWithOwnership::list_all()
{
  return blackboard_->list_all();
}


InterfaceInfoList *
BlackBoardWithOwnership::list(const char *type_pattern, const char *id_pattern)
{
  return blackboard_->list(type_pattern, id_pattern);
}


bool
BlackBoardWithOwnership::is_alive() const throw()
{
  return blackboard_->is_alive();
}


bool
BlackBoardWithOwnership::try_aliveness_restore() throw()
{
  return blackboard_->try_aliveness_restore();
}

void
BlackBoardWithOwnership::register_listener(BlackBoardInterfaceListener *listener,
					   ListenerRegisterFlag flag)
{
  blackboard_->register_listener(listener, flag);
}

void
BlackBoardWithOwnership::update_listener(BlackBoardInterfaceListener *listener,
                            ListenerRegisterFlag flag)
{
  if (! listener)  return;
  blackboard_->update_listener(listener, flag);
}


void
BlackBoardWithOwnership::unregister_listener(BlackBoardInterfaceListener *listener)
{
  if (! listener) return;
  blackboard_->unregister_listener(listener);
}

void
BlackBoardWithOwnership::register_observer(BlackBoardInterfaceObserver *observer)
{
  if (! observer) return;
  blackboard_->register_observer(observer);
}


void
BlackBoardWithOwnership::unregister_observer(BlackBoardInterfaceObserver *observer)
{
  if (! observer) return;
  blackboard_->unregister_observer(observer);
}


} // end namespace fawkes
