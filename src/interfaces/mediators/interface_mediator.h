
/***************************************************************************
 *  interface_mediator.h - BlackBoard Interface Mediator
 *
 *  Generated: Tue Oct 17 15:35:45 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __INTERFACE_MEDIATOR_H_
#define __INTERFACE_MEDIATOR_H_

class Interface;

class InterfaceMediator
{
 public:
  /** Virtual destructor */
  virtual ~InterfaceMediator() {}

  /** Check if a writer exists for the given interface.
   * @param interface interface to check
   * @return true, if there is any writer for the given interface, false otherwise
   */
  virtual bool existsWriter(const Interface *interface) const           = 0;

  /** Notify of data change.
   * Notify all subscribers of the given interface of a data change.
   * This also influences logging and sending data over the network so it is
   * mandatory to call this function! The interface base class write method does
   * that for you.
   * @param interface interface whose subscribers to notify
   * @see Interface::write()
   */
  virtual void notifyOfDataChange(const Interface *interface)           = 0;
};

#endif
