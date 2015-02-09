
/***************************************************************************
 *  interface_mediator.h - BlackBoard Interface Mediator
 *
 *  Generated: Tue Oct 17 15:35:45 2006
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __INTERFACE_MEDIATOR_H_
#define __INTERFACE_MEDIATOR_H_

#include <list>
#include <string>

namespace fawkes {

class Interface;

/** Interface mediator interface.
 * An interface mediator is used by interfaces to communicate events and to query
 * status information which need interaction with the BlackBoard.
 * @author Tim Niemueller
 */
class InterfaceMediator
{
 public:
  /** Virtual destructor */
  virtual ~InterfaceMediator() {}

  /** Check if a writer exists for the given interface.
   * @param interface interface to check
   * @return true, if there is any writer for the given interface, false otherwise
   */
  virtual bool exists_writer(const Interface *interface) const           = 0;

  /** Get number of readers.
   * Get the number of readers that the given interface has.
   * @param interface interface to check
   * @return number of readers currently registered for the given interface.
   */
  virtual unsigned int num_readers(const Interface *interface) const     = 0;

  /** Get owners of interfaces who opened for reading.
   * @param interface an interface to query for the UID
   * @return list of readers for this interface
   */
  virtual std::list<std::string>  readers(const Interface *interface) const = 0;

  /** Get writer of interface.
   * @param interface an interface to query for the UID
   * @return owner name of writing interface instance, or empty string of no writer exists
   */
  virtual std::string             writer(const Interface *interface) const = 0;

  /** Notify of data change.
   * Notify all subscribers of the given interface of a data change.
   * This also influences logging and sending data over the network so it is
   * mandatory to call this function! The interface base class write method does
   * that for you.
   * @param interface interface whose subscribers to notify
   * @see Interface::write()
   */
  virtual void notify_of_data_change(const Interface *interface)         = 0;
};

} // end namespace fawkes

#endif
