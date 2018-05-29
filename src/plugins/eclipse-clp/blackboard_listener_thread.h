/***************************************************************************
 *  blackboard_listener_thread.h - Convert blackboard events to eclipse terms
 *
 *  Copyright  2017  Victor Mataré
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

#ifndef BLACKBOARD_LISTENER_THREAD_H
#define BLACKBOARD_LISTENER_THREAD_H


#include <aspect/blackboard.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <libs/blackboard/interface_observer.h>
#include <libs/blackboard/interface_listener.h>

#include <map>
#include <queue>
#include <memory>

#include <eclipseclass.h>

#include "externals/blackboard.h"


/** Keeps a queue of subscribed blackboard events that can be queried in a thread-safe manner */
class BlackboardListenerThread
    : public fawkes::Thread
    , public fawkes::LoggingAspect
    , public fawkes::ConfigurableAspect
    , public fawkes::BlackBoardAspect
    , public fawkes::BlackBoardInterfaceObserver
    , public fawkes::BlackBoardInterfaceListener
{
private:
  using string = std::string;
  template<class T> using queue = std::queue<T>;
  template<class T> using shared_ptr = std::shared_ptr<T>;
  template<class T1, class T2> using map = std::map<T1, T2>;
  using Interface = fawkes::Interface;
  using Mutex = fawkes::Mutex;

public:
  BlackboardListenerThread();

  void observe_pattern(const char *type_pattern, const char *id_pattern) noexcept;
  void listen_for_change(Interface *interface) noexcept;

  virtual void bb_interface_created(const char *type, const char *id) noexcept override;
  virtual void bb_interface_destroyed(const char *type, const char *id) noexcept override;
  virtual void bb_interface_data_changed(Interface *interface) noexcept override;

  static BlackboardListenerThread *instance();
  static void cleanup_instance();


  /** Abstract superclass for blackboard events */
  class Event {
  public:
    /** Constructor
     * @param type Blackboard interface type as string
     * @param id Blackboard interface ID
     */
    Event(const std::string &type, const std::string &id)
      : type(type), id(id)
    {}

    virtual ~Event();

    /** Return an eclipse term representing the event (abstract)
     * @return An eclipse term representing the event (abstract)
     */
    virtual operator EC_word () = 0;

    /** Return the UID (i.e. type::id) of the blackboard interface that triggered the event
     * @return The UID (i.e. type::id) of the blackboard interface that triggered the event
     */
    std::string uid()
    { return type + "::" + id; }

  protected:
    /** Triggering interface's type name */
    string type;

    /** Triggering interface's ID */
    string id;
  };


  /** A new interface was created */
  class Created : public Event {
  public:
    using Event::Event;
    virtual operator EC_word ();
  };


  /** An interface was destroyed */
  class Destroyed : public Event {
  public:
    using Event::Event;
    virtual operator EC_word ();
  };


  /** An interface changed */
  class Changed : public Event {
  public:
    /** Constructor
     * @param interface The interface that changed
     */
    Changed(Interface *interface)
      : Event(interface->type(), interface->id()), interface(interface)
    {}

    virtual operator EC_word ();

  private:
    fawkes::Interface *interface;
  };


  bool event_pending();
  shared_ptr<Event> event_pop();

private:
  Mutex state_mutex_;

  static BlackboardListenerThread *instance_;

  map<string, fawkes::Interface *> last_iface_of_type_;
  queue<shared_ptr<Event>> iface_events_;
};

#endif // BLACKBOARD_LISTENER_THREAD_H
