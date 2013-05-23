
/***************************************************************************
 *  mongodb_log_bb_thread.h - MongoDB blackboard logging thread
 *
 *  Created: Wed Dec 08 23:08:14 2010
 *  Copyright  2010-2012  Tim Niemueller [www.niemueller.de]
 *             2012       Bastian Klingen
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

#ifndef __PLUGINS_MONGODB_LOG_MONGODB_LOG_BB_THREAD_H_
#define __PLUGINS_MONGODB_LOG_MONGODB_LOG_BB_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <plugins/mongodb/aspect/mongodb.h>

#include <blackboard/interface_observer.h>
#include <blackboard/interface_listener.h>
#include <core/utils/lock_map.h>
#include <core/utils/lock_set.h>

#include <string>


class MongoLogBlackboardThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::MongoDBAspect,
  public fawkes::BlackBoardInterfaceObserver
{
 public:
  MongoLogBlackboardThread();
  virtual ~MongoLogBlackboardThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for BlackBoardInterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  /** Mongo Logger interface listener. */
  class InterfaceListener : public fawkes::BlackBoardInterfaceListener
  {
   public:
    InterfaceListener(fawkes::BlackBoard *blackboard,
		      fawkes::Interface *interface,
		      mongo::DBClientBase *mongodb,
		      std::string &database,
		      fawkes::LockSet<std::string> &colls,
		      fawkes::Logger *logger,
		      fawkes::Time *now);
    ~InterfaceListener();

    /** Get MongoDB Client.
     * @return MongoDB client */
    mongo::DBClientBase * mongodb_client() const
    { return mongodb_; }

    // for BlackBoardInterfaceListener
    virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

   private:
    fawkes::BlackBoard  *blackboard_;
    fawkes::Interface   *interface_;
    mongo::DBClientBase *mongodb_;
    fawkes::Logger      *logger_;
    std::string          collection_;
    std::string         &database_;
    fawkes::LockSet<std::string> &collections_;
    fawkes::Time        *now_;
  };


  fawkes::LockMap<std::string, InterfaceListener *> listeners_;
  fawkes::LockSet<std::string> collections_;
  std::string database_;
  fawkes::Time        *now_;

  std::vector<std::string> excludes_;
};

#endif
