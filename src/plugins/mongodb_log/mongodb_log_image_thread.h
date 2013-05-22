
/***************************************************************************
 *  image_thread.h - Thread to log images to MongoDB
 *
 *  Created: Tue Apr 10 22:12:27 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_MONGODB_LOG_MONGODB_LOG_IMAGE_THREAD_H_
#define __PLUGINS_MONGODB_LOG_MONGODB_LOG_IMAGE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/pointcloud.h>
#include <plugins/mongodb/aspect/mongodb.h>
#include <core/threading/mutex.h>

#include <list>
#include <queue>
#include <set>
#include <string>

namespace firevision {
  class SharedMemoryImageBuffer;
}

namespace fawkes {
  class Mutex;
  class TimeWait;
}

namespace mongo {
  class GridFS;
}

class MongoLogImagesThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::MongoDBAspect
{
 public:
  MongoLogImagesThread();
  virtual ~MongoLogImagesThread();

  virtual void init();
  virtual void loop();
  virtual bool prepare_finalize_user();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void update_images();
  void get_sets(std::set<std::string> &missing_images,
                std::set<std::string> &unbacked_images);

 private:
  /// @cond INTERNALS
  typedef struct {
    std::string				 topic_name;
    fawkes::Time                         last_sent;
    firevision::SharedMemoryImageBuffer *img;
  } ImageInfo;
  /// @endcond
  std::map<std::string, ImageInfo> imgs_;

  fawkes::Time *last_update_;
  fawkes::Time *now_;
  mongo::DBClientBase *mongodb_;
  mongo::GridFS       *gridfs_;
  std::string          collection_;
  std::string          database_;

  fawkes::Mutex       *mutex_;
  fawkes::TimeWait    *wait_;

  std::vector<std::string> includes_;
  std::vector<std::string> excludes_;

  unsigned int         cfg_chunk_size_;
  float                cfg_storage_interval_;
};

#endif
