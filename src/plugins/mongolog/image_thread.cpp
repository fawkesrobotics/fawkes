
/***************************************************************************
 *  image_thread.cpp - Thread to log images to MongoDB
 *
 *  adapted from ros/image_thread.cpp
 *
 *  Created: Tue Apr 10 22:12:38 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
 *  Modified: Thu Jul 12 10:00:00 2012 by Bastian Klingen
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

#include "image_thread.h"

#include <core/threading/mutex_locker.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/colorspaces.h>

// from MongoDB
#include <mongo/client/dbclient.h>

using namespace fawkes;
using namespace firevision;
using namespace mongo;

/** @class MongoLogImagesThread "image_thread.h"
 * Thread to export Fawkes images to MongoDB.
 * @author Tim Niemueller
 * @author Bastian Klingen
 */

/** Constructor. */
MongoLogImagesThread::MongoLogImagesThread()
  : Thread("MongoLogImagesThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

/** Destructor. */
MongoLogImagesThread::~MongoLogImagesThread()
{
}



void
MongoLogImagesThread::init()
{
  __database = "fflog";
  try {
    __database = config->get_string("/plugins/mongolog/database");
  } catch (Exception &e) {
    logger->log_info(name(), "No database configured, writing to %s",
		     __database.c_str());
  }
  __collection = __database + ".images";
  __mongodb    = mongodb_client;

  last_update_ = new Time(clock);
  now_ = new Time(clock);
  update_images();
}

void
MongoLogImagesThread::finalize()
{
  std::map<std::string, ImageInfo>::iterator p;
  for (p = imgs_.begin(); p != imgs_.end(); ++p) {
    logger->log_info(name(), "Closing image %s", p->first.c_str());
    delete p->second.img;
  }
  imgs_.clear();
}


void
MongoLogImagesThread::loop()
{
  now_->stamp();
  if (*now_ - last_update_ >= 5.0) {
    *last_update_ = now_;
    update_images();
  }

  std::map<std::string, ImageInfo>::iterator p;
  for (p = imgs_.begin(); p != imgs_.end(); ++p) {
    ImageInfo &imginfo = p->second;

    fawkes::Time cap_time = imginfo.img->capture_time();
    if ((imginfo.last_sent != cap_time)) {
      BSONObjBuilder document;
      imginfo.last_sent = cap_time;
      document.appendTimestamp("timestamp", cap_time.in_msec());

      BSONArrayBuilder subb(document.subobjStart("image"));
      document.append("image_id", imginfo.img->image_id());
      document.append("width", imginfo.img->width());
      document.append("height", imginfo.img->height());
      document.append("colorspace", colorspace_to_string(imginfo.img->colorspace()));
      document.appendBinData("data", (int) imginfo.img->data_size(), BinDataGeneral, imginfo.img->buffer());
      subb.doneFast();
      __mongodb->insert(__collection, document.obj());
    }
  }
 
}


void
MongoLogImagesThread::update_images()
{
  std::set<std::string> missing_images;
  std::set<std::string> unbacked_images;
  get_sets(missing_images, unbacked_images);

  if (! unbacked_images.empty()) {
    std::set<std::string>::iterator i;
    for (i = unbacked_images.begin(); i != unbacked_images.end(); ++i) {
      logger->log_info(name(), "Shutting down MongoLog for no longer available image %s",
                       i->c_str());
      ImageInfo &imginfo = imgs_[*i];
      delete imginfo.img;
      imgs_.erase(*i);
    }
  }

  if (! missing_images.empty()) {
    std::set<std::string>::iterator i;
    for (i = missing_images.begin(); i != missing_images.end(); ++i) {
      logger->log_info(name(), "Creating MongoLog for new image %s",
                       i->c_str());

      ImageInfo &imginfo = imgs_[*i];
      imginfo.img = new SharedMemoryImageBuffer(i->c_str());
    }
  }
}


void
MongoLogImagesThread::get_sets(std::set<std::string> &missing_images,
                          std::set<std::string> &unbacked_images)
{
  std::set<std::string> published_images;
  std::map<std::string, ImageInfo>::iterator p;
  for (p = imgs_.begin(); p != imgs_.end(); ++p) {
    if (p->second.img->num_attached() > 1) {
      published_images.insert(p->first);
    }
  }

  std::set<std::string> image_buffers;
  SharedMemoryImageBufferHeader *h = new SharedMemoryImageBufferHeader();
  SharedMemory::SharedMemoryIterator i =
    SharedMemory::find(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h);
  SharedMemory::SharedMemoryIterator endi = SharedMemory::end();
							    
  while ( i != endi ) {
    const SharedMemoryImageBufferHeader *ih =
      dynamic_cast<const SharedMemoryImageBufferHeader *>(*i);
    if ( ih ) {
      image_buffers.insert(ih->image_id());
    }
    ++i;
  }
  delete h;

  missing_images.clear();
  unbacked_images.clear();

  std::set_difference(image_buffers.begin(), image_buffers.end(),
                      published_images.begin(), published_images.end(),
                      std::inserter(missing_images, missing_images.end()));

  std::set_difference(published_images.begin(), published_images.end(),
                      image_buffers.begin(), image_buffers.end(),
                      std::inserter(unbacked_images, unbacked_images.end()));
}
