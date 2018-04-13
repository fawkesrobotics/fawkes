
/***************************************************************************
 *  mongodb_log_image_thread.cpp - Thread to log images to MongoDB
 *
 *  Created: Tue Apr 10 22:12:38 2012
 *  Copyright  2011-2017  Tim Niemueller [www.niemueller.de]
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

#include "mongodb_log_image_thread.h"

#include <core/threading/mutex_locker.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/colorspaces.h>
#include <utils/time/wait.h>

// from MongoDB
#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>

#include <fnmatch.h>

using namespace fawkes;
using namespace firevision;
using namespace mongo;

/** @class MongoLogImagesThread "mongodb_log_image_thread.h"
 * Thread to export Fawkes images to MongoDB.
 * @author Tim Niemueller
 * @author Bastian Klingen
 */

/** Constructor. */
MongoLogImagesThread::MongoLogImagesThread()
  : Thread("MongoLogImagesThread", Thread::OPMODE_CONTINUOUS),
    MongoDBAspect("default")
{
  set_prepfin_conc_loop(true);
}

/** Destructor. */
MongoLogImagesThread::~MongoLogImagesThread()
{
}



void
MongoLogImagesThread::init()
{
  database_ = "fflog";
  try {
    database_ = config->get_string("/plugins/mongodb-log/database");
  } catch (Exception &e) {
    logger->log_info(name(), "No database configured, writing to %s",
		     database_.c_str());
  }

  cfg_storage_interval_ =
    config->get_float("/plugins/mongodb-log/images/storage-interval");

  cfg_chunk_size_ = 2097152; // 2 MB
  try {
    cfg_chunk_size_ = config->get_uint("/plugins/mongodb-log/images/chunk-size");
  } catch (Exception &e) {} // ignored, use default
  logger->log_info(name(), "Chunk size: %u", cfg_chunk_size_);

  try {
    includes_ = config->get_strings("/plugins/mongodb-log/images/includes");
  } catch (Exception &e) {} // ignored, no include rules
  try {
    excludes_ = config->get_strings("/plugins/mongodb-log/images/excludes");
  } catch (Exception &e) {} // ignored, no include rules

  mongodb_    = mongodb_client;
  gridfs_  = new GridFS(*mongodb_, database_);

  last_update_ = new Time(clock);
  now_ = new Time(clock);
  wait_  = new TimeWait(clock, cfg_storage_interval_ * 1000000.);
  mutex_ = new Mutex();
  update_images();
}

bool
MongoLogImagesThread::prepare_finalize_user()
{
  mutex_->lock();
  return true;
}

void
MongoLogImagesThread::finalize()
{
  logger->log_debug(name(), "Finalizing MongoLogImagesThread");
  std::map<std::string, ImageInfo>::iterator p;
  for (p = imgs_.begin(); p != imgs_.end(); ++p) {
    delete p->second.img;
  }
  imgs_.clear();
  delete gridfs_;
  delete wait_;
  delete mutex_;
  delete now_;
  delete last_update_;
}


void
MongoLogImagesThread::loop()
{
  MutexLocker lock(mutex_);
  fawkes::Time loop_start(clock);
  wait_->mark_start();
  unsigned int num_stored = 0;

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
      document.append("timestamp", (long long) cap_time.in_msec());

      BSONObjBuilder subb(document.subobjStart("image"));
      subb.append("image_id", imginfo.img->image_id());
      subb.append("width", imginfo.img->width());
      subb.append("height", imginfo.img->height());
      subb.append("colorspace", colorspace_to_string(imginfo.img->colorspace()));

      std::stringstream name;
      name << imginfo.topic_name << "_" << cap_time.in_msec();
      subb.append("data", gridfs_->storeFile((char*) imginfo.img->buffer(), imginfo.img->data_size(), name.str()));

      subb.doneFast();
      collection_ = database_ + "."  + imginfo.topic_name;
      try {
	mongodb_->insert(collection_, document.obj());
	++num_stored;
      } catch (mongo::DBException &e) {
	logger->log_warn(this->name(), "Failed to insert image %s into %s: %s",
			 imginfo.img->image_id(), collection_.c_str(), e.what());
      }
    }
  }

  mutex_->unlock();
  fawkes::Time loop_end(clock);
  logger->log_debug(name(), "Stored %u of %zu images in %.1f ms",
		    num_stored, imgs_.size(), (loop_end - &loop_start) * 1000.);
  wait_->wait();
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

      std::vector<std::string>::iterator f;
      bool include = includes_.empty();
      if (! include) {
	for (f = includes_.begin(); f != includes_.end(); ++f) {
	  if (fnmatch(f->c_str(), i->c_str(), 0) != FNM_NOMATCH) {
	    include = true;
	    break;
	  }
	}
      }
      if (include) {
	for (f = excludes_.begin(); f != excludes_.end(); ++f) {
	  if (fnmatch(f->c_str(), i->c_str(), 0) != FNM_NOMATCH) {
	    include = false;
	    break;
	  }
	}
      }
      if (! include) {
	//logger->log_info(name(), "Excluding image %s", i->c_str());
	continue;
      }

      logger->log_info(name(), "Starting to log image %s", i->c_str());

      std::string topic_name = std::string("Images.") + *i;
      size_t pos = 0;
      while ((pos = topic_name.find_first_of(" -", pos)) != std::string::npos) {
        topic_name.replace(pos, 1, "_");
        pos = pos + 1;
      }

      ImageInfo &imginfo = imgs_[*i];
      imginfo.topic_name = topic_name;
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
