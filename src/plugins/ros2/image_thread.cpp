
/***************************************************************************
 *  image_thread.cpp - Thread to exchange point clouds
 *
 *  Created: Tue Apr 10 22:12:38 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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
#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace fawkes;
using namespace firevision;

/** @class ROS2ImagesThread "image_thread.h"
 * Thread to export Fawkes images to ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2ImagesThread::ROS2ImagesThread()
: Thread("ROS2ImagesThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

/** Destructor. */
ROS2ImagesThread::~ROS2ImagesThread()
{
}

void
ROS2ImagesThread::init()
{
	it_          = new image_transport::ImageTransport(node_handle);
	last_update_ = new Time(clock);
	now_         = new Time(clock);
	update_images();
}

void
ROS2ImagesThread::finalize()
{
	delete it_;
	delete last_update_;
	delete now_;

	std::map<std::string, PublisherInfo>::iterator p;
	for (p = pubs_.begin(); p != pubs_.end(); ++p) {
		logger->log_info(name(), "Closing image %s", p->first.c_str());
		delete p->second.img;
	}
	pubs_.clear();
}

void
ROS2ImagesThread::loop()
{
	now_->stamp();
	if (*now_ - last_update_ >= 5.0) {
		*last_update_ = now_;
		update_images();
	}

	std::map<std::string, PublisherInfo>::iterator p;
	for (p = pubs_.begin(); p != pubs_.end(); ++p) {
		PublisherInfo &pubinfo = p->second;

		fawkes::Time cap_time = pubinfo.img->capture_time();
		if ((pubinfo.last_sent != cap_time) && (pubinfo.pub.getNumSubscribers() > 0)) {
			pubinfo.last_sent = cap_time;

			//logger->log_debug(name(), "Need to send %s", p->first.c_str());
			//pubinfo.msg.header.seq += 1;
			pubinfo.msg.header.stamp = rclcpp::Time(cap_time.get_sec(), cap_time.get_usec() * 1000);
			convert(pubinfo.img->colorspace(),
			        RGB,
			        pubinfo.img->buffer(),
			        &pubinfo.msg.data[0],
			        pubinfo.msg.width,
			        pubinfo.msg.height);

			pubinfo.pub.publish(pubinfo.msg);
		}
	}
}

void
ROS2ImagesThread::update_images()
{
	std::set<std::string> missing_images;
	std::set<std::string> unbacked_images;
	get_sets(missing_images, unbacked_images);

	if (!unbacked_images.empty()) {
		std::set<std::string>::iterator i;
		for (i = unbacked_images.begin(); i != unbacked_images.end(); ++i) {
			logger->log_info(name(),
			                 "Shutting down publisher for no longer available image %s",
			                 i->c_str());
			PublisherInfo &pubinfo = pubs_[*i];
			delete pubinfo.img;
			pubs_.erase(*i);
		}
	}

	if (!missing_images.empty()) {
		std::set<std::string>::iterator i;
		for (i = missing_images.begin(); i != missing_images.end(); ++i) {
			logger->log_info(name(), "Creating publisher for new image %s", i->c_str());

			std::string            topic_name = std::string("fawkes_imgs/") + *i;
			std::string::size_type pos        = 0;
			while ((pos = topic_name.find("-", pos)) != std::string::npos) {
				topic_name.replace(pos, 1, "_");
			}
			for (pos = 0; (pos = topic_name.find(".", pos)) != std::string::npos;) {
				topic_name.replace(pos, 1, "_");
			}

			PublisherInfo pubinfo;
			pubinfo.pub = it_->advertise(topic_name, 1);
			pubinfo.img = new SharedMemoryImageBuffer(i->c_str());

			pubinfo.msg.header.frame_id = pubinfo.img->frame_id();
			pubinfo.msg.height          = pubinfo.img->height();
			pubinfo.msg.width           = pubinfo.img->width();
			pubinfo.msg.encoding        = sensor_msgs::image_encodings::RGB8;
			pubinfo.msg.step            = pubinfo.msg.width * 3; // for RGB
			pubinfo.msg.data.resize(colorspace_buffer_size(RGB, pubinfo.msg.width, pubinfo.msg.height));

			pubs_[*i] = pubinfo;
		}
	}
}

void
ROS2ImagesThread::get_sets(std::set<std::string> &missing_images,
                          std::set<std::string> &unbacked_images)
{
	std::set<std::string>                          published_images;
	std::map<std::string, PublisherInfo>::iterator p;
	for (p = pubs_.begin(); p != pubs_.end(); ++p) {
		if (p->second.img->num_attached() > 1) {
			published_images.insert(p->first);
		}
	}

	std::set<std::string>              image_buffers;
	SharedMemoryImageBufferHeader *    h    = new SharedMemoryImageBufferHeader();
	SharedMemory::SharedMemoryIterator i    = SharedMemory::find(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h);
	SharedMemory::SharedMemoryIterator endi = SharedMemory::end();

	while (i != endi) {
		const SharedMemoryImageBufferHeader *ih =
		  dynamic_cast<const SharedMemoryImageBufferHeader *>(*i);
		if (ih) {
			image_buffers.insert(ih->image_id());
		}
		++i;
	}
	delete h;

	missing_images.clear();
	unbacked_images.clear();

	std::set_difference(image_buffers.begin(),
	                    image_buffers.end(),
	                    published_images.begin(),
	                    published_images.end(),
	                    std::inserter(missing_images, missing_images.end()));

	std::set_difference(published_images.begin(),
	                    published_images.end(),
	                    image_buffers.begin(),
	                    image_buffers.end(),
	                    std::inserter(unbacked_images, unbacked_images.end()));
}
