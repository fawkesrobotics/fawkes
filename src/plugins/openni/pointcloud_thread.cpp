
/***************************************************************************
 *  pointcloud_thread.cpp - OpenNI point cloud provider thread
 *
 *  Created: Fri Mar 25 23:49:11 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
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

#include "pointcloud_thread.h"

#include "image_thread.h"
#include "utils/setup.h"

#include <core/threading/mutex_locker.h>
#include <fvutils/base/types.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/rgb.h>
#include <fvutils/ipc/shm_image.h>
#ifdef HAVE_PCL
#	include <pcl_utils/utils.h>
#endif

#include <memory>

using namespace fawkes;
using namespace firevision;

/** @class OpenNiPointCloudThread "pointcloud_thread.h"
 * OpenNI Point Cloud Provider Thread.
 * This thread provides a point cloud calculated from the depth image
 * acquired via OpenNI and provides it as a
 * SharedMemoryImageBuffer to other FireVision plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param img_thread OpenNI image thread, used for XYZRGB point clouds
 */
OpenNiPointCloudThread::OpenNiPointCloudThread(OpenNiImageThread *img_thread)
: Thread("OpenNiPointCloudThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE)
{
	img_thread_ = img_thread;
}

/** Destructor. */
OpenNiPointCloudThread::~OpenNiPointCloudThread()
{
}

void
OpenNiPointCloudThread::init()
{
	MutexLocker lock(openni.objmutex_ptr());

	image_rgb_buf_ = NULL;

	depth_gen_ = new xn::DepthGenerator();
#if __cplusplus >= 201103L
	std::unique_ptr<xn::DepthGenerator> depthgen_uniqueptr(depth_gen_);
	std::unique_ptr<xn::ImageGenerator> imagegen_uniqueptr(image_gen_);
#else
	std::auto_ptr<xn::DepthGenerator> depthgen_uniqueptr(depth_gen_);
	std::auto_ptr<xn::ImageGenerator> imagegen_uniqueptr(image_gen_);
#endif

	image_gen_ = new xn::ImageGenerator();

	XnStatus st;

	fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, depth_gen_);
	fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_IMAGE, image_gen_);
	fawkes::openni::setup_map_generator(*image_gen_, config);
	fawkes::openni::setup_map_generator(*depth_gen_, config);

	depth_md_ = new xn::DepthMetaData();
	depth_gen_->GetMetaData(*depth_md_);

	cfg_register_depth_image_ = false;
	try {
		cfg_register_depth_image_ = config->get_bool("/plugins/openni/register_depth_image");
	} catch (Exception &e) {
	}

	cfg_frame_depth_ = config->get_string("/plugins/openni/frame/depth");
	cfg_frame_image_ = config->get_string("/plugins/openni/frame/image");

	pcl_xyz_buf_ = new SharedMemoryImageBuffer("openni-pointcloud-xyz",
	                                           CARTESIAN_3D_FLOAT,
	                                           depth_md_->XRes(),
	                                           depth_md_->YRes());

	pcl_xyz_buf_->set_frame_id(cfg_register_depth_image_ ? cfg_frame_image_.c_str()
	                                                     : cfg_frame_depth_.c_str());

	pcl_xyzrgb_buf_ = new SharedMemoryImageBuffer("openni-pointcloud-xyzrgb",
	                                              CARTESIAN_3D_FLOAT_RGB,
	                                              depth_md_->XRes(),
	                                              depth_md_->YRes());

	pcl_xyzrgb_buf_->set_frame_id(cfg_register_depth_image_ ? cfg_frame_image_.c_str()
	                                                        : cfg_frame_depth_.c_str());

	// this is magic from ROS openni_device.cpp, reading code from
	// openni-primesense suggests that SXGA is the base configuration
	XnUInt64 zpd; // zero plane distance
	if ((st = depth_gen_->GetIntProperty("ZPD", zpd)) != XN_STATUS_OK) {
		throw Exception("Failed to get ZPD: %s", xnGetStatusString(st));
	}
	XnDouble pixel_size; // zero plane pixel size
	if ((st = depth_gen_->GetRealProperty("ZPPS", pixel_size)) != XN_STATUS_OK) {
		throw Exception("Failed to get ZPPS: %s", xnGetStatusString(st));
	}

	if ((st = depth_gen_->GetIntProperty("NoSampleValue", no_sample_value_)) != XN_STATUS_OK) {
		throw Exception("Failed to get NoSampleValue: %s", xnGetStatusString(st));
	}
	if ((st = depth_gen_->GetIntProperty("ShadowValue", shadow_value_)) != XN_STATUS_OK) {
		throw Exception("Failed to get ShadowValue: %s", xnGetStatusString(st));
	}

	width_      = depth_md_->XRes();
	height_     = depth_md_->YRes();
	float scale = width_ / (float)XN_SXGA_X_RES;
	if (cfg_register_depth_image_) {
		// magic number taken from ROS/PCL openni_device.cpp
		const float rgb_focal_length_SXGA = 1050;
		focal_length_                     = rgb_focal_length_SXGA * scale;
	} else {
		focal_length_ = ((float)zpd / pixel_size) * scale;
	}
	foc_const_ = 0.001 / focal_length_;
	center_x_  = (width_ / 2.) - .5f;
	center_y_  = (height_ / 2.) - .5f;

	image_gen_->StartGenerating();
	depth_gen_->StartGenerating();

	capture_start_ = new Time(clock);
	capture_start_->stamp_systime();
	// Update once to get timestamp
	depth_gen_->WaitAndUpdateData();
	// arbitrarily define the zero reference point,
	// we can't get any closer than this
	*capture_start_ -= (long int)depth_gen_->GetTimestamp();

	image_gen_->WaitAndUpdateData();

	if (cfg_register_depth_image_) {
		// RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
		// (from ROS openni_camera)
		unsigned short usb_vendor = 0, usb_product = 0;
		fawkes::openni::get_usb_info(*depth_gen_, usb_vendor, usb_product);

		if ((usb_vendor == 0x045e) && (usb_product == 0x02ae)) {
			if (depth_gen_->SetIntProperty("RegistrationType", 2) != XN_STATUS_OK) {
				throw Exception("Failed to set registration type");
			}
		} else {
			if (depth_gen_->SetIntProperty("RegistrationType", 1) != XN_STATUS_OK) {
				throw Exception("Failed to set registration type");
			}
		}

		logger->log_info(name(), "Setting depth alternate viewpoint to image");
		fawkes::openni::setup_alternate_viewpoint(*depth_gen_, *image_gen_);
	}

	// Fails with "Bad Paramter" on OpenNI 1.3.2.1/PS 5.0.3.3
	//logger->log_info(name(), "Setting depth/image synchronization");
	//fawkes::openni::setup_synchronization(*depth_gen_, *image_gen_);

#ifdef HAVE_PCL
	cfg_generate_pcl_ = true;
	try {
		cfg_generate_pcl_ = config->get_bool("/plugins/openni-pointcloud/generate-pcl");
	} catch (Exception &e) {
	}

	if (cfg_generate_pcl_) {
		pcl_xyz_           = new pcl::PointCloud<pcl::PointXYZ>();
		pcl_xyz_->is_dense = false;
		pcl_xyz_->width    = width_;
		pcl_xyz_->height   = height_;
		pcl_xyz_->points.resize((size_t)width_ * (size_t)height_);
		pcl_xyz_->header.frame_id = cfg_register_depth_image_ ? cfg_frame_image_ : cfg_frame_depth_;

		pcl_xyzrgb_           = new pcl::PointCloud<pcl::PointXYZRGB>();
		pcl_xyzrgb_->is_dense = false;
		pcl_xyzrgb_->width    = width_;
		pcl_xyzrgb_->height   = height_;
		pcl_xyzrgb_->points.resize((size_t)width_ * (size_t)height_);
		pcl_xyzrgb_->header.frame_id = cfg_register_depth_image_ ? cfg_frame_image_ : cfg_frame_depth_;

		pcl_manager->add_pointcloud("openni-pointcloud-xyz", pcl_xyz_);
		pcl_manager->add_pointcloud("openni-pointcloud-xyzrgb", pcl_xyzrgb_);
	}
#endif

	depthgen_uniqueptr.release();
	imagegen_uniqueptr.release();
}

void
OpenNiPointCloudThread::finalize()
{
#ifdef HAVE_PCL
	pcl_manager->remove_pointcloud("openni-pointcloud-xyz");
	pcl_manager->remove_pointcloud("openni-pointcloud-xyzrgb");
#endif

	// we do not stop generating, we don't know if there is no other plugin
	// using the node.
	delete depth_gen_;
	delete depth_md_;
	delete pcl_xyz_buf_;
	delete pcl_xyzrgb_buf_;
	delete capture_start_;
}

void
OpenNiPointCloudThread::fill_xyz_no_pcl(fawkes::Time &ts, const XnDepthPixel *const depth_data)
{
	pcl_xyz_buf_->lock_for_write();
	pcl_xyz_buf_->set_capture_time(&ts);

	pcl_point_t *pclbuf = (pcl_point_t *)pcl_xyz_buf_->buffer();

	unsigned int idx = 0;
	for (unsigned int h = 0; h < height_; ++h) {
		for (unsigned int w = 0; w < width_; ++w, ++idx, ++pclbuf) {
			// Check for invalid measurements
			if (depth_data[idx] == 0 || depth_data[idx] == no_sample_value_
			    || depth_data[idx] == shadow_value_) {
				// invalid
				pclbuf->x = pclbuf->y = pclbuf->z = 0.f;
			} else {
				// Fill in XYZ
				pclbuf->x = depth_data[idx] * 0.001f;
				pclbuf->y = -(w - center_x_) * depth_data[idx] * foc_const_;
				pclbuf->z = -(h - center_y_) * depth_data[idx] * foc_const_;
			}
		}
	}

	pcl_xyz_buf_->unlock();
}

void
OpenNiPointCloudThread::fill_xyzrgb_no_pcl(fawkes::Time &ts, const XnDepthPixel *const depth_data)
{
	pcl_xyzrgb_buf_->lock_for_write();
	pcl_xyzrgb_buf_->set_capture_time(&ts);

	pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)pcl_xyzrgb_buf_->buffer();

	unsigned int idx = 0;
	for (unsigned int h = 0; h < height_; ++h) {
		for (unsigned int w = 0; w < width_; ++w, ++idx, ++pclbuf_rgb) {
			// Check for invalid measurements
			if (depth_data[idx] == 0 || depth_data[idx] == no_sample_value_
			    || depth_data[idx] == shadow_value_) {
				// invalid
				pclbuf_rgb->x = pclbuf_rgb->y = pclbuf_rgb->z = 0.f;
			} else {
				// Fill in XYZ
				pclbuf_rgb->x = depth_data[idx] * 0.001f;
				pclbuf_rgb->y = -(w - center_x_) * depth_data[idx] * foc_const_;
				pclbuf_rgb->z = -(h - center_y_) * depth_data[idx] * foc_const_;
			}
		}
	}

	fill_rgb_no_pcl();

	pcl_xyzrgb_buf_->unlock();
}

void
OpenNiPointCloudThread::fill_xyz_xyzrgb_no_pcl(fawkes::Time &            ts,
                                               const XnDepthPixel *const depth_data)
{
	pcl_xyz_buf_->lock_for_write();
	pcl_xyz_buf_->set_capture_time(&ts);

	pcl_xyzrgb_buf_->lock_for_write();
	pcl_xyzrgb_buf_->set_capture_time(&ts);

	pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)pcl_xyzrgb_buf_->buffer();
	pcl_point_t *       pclbuf_xyz = (pcl_point_t *)pcl_xyz_buf_->buffer();

	unsigned int idx = 0;
	for (unsigned int h = 0; h < height_; ++h) {
		for (unsigned int w = 0; w < width_; ++w, ++idx, ++pclbuf_rgb, ++pclbuf_xyz) {
			// Check for invalid measurements
			if (depth_data[idx] == 0 || depth_data[idx] == no_sample_value_
			    || depth_data[idx] == shadow_value_) {
				// invalid
				pclbuf_rgb->x = pclbuf_rgb->y = pclbuf_rgb->z = 0.f;
				pclbuf_xyz->x = pclbuf_xyz->y = pclbuf_xyz->z = 0.f;
			} else {
				// Fill in XYZ
				pclbuf_rgb->x = pclbuf_xyz->x = depth_data[idx] * 0.001f;
				pclbuf_rgb->y = pclbuf_xyz->y = -(w - center_x_) * depth_data[idx] * foc_const_;
				pclbuf_rgb->z = pclbuf_xyz->z = -(h - center_y_) * depth_data[idx] * foc_const_;
			}
		}
	}

	fill_rgb_no_pcl();

	pcl_xyzrgb_buf_->unlock();
	pcl_xyz_buf_->unlock();
}

void
OpenNiPointCloudThread::fill_rgb_no_pcl()
{
	if (!image_rgb_buf_) {
		try {
			image_rgb_buf_ = new SharedMemoryImageBuffer("openni-image-rgb");
		} catch (Exception &e) {
			logger->log_warn(name(), "Failed to open openni-image-rgb shm image buffer");
			return;
		}
	}

	img_thread_->wait_loop_done();

	pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)pcl_xyzrgb_buf_->buffer();
	RGB_t *             imagebuf   = (RGB_t *)image_rgb_buf_->buffer();

	for (unsigned int i = 0; i < width_ * height_; ++i) {
		pclbuf_rgb->r = imagebuf[i].R;
		pclbuf_rgb->g = imagebuf[i].G;
		pclbuf_rgb->b = imagebuf[i].B;
	}
}

#ifdef HAVE_PCL
void
OpenNiPointCloudThread::fill_xyz(fawkes::Time &ts, const XnDepthPixel *const depth_data)
{
	pcl::PointCloud<pcl::PointXYZ> &pcl = **pcl_xyz_;
	pcl.header.seq += 1;
	pcl_utils::set_time(pcl_xyz_, ts);

	pcl_xyz_buf_->lock_for_write();
	pcl_xyz_buf_->set_capture_time(&ts);

	pcl_point_t *pclbuf = (pcl_point_t *)pcl_xyz_buf_->buffer();

	unsigned int idx = 0;
	for (unsigned int h = 0; h < height_; ++h) {
		for (unsigned int w = 0; w < width_; ++w, ++idx, ++pclbuf) {
			// Check for invalid measurements
			if (depth_data[idx] == 0 || depth_data[idx] == no_sample_value_
			    || depth_data[idx] == shadow_value_) {
				// invalid
				pclbuf->x = pclbuf->y = pclbuf->z = 0.f;
				pcl.points[idx].x = pcl.points[idx].y = pcl.points[idx].z = 0.f;
			} else {
				// Fill in XYZ
				pclbuf->x = pcl.points[idx].x = depth_data[idx] * 0.001f;
				pclbuf->y = pcl.points[idx].y = -(w - center_x_) * depth_data[idx] * foc_const_;
				pclbuf->z = pcl.points[idx].z = -(h - center_y_) * depth_data[idx] * foc_const_;
			}
		}
	}

	pcl_xyz_buf_->unlock();
}

void
OpenNiPointCloudThread::fill_xyzrgb(fawkes::Time &ts, const XnDepthPixel *const depth_data)
{
	pcl::PointCloud<pcl::PointXYZRGB> &pcl_rgb = **pcl_xyzrgb_;
	pcl_rgb.header.seq += 1;
	pcl_utils::set_time(pcl_xyzrgb_, ts);

	pcl_xyzrgb_buf_->lock_for_write();
	pcl_xyzrgb_buf_->set_capture_time(&ts);

	pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)pcl_xyzrgb_buf_->buffer();

	unsigned int idx = 0;
	for (unsigned int h = 0; h < height_; ++h) {
		for (unsigned int w = 0; w < width_; ++w, ++idx, ++pclbuf_rgb) {
			// Check for invalid measurements
			if (depth_data[idx] == 0 || depth_data[idx] == no_sample_value_
			    || depth_data[idx] == shadow_value_) {
				// invalid
				pclbuf_rgb->x = pclbuf_rgb->y = pclbuf_rgb->z = 0.f;
				pcl_rgb.points[idx].x = pcl_rgb.points[idx].y = pcl_rgb.points[idx].z = 0.f;
			} else {
				// Fill in XYZ
				pclbuf_rgb->x = pcl_rgb.points[idx].x = depth_data[idx] * 0.001f;
				pclbuf_rgb->y = pcl_rgb.points[idx].y = -(w - center_x_) * depth_data[idx] * foc_const_;
				pclbuf_rgb->z = pcl_rgb.points[idx].z = -(h - center_y_) * depth_data[idx] * foc_const_;
			}
		}
	}

	fill_rgb(pcl_rgb);

	pcl_xyzrgb_buf_->unlock();
}

void
OpenNiPointCloudThread::fill_xyz_xyzrgb(fawkes::Time &ts, const XnDepthPixel *const depth_data)
{
	pcl::PointCloud<pcl::PointXYZRGB> &pcl_rgb = **pcl_xyzrgb_;
	pcl_rgb.header.seq += 1;
	pcl_utils::set_time(pcl_xyzrgb_, ts);

	pcl::PointCloud<pcl::PointXYZ> &pcl_xyz = **pcl_xyz_;
	pcl_xyz.header.seq += 1;
	pcl_utils::set_time(pcl_xyz_, ts);

	pcl_xyz_buf_->lock_for_write();
	pcl_xyz_buf_->set_capture_time(&ts);

	pcl_xyzrgb_buf_->lock_for_write();
	pcl_xyzrgb_buf_->set_capture_time(&ts);

	pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)pcl_xyzrgb_buf_->buffer();
	pcl_point_t *       pclbuf_xyz = (pcl_point_t *)pcl_xyz_buf_->buffer();

	unsigned int idx = 0;
	for (unsigned int h = 0; h < height_; ++h) {
		for (unsigned int w = 0; w < width_; ++w, ++idx, ++pclbuf_rgb, ++pclbuf_xyz) {
			// Check for invalid measurements
			if (depth_data[idx] == 0 || depth_data[idx] == no_sample_value_
			    || depth_data[idx] == shadow_value_) {
				// invalid
				pclbuf_rgb->x = pclbuf_rgb->y = pclbuf_rgb->z = 0.f;
				pcl_rgb.points[idx].x = pcl_rgb.points[idx].y = pcl_rgb.points[idx].z = 0.f;

				pclbuf_xyz->x = pclbuf_xyz->y = pclbuf_xyz->z = 0.f;
				pcl_xyz.points[idx].x = pcl_xyz.points[idx].y = pcl_xyz.points[idx].z = 0.f;
			} else {
				// Fill in XYZ
				pclbuf_rgb->x = pcl_rgb.points[idx].x = pclbuf_xyz->x = pcl_xyz.points[idx].x =
				  depth_data[idx] * 0.001f;
				pclbuf_rgb->y = pcl_rgb.points[idx].y = pclbuf_xyz->y = pcl_xyz.points[idx].y =
				  -(w - center_x_) * depth_data[idx] * foc_const_;
				pclbuf_rgb->z = pcl_rgb.points[idx].z = pclbuf_xyz->z = pcl_xyz.points[idx].z =
				  -(h - center_y_) * depth_data[idx] * foc_const_;
			}
		}
	}

	fill_rgb(pcl_rgb);

	pcl_xyzrgb_buf_->unlock();
	pcl_xyz_buf_->unlock();
}

void
OpenNiPointCloudThread::fill_rgb(pcl::PointCloud<pcl::PointXYZRGB> &pcl_rgb)
{
	if (!image_rgb_buf_) {
		try {
			image_rgb_buf_ = new SharedMemoryImageBuffer("openni-image-rgb");
		} catch (Exception &e) {
			logger->log_warn(name(), "Failed to open openni-image-rgb shm image buffer");
			return;
		}
	}

	img_thread_->wait_loop_done();

	pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)pcl_xyzrgb_buf_->buffer();
	RGB_t *             imagebuf   = (RGB_t *)image_rgb_buf_->buffer();

	for (unsigned int i = 0; i < width_ * height_; ++i) {
		pclbuf_rgb->r = pcl_rgb.points[i].r = imagebuf[i].R;
		pclbuf_rgb->g = pcl_rgb.points[i].g = imagebuf[i].G;
		pclbuf_rgb->b = pcl_rgb.points[i].b = imagebuf[i].B;
	}
}

#endif

void
OpenNiPointCloudThread::loop()
{
	MutexLocker lock(openni.objmutex_ptr());
	bool        is_data_new = depth_gen_->IsDataNew();
	depth_gen_->GetMetaData(*depth_md_);
	const XnDepthPixel *const data = depth_md_->Data();
	// experimental: unlock here as we do not invoke any methods anymore
	// since data has been updated earlier in the sensor hook we should be safe
	lock.unlock();

	bool xyz_requested = (pcl_xyz_buf_->num_attached() > 1)
#ifdef HAVE_PCL
	                     // 2 is us and the PCL manager of the PointCloudAspect
	                     || (cfg_generate_pcl_ && ((pcl_xyz_.use_count() > 2)))
#endif
	  ;
	bool xyzrgb_requested = (pcl_xyzrgb_buf_->num_attached() > 1)
#ifdef HAVE_PCL
	                        // 2 is us and the PCL manager of the PointCloudAspect
	                        || (cfg_generate_pcl_ && ((pcl_xyzrgb_.use_count() > 2)))
#endif
	  ;

	if (is_data_new && (xyz_requested || xyzrgb_requested)) {
		// convert depth to points
		fawkes::Time ts = *capture_start_ + (long int)depth_gen_->GetTimestamp();

#ifdef HAVE_PCL
		if (cfg_generate_pcl_) {
			if (xyz_requested && xyzrgb_requested) {
				fill_xyz_xyzrgb(ts, data);
			} else if (xyz_requested) {
				fill_xyz(ts, data);
			} else if (xyzrgb_requested) {
				fill_xyzrgb(ts, data);
			}

		} else {
#endif
			if (xyz_requested && xyzrgb_requested) {
				fill_xyz_xyzrgb_no_pcl(ts, data);
			} else if (xyz_requested) {
				fill_xyz_no_pcl(ts, data);
			} else if (xyzrgb_requested) {
				fill_xyzrgb_no_pcl(ts, data);
			}
#ifdef HAVE_PCL
		}
#endif

		// close rgb image buffer if no longer required
		if (!xyzrgb_requested && image_rgb_buf_) {
			delete image_rgb_buf_;
			image_rgb_buf_ = NULL;
		}
	}
}
