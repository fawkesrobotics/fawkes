
/***************************************************************************
 *  pipeline_thread.cpp - FireVision Facer Pipeline Thread
 *
 *  Created: Sat Sat 19 12:41:48 2008 (on the way to German Open 2008, Hannover)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <apps/facer/pipeline_thread.h>

#include <cams/camera.h>
#include <fvutils/ipc/shm_image.h>
#include <utils/time/tracker.h>
#include <fvutils/writers/seq_writer.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/recognition/faces.h>
#include <fvutils/adapters/iplimage.h>
#include <classifiers/faces.h>

#include <opencv/cv.h>
#include <cstring>

/** @class FacerPipelineThread <apps/facer/pipeline_thread.h>
 * FireVision facer pipeline thread.
 * This implements the functionality of the FacerPipelinePlugin.
 * @author Tim Niemueller
 */

/** Constructor. */
FacerPipelineThread::FacerPipelineThread()
  : Thread("FacerPipelineThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC)
{
}


/** Destructor. */
FacerPipelineThread::~FacerPipelineThread()
{
}


void
FacerPipelineThread::init()
{
  try {
    __cfg_haarcascade_file  = config->get_string("/firevision/facer/haarcascade_file");
    __cfg_haar_scale_factor = config->get_float("/firevision/facer/haar_scale_factor");
    __cfg_min_neighbours    = config->get_int("/firevision/facer/min_neighbours");
  } catch (Exception &e) {
    throw;
  }

  try {
    logger->log_debug(name(), "Registering for camera '%s'",
		      config->get_string("/firevision/facer/camera").c_str());
    __cam = vision_master->register_for_camera(config->get_string("/firevision/facer/camera").c_str(), this);
  } catch (Exception &e) {
    e.append("FacerPipelineThread::init() failed");
    throw;
  }
  try {
    __shm = new SharedMemoryImageBuffer("facer-processed", __cam->colorspace(),
				      __cam->pixel_width(), __cam->pixel_height());
    if ( ! __shm->is_valid() ) {
      throw Exception("Shared memory segment not valid");
    }
  } catch (Exception &e) {
    delete __cam;
    __cam = NULL;
    throw;
  }

  try {
    __facer_if = blackboard->open_for_writing<FacerInterface>("Facer");
  } catch (Exception &e) {
    delete __cam;
    delete __shm;
    throw;
  }

  __tt = NULL;
  try {
    if ( config->get_bool("/firevision/facer/use_time_tracker") ) {
      __tt = new TimeTracker();
      __ttc_capture = __tt->add_class("Capture");
      __ttc_memcpy  = __tt->add_class("Memcpy");
      __ttc_dispose = __tt->add_class("Dispose");
      __loop_count  = 0;
    }
  } catch (Exception &e) {
    // ignored, not critical, can be from config->get_*()
  }

  __image = cvCreateImage(cvSize(__cam->pixel_width(), __cam->pixel_height()), IPL_DEPTH_8U, 3);

  __classifier = new FacesClassifier(__cfg_haarcascade_file.c_str(),
				     __cam->pixel_width(), __cam->pixel_height(),
				     __image,
				     __cfg_haar_scale_factor,
				     __cfg_min_neighbours);
  __facerecog  = new FaceRecognizer();

  __opmode = FacerInterface::OPMODE_DISABLED;
  __face_label = "";
}


void
FacerPipelineThread::finalize()
{
  logger->log_debug(name(), "Unregistering from vision master");
  vision_master->unregister_thread(this);
  blackboard->close(__facer_if);
  delete __classifier;
  delete __facerecog;
  delete __cam;
  delete __shm;
  delete __tt;
}


/** Thread loop. */
void
FacerPipelineThread::loop()
{
  __rois = NULL;
  bool debug = false; 

  __cam->capture();
  memcpy(__shm->buffer(), __cam->buffer(), __cam->buffer_size());
  IplImageAdapter::convert_image_bgr(__cam->buffer(), __image);

  while ( ! __facer_if->msgq_empty() ) {
    if ( __facer_if->msgq_first_is<FacerInterface::SetOpmodeMessage>() ) {
      FacerInterface::SetOpmodeMessage *msg = __facer_if->msgq_first<FacerInterface::SetOpmodeMessage>();
      __opmode = msg->opmode();
    } else if ( __facer_if->msgq_first_is<FacerInterface::LearnFaceMessage>() ) {
      FacerInterface::LearnFaceMessage *msg = __facer_if->msgq_first<FacerInterface::LearnFaceMessage>();
      __opmode = FacerInterface::OPMODE_LEARNING;
      __face_label = msg->face_label();
    }

    __facer_if->msgq_pop();
  }

  // unecessary since we pass our own IplImage and alread did the conversion
  //__classifier->set_src_buffer(__cam->buffer(), __cam->pixel_width(), __cam->pixel_height());

  switch (__opmode) {
  case FacerInterface::OPMODE_DETECTION:
    // detect faces, just put up the recognized faces
    __rois = __classifier->classify();
    break;

  case FacerInterface::OPMODE_RECOGNITION:
    // detect faces, then try to recognize the found faces
    __rois = __classifier->classify();
    if ( ! __rois->empty() ) {
      // pass only most dominant ROI (biggest one)

      ROI &roi = *(__rois->begin());
      CvRect roi_rect = cvRect(roi.start.x, roi.start.y, roi.width, roi.height);
      cvSetImageROI(__image, roi_rect);
      IplImage *face = cvCreateImage( cvSize(roi.width, roi.height),
				      __image->depth, __image->nChannels);
      
      
      cvCopyImage(__image, face);
      cvResetImageROI(__image);

      if( debug ) { 
	char buffer[PATH_MAX]; 
	sprintf( buffer,"%f.png", rand() ); 
	cvvSaveImage( buffer, face ); 
      }

      std::vector<IplImage *> face_images;
      face_images.push_back(face);
      std::vector<std::string> face_labels = __facerecog->recognize(face_images);
      __face_label = face_labels[0];

      __facer_if->set_face_label(__face_label.c_str());
    } else {
      logger->log_info("FacerPipelineThread", "No ROIs found");
    }
    break;

  case FacerInterface::OPMODE_LEARNING:
    // train new faces, recognition data is learned from current images
    break;

  default:
    logger->log_debug("FacerPipelineThread", "disabled");
    break;
  }

  __facer_if->set_opmode(__opmode);
  __facer_if->write();


  delete __rois;
  __cam->dispose_buffer();
}
