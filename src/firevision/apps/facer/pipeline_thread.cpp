
/***************************************************************************
 *  pipeline_thread.cpp - FireVision Facer Pipeline Thread
 *
 *  Created: Sat Sat 19 12:41:48 2008 (on the way to German Open 2008, Hannover)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *             2008 Vaishak Belle
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free     Software Foundation; either version 2 of the License, or
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
#include <stdio.h>
#include <cams/camera.h>
#include <fvutils/ipc/shm_image.h>
#include <utils/time/tracker.h>
#include <fvutils/writers/seq_writer.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/recognition/faces.h>
#include <fvutils/adapters/iplimage.h>
#include <classifiers/faces.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstring>
#include <map>
#include <string>

#define __SUBSEQ_FACES 3


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
  //  std::map<int, std::string> persons;
  __roi_not_found_flag = false; 
  __person_recognized_cnt = 0; 
  

  try {
    // detection specific
    __cfg_haarcascade_file  = config->get_string("/firevision/facer/haarcascade_file");
    __cfg_haar_scale_factor = config->get_float("/firevision/facer/haar_scale_factor");
    __cfg_min_neighbours    = config->get_int("/firevision/facer/min_neighbours");
    // recognition specific
    __cfg_dir_path = config->get_string("/firevision/facer/faces_dir_path");
    __cfg_forest_size = config->get_uint("/firevision/facer/forest_size"); 
    __cfg_number_identities = config->get_uint("/firevision/facer/number_identities");
    __saved_faces = 0;

    Configuration::ValueIterator *i = config->search("/firevision/facer/identity_");
    while(i->next()) {
      if ( i->is_string() ) {
	char *tmp = strdup(i->path());
        char *saveptr;
        char *s = strtok_r(tmp, "_", &saveptr);
        s = strtok_r(NULL, "_", &saveptr);
        int id = atoi(s);
        free(tmp);
        __persons[id] = i->get_string();
      } else {
        logger->log_warn("FacerPipelineThread", "Value %s is not of type string, but of type %s", i->path(), i->type());
      }
    }
    delete i;

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

  __facerecog  = new FaceRecognizer( (__cfg_dir_path).c_str(),  __cfg_number_identities, __cfg_forest_size );
  //  logger->log_info("FacerPipelineThread","number of identiites instantiated %d", __facerecog->get_n_identities()); 
  for (std::map<int, std::string>::iterator i = __persons.begin(); i != __persons.end(); ++i ) {
    __facerecog->add_identity(i->first, i->second);
  }
  //  logger->log_info("FacerPipelineThread","number of identiites instantiated %d after instation", __facerecog->get_n_identities()); 
  //  __opmode = FacerInterface::OPMODE_RECOGNITION;
    __opmode = FacerInterface::OPMODE_LEARNING; 
  __face_label = "";
}


void
FacerPipelineThread::finalize()
{
  logger->log_debug(name(), "Unregistering from vision master");
  vision_master->unregister_thread(this);
  blackboard->close(__facer_if);
  cvReleaseImage(&__image);
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
  bool debug = true; 

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
	char *buffer;
	asprintf( &buffer,"%d.png", ++__saved_faces ); 
	cvvSaveImage( buffer, face ); 
	free( buffer );
      }

      std::vector<IplImage *> face_images;
      face_images.push_back(face);
      // the second parameter, number_of_identities, is 0 since we are NOT learning new ppl at the momnent

      std::vector<int> recognition_indices = __facerecog->recognize( face_images, 0 ); 
      int recognition_index = recognition_indices.at(0); 

      std::vector<std::string> face_labels = __facerecog->get_identities(recognition_indices); 
      if( face_labels.size() != 0 ) {
	__face_label = face_labels[0];
        logger->log_info("FacerPipelineThread", "Current Face is recognized, person is %s", __face_label.c_str());
	if( __roi_not_found_flag ) //previously a face was not reocgnized
	  { 
	    if( __person_recognized_cnt == 0 ) // start recording __SUBSEQ_FACES recognitions 
	      {
		__roi_not_found_flag = false; // reset the flag until another no-roi found
		__person_labels[__person_recognized_cnt++] = recognition_index; // first recognition
	      }
	    else 
	      {
		__person_recognized_cnt = 0; // person went away, stop recording - reset flag
	      }
	  }
	else 
	  {
	    if( __person_recognized_cnt < __SUBSEQ_FACES ) // if below __SUBSEQ_FACES 
	      __person_labels[__person_recognized_cnt++] = recognition_index;  // subsequent recognitions 
	    else 
	      {
		int histogram[__facerecog->get_n_identities()];  // all recorded 
		for( int i = 0; i < __facerecog->get_n_identities(); i++ )
		  histogram[i] = 0;
		
		for( int i = 0; i < (int)recognition_indices.size(); i++ ) 
		  {
		    ++histogram[recognition_indices.at(i)]; // record recognitions for each
		  }

		int max_label = -1, max_recognized = -1;
		for( int i = 0; i < __facerecog->get_n_identities(); i++ )
		  {
		    if( histogram[__facerecog->get_n_identities()] > max_recognized ) //ginf max
		      {
			max_label = i;
			max_recognized = histogram[__facerecog->get_n_identities()];
		      }
		  }
		
		// can also put a threshold here
		logger->log_info("FacerPipelineThread","The final recognized identitey %d and %s", max_label, (face_labels.at(max_label)).c_str()); 

		   
	      }
	  }
      	
      
      } else {
	logger->log_info("FacerPipelineThread", "No identity returned. Most probably forest has not been instantiated/training images not supplied."); 
      }
      
      __facer_if->set_face_label(__face_label.c_str());
    } else {
      logger->log_info("FacerPipelineThread", "No ROIs found");
      __roi_not_found_flag = true; 
    }
    break;

  case FacerInterface::OPMODE_LEARNING:
    // train new faces, recognition data is learned from current images
    __rois = __classifier->classify();
    if ( ! __rois->empty() ) {
      // pass only most dominant ROI (biggest one)

      logger->log_debug("FacerPipelineThread", "Smile, you're on robot TV!");

      ROI &roi = *(__rois->begin());
      CvRect roi_rect = cvRect(roi.start.x, roi.start.y, roi.width, roi.height);
      cvSetImageROI(__image, roi_rect);
      IplImage *face = cvCreateImage( cvSize(roi.width, roi.height),
				      __image->depth, __image->nChannels);
      
      cvCopyImage(__image, face);
      cvResetImageROI(__image);

      IplImage *scaled_face = cvCreateImage( cvSize(48, 48),
                                             __image->depth, __image->nChannels);

      cvResize(face, scaled_face, CV_INTER_LINEAR);
      //      cvvSaveImage( buffer, scaled_face ); 
      
      //            if( debug ) {
      char *buffer;
      char* dirname;
      asprintf( &dirname, "%s%d", (__cfg_dir_path).c_str(), __facerecog->get_n_identities() ); 
      mkdir( dirname, 777 ); 
	
      //	logger->log_info("FacerPipelineThread","NUMBER Of identiites = %d", __facerecog->get_n_identities() ); 
      //	asprintf( &buffer,"%s%d/%d.png",(__cfg_dir_path).c_str(), __facerecog->get_n_identities(), ++__saved_faces ); 
      asprintf( &buffer, "%d.png", ++__saved_faces ); 
      cvvSaveImage( buffer, scaled_face ); 
      char* new_name;
      asprintf( &new_name, "%s%d/%d.png", (__cfg_dir_path).c_str(), __facerecog->get_n_identities(), __saved_faces); 
      rename( buffer, new_name ); 
      //rename("%d.png","%s%d/%d.png", __saved_faces, (__cfg_dir_path).c_str(), __facerecog->get_n_identities(), __saved_faces );
      logger->log_debug("FacerPipelineThread", "Face saved to %s", new_name);
	
      free( new_name ); 
      free( dirname ); 
      free( buffer );
      //    }

      cvReleaseImage(&face);
      cvReleaseImage(&scaled_face);
    } else {
      logger->log_info("FacerPipelineThread", "No ROIs found");

    }


    // @TODO: do the procedure below in the calling functoin 

//     //    std::map<int, std::string> persons;
//     try {

//       Configuration::ValueIterator *i = config->search("/firevision/facer/identity_");
//       while(i->next()) {
// 	if ( i->is_string() ) {
// 	  char *tmp = strdup(i->path());
// 	  char *saveptr;
// 	  char *s = strtok_r(tmp, "_", &saveptr);
// 	  s = strtok_r(NULL, "_", &saveptr);
// 	  int id = atoi(s);
// 	  free(tmp);
// 	  __persons[id] = i->get_string();
// 	} else {
// 	  logger->log_warn("FacerPipelineThread", "Value %s is not of type string, but of type %s", i->path(), i->type());
// 	}
//       }
//       delete i;

//     } catch (Exception &e) {
//       throw;
//     }

    delete __facerecog; 

//     __facerecog  = new FaceRecognizer( (__cfg_dir_path).c_str(),  __cfg_number_identities + 1, __cfg_forest_size );
//     //  logger->log_info("FacerPipelineThread","number of identiites instantiated %d", __facerecog->get_n_identities()); 
//     for (std::map<int, std::string>::iterator i = __persons.begin(); i != __persons.end(); ++i ) {
//       __facerecog->add_identity(i->first, i->second);
//       __facerecog->add_identity( __facerecog->get_n_identities(), "NewPerson" ); 
//     }
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
