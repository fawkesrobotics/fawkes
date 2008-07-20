
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

#include <firevision/fvutils/color/colorspaces.h>
#include <apps/facer/pipeline_thread.h>
#include <fvutils/writers/png.h>
#include <stdio.h>
#include <cams/camera.h>
#include <fvutils/ipc/shm_image.h>
#include <utils/time/tracker.h>
#include <utils/time/time.h>
#include <utils/time/clock.h>
#include <fvutils/writers/seq_writer.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/recognition/faces.h>
#include <fvutils/adapters/iplimage.h>
#include <classifiers/faces.h>
#include <fvutils/recognition/forest/forest_aux.h>
#include <fvutils/recognition/forest/forest.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstring>
#include <map>
#include <string>

#define __SUBSEQ_FACES 3

using namespace fawkes;

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
  __roi_not_found_flag = true; 
  __person_recognized_cnt = 0; 
  __nos_new_to_save = 40;
  __new_identity_name = "Ceaser"; 
  __time_det = new fawkes::Time(clock);

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

  logger->log_info("FacerPipelineThread","training:%s, number:%d, forest_size:%d .", (__cfg_dir_path).c_str(), __cfg_number_identities, __cfg_forest_size); 
  __facerecog  = new FaceRecognizer( (__cfg_dir_path).c_str(),  __cfg_number_identities, __cfg_forest_size );

//   bool debug = false;
//   if( debug )
//     { 

//       // works fine 
//       int train_height, train_width; 
//       ForestConfigClass configClassInstance( __cfg_number_identities ); 
//       char buffer[PATH_MAX];
//       strcpy( buffer, (__cfg_dir_path).c_str() ); 
//       ForestClass* fcI = new ForestClass( buffer, __cfg_number_identities, train_height, train_width, configClassInstance, __cfg_forest_size ); 
//       IplImage *test = getImageFromLocation("/home/robocup/databases/hannover_20_04/organized/testing/2/timniemuller52.png"); 
//       logger->log_info("FacerPipleLineThread","debug mode: %d", getClassLabelFromForest( fcI, test ));
//       delete fcI;
//       cvReleaseImage( &test ); 
//     }
  

  //  logger->log_info("FacerPipelineThread","number of identiites instantiated %d", __facerecog->get_n_identities()); 
  for (std::map<int, std::string>::iterator i = __persons.begin(); i != __persons.end(); ++i ) {
    //@comment:works:    logger->log_info("FacerPipelineThread","%d %s", i->first, (i->second).c_str() ); 
    __facerecog->add_identity(i->first, i->second);
  }
  //  logger->log_info("FacerPipelineThread","number of identiites instantiated %d after instation", __facerecog->get_n_identities()); 
    __opmode = FacerInterface::OPMODE_RECOGNITION;
  //    __opmode = FacerInterface::OPMODE_LEARNING; 
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
  //bool debug = true; 
  fawkes::Time time_det_now(clock);

  __cam->capture();
  memcpy(__shm->buffer(), __cam->buffer(), __cam->buffer_size());
  IplImageAdapter::convert_image_bgr(__cam->buffer(), __image);

//   bool save_all_objects = false; // Hannover 2008 hack. remove this part .. need a seperate class
//   if( save_all_objects ) 
//     { 
//       char *buffer;
//       asprintf( &buffer, "%d.png", ++__saved_faces );
//       cvvSaveImage( buffer, __image ); 
//       free( buffer );

//     }
  

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

      if( roi.width < 70 || roi.height < 70 ) //heuristically set 
	{ 
	  logger->log_info("FacerPipelineThread", " the roi is too small");
	  __facer_if->set_num_detections(0);
	  __facer_if->set_sec_since_detection(time_det_now - __time_det);
	  __facer_if->write();
	  
	  break; 
	}
      
      __facer_if->set_sec_since_detection(0); 
      __time_det->stamp();
      __facer_if->set_num_detections((int)__rois->size());
      __facer_if->write();

      cvSetImageROI(__image, roi_rect);
      IplImage *face = cvCreateImage( cvSize(roi.width, roi.height),
				      __image->depth, __image->nChannels); 

      cvCopyImage( __image, face );  
      cvResetImageROI(__image);

      
      
      IplImage *scaled_face = cvCreateImage( cvSize(48, 48),
                                             __image->depth, __image->nChannels);
      
      cvResize(face, scaled_face, CV_INTER_LINEAR);
      //      cvReleaseImage( &face ); 
      
      
//       if( debug ) { 
// 	char *buffer;
// 	asprintf( &buffer,"face-%d.png", ++__saved_faces ); 
// 	cvvSaveImage( buffer, scaled_face ); 
// 	asprintf( &buffer, "face-original-%d.png", __saved_faces );
// 	cvvSaveImage( buffer, face ); 
// 	free( buffer );
//       }

      cvReleaseImage( &face ); 
      
      std::vector<IplImage *> face_images;
      face_images.push_back(scaled_face);
      //      cvReleaseImage( &scaled_face ); 
      // the second parameter, number_of_identities, is 0 since we are NOT learning new ppl at the momnent
      
      std::vector<int> recognition_indices = __facerecog->recognize( face_images, 0 ); 
      logger->log_info("FacerPipelineThread","number of recognized ppl is %d", recognition_indices.size() ); 
      int recognition_index = recognition_indices.at(0); 
      
      logger->log_info("FacerPipelineThread","recognition_index is %d", recognition_index ); 
      
      std::vector<std::string> face_labels = __facerecog->get_identities(recognition_indices); 
      std::string l_face_label; 
      if( face_labels.size() != 0 ) {
	l_face_label = face_labels[0]; //l_face_label is the name of the recognized person
        logger->log_info("FacerPipelineThread", "Current Face is recognized, person is %d: %s", recognition_index, l_face_label.c_str());
	if( __roi_not_found_flag ) //previously a face was not detected
	  { 
	    if( __person_recognized_cnt == 0 ) // start recording __SUBSEQ_FACES recognitions 
	      {
		__roi_not_found_flag = false; // reset the flag until another no-roi found
		__last_recognized_person_index = -1; 
		__person_labels[__person_recognized_cnt++] = recognition_index; // first recognition
		__person_names[__person_recognized_cnt - 1] = l_face_label; 
	      }
	    else 
	      {
		__person_recognized_cnt = 0; // person went away, stop recording - reset flag
	      }
	  }
	else if( __last_recognized_person_index == recognition_index  ) 
	  { 
	    logger->log_info("FacerPipelineThread","Recognized identity is still %s", l_face_label.c_str() ); 
	    __person_recognized_cnt = 0;
	  }
	else 
	  {
	    if( __person_recognized_cnt < __SUBSEQ_FACES ) // if below __SUBSEQ_FACES 
	      { 
		__person_labels[__person_recognized_cnt++] = recognition_index;  // subsequent recognitions 
		__person_names[__person_recognized_cnt -1] = l_face_label; 
	      }
	    else 
	      {
		int histogram[__facerecog->get_n_identities()];  // all recorded 
		for( int i = 0; i < __facerecog->get_n_identities(); i++ )
		  histogram[i] = 0;
		
		for( int i = 0; i < __SUBSEQ_FACES; i++ ) 
		  {
		    ++histogram[__person_labels[i]];  //		    ++histogram[recognition_indices.at(i)]; // record recognitions for each
		  }



		int max_label = -1, max_recognized = -1;
		for( int i = 0; i < __facerecog->get_n_identities(); i++ )
		  {
		    if( histogram[i] > max_recognized ) //ginf max
		      {
			max_label = i;
			max_recognized = histogram[i];
		      }
		  }
		// can also put a threshold here
		logger->log_info("FacerPipelineThread","The final recognized identitey %d and %s", max_label, (__person_names[max_label]).c_str()); 
		__last_recognized_person_index = max_label; 
		__face_label = __person_names[max_label]; // fix this!
		//		__person_recognized_cnt = 0; 
		
	      }
	  }
      	
	cvReleaseImage( &scaled_face ); 	
      } else {
	logger->log_info("FacerPipelineThread", "No identity returned. Most probably forest has not been instantiated/training images not supplied."); 
      }
      
      __facer_if->set_face_label(__face_label.c_str());
    } 
    else {
      __face_label = "";
      __facer_if->set_num_detections(0);
      __facer_if->set_face_label( __face_label.c_str() ); 
      __facer_if->set_sec_since_detection(time_det_now - __time_det);
      logger->log_info("FacerPipelineThread", "No ROIs found during Face Recognition.");
      __roi_not_found_flag = true; 
    }
    break;
    
  case FacerInterface::OPMODE_LEARNING:
    // train new faces, recognition data is learned from current images
    __rois = __classifier->classify();
    if ( ! __rois->empty() && __saved_faces < __nos_new_to_save) {
      // pass only most dominant ROI (biggest one)
      logger->log_debug("FacerPipelineThread", "Smile, you're on robot TV!");

      ROI &roi = *(__rois->begin());
      CvRect roi_rect = cvRect(roi.start.x, roi.start.y, roi.width, roi.height);

      

      if( roi.width < 70 || roi.height < 70) //heuristically set 
	{ 
	  logger->log_info("FacerPipelineThread", " the roi is too small");
	  break; 
	}

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

      char* dirname;
      asprintf( &dirname, "%s%d", (__cfg_dir_path).c_str(), __facerecog->get_n_identities() ); 
      //      mkdir( dirname, 777 );  //this mkidr statement is giving me problems 
	
      //	logger->log_info("FacerPipelineThread","NUMBER Of identiites = %d", __facerecog->get_n_identities() ); 
      //      asprintf( &buffer,"%s%d/%d.png",(__cfg_dir_path).c_str(), __facerecog->get_n_identities(), ++__saved_faces ); 

      ++__saved_faces;
      //      asprintf( &buffer, "%d.png", __saved_faces );  // save a copy locally

      char *new_name;
      asprintf( &new_name, "%s%d/%d.png", (__cfg_dir_path).c_str(), __facerecog->get_n_identities(), __saved_faces );
      PNGWriter pngwr(new_name, scaled_face->height, scaled_face->width ); 
      pngwr.set_buffer( RGB, (unsigned char*)scaled_face->imageData );
      pngwr.write();  
 

      //      cvvSaveImage( buffer, scaled_face ); 


      //      rename( buffer, new_name ); 
      //rename("%d.png","%s%d/%d.png", __saved_faces, (__cfg_dir_path).c_str(), __facerecog->get_n_identities(), __saved_faces );
      logger->log_debug("FacerPipelineThread", "Face saved to %s", new_name);
	
      free( new_name ); 
      free( dirname ); 
      //      free( buffer );
      //    }

      cvReleaseImage(&face);
      cvReleaseImage(&scaled_face);
    } else {
      logger->log_info("FacerPipelineThread", "No ROIs found");
    }

    if( __saved_faces == __nos_new_to_save )
      { 
	// @TODO: do the procedure below in the calling functoin 
	
	//    std::map<int, std::string> persons;
	try {
	  
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
	  __persons[__cfg_number_identities] = __new_identity_name; 
	  
	} catch (Exception &e) {
	  throw;
	}
	
	logger->log_info("FacerPipelineThread","Previous number of identities were (%d, %d ) and forest size of %d", __facerecog->get_n_identities(), __cfg_number_identities +1 , __cfg_forest_size );  
	delete __facerecog; 
	
	
	__facerecog  = new FaceRecognizer( (__cfg_dir_path).c_str(),  __cfg_number_identities + 1, __cfg_forest_size );
	//  logger->log_info("FacerPipelineThread","number of identiites instantiated %d", __facerecog->get_n_identities()); 
	for (std::map<int, std::string>::iterator i = __persons.begin(); i != __persons.end(); ++i ) {
	  __facerecog->add_identity(i->first, i->second);
	  //	  __facerecog->add_identity( __facerecog->get_n_identities(), "NewPerson" ); 
	}


	//check if there is any entry in the config setup for additional new persons
	char tmpBuffer[256];
	sprintf(tmpBuffer,"/firevision/facer/identity_%d", __cfg_number_identities );
	
	Configuration::ValueIterator *jVI = config->search(tmpBuffer);
	if( jVI->next() && jVI->is_string() )
	  { 
	    __new_identity_name = jVI->get_string();
	  }
	else
	  { //config file doesnt contain details for the new person name
	    char tmpBuffer[256];
	    sprintf(tmpBuffer,"newperson%d", __cfg_number_identities + 1); 
	    __new_identity_name = tmpBuffer; //DUMMY NAME
	  }
	
	__facerecog->add_identity( __cfg_number_identities, __new_identity_name ); 
	++__cfg_number_identities;

	//	logger->log_info("FacerPipelineThread","The cfg_num, forest bin size and name %d, %s", __cfg_number_identities, __facerecog->get_n_identities(), __new_identity_name.c_str() ); 
	__saved_faces = 0; 
	//done learning .. now go to recognition 
	__opmode = FacerInterface::OPMODE_RECOGNITION;
	logger->log_info("FacerPiplelineThread","New identity:%s saved", (__new_identity_name).c_str() ); 
      }
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
