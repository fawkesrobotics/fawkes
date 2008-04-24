
/***************************************************************************
 *  pipeline_thread.h - FireVision Facer Pipeline Thread
 *
 *  Created: Apr Sat 19 12:39:26 2008 (on the way to German Open 2008, Hannover)
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

#ifndef __FIREVISION_APPS_FACER_PIPELINE_THREAD_H_
#define __FIREVISION_APPS_FACER_PIPELINE_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

#include <interfaces/facer.h>

#include <fvutils/base/roi.h>

#include <string>
#include <list>

class Camera;
class SharedMemoryImageBuffer;
class TimeTracker;
class FacesClassifier;
class FaceRecognizer;

typedef struct _IplImage IplImage;

class FacerPipelineThread
: public Thread,
  public ConfigurableAspect,
  public LoggingAspect,
  public VisionAspect,
  public BlackBoardAspect
{
 public:
  FacerPipelineThread();
  virtual ~FacerPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  Camera *__cam;
  SharedMemoryImageBuffer *__shm;
  IplImage *__image;

  std::map<int, std::string> __persons;
 
  bool __roi_not_found_flag; 
  int __person_recognized_cnt;
  int __person_labels[10]; 

  TimeTracker *__tt;
  unsigned int __loop_count;
  unsigned int __ttc_capture;
  unsigned int __ttc_memcpy;
  unsigned int __ttc_dispose;

  FacesClassifier *__classifier;
  FacerInterface *__facer_if;
  FacerInterface::if_facer_opmode_t  __opmode;
  unsigned int __nos_new_to_save; 
  std::string __new_identity_name; 

  std::string __face_label;
  std::list<ROI> *__rois;

  std::string __cfg_haarcascade_file;
  float       __cfg_haar_scale_factor;
  int         __cfg_min_neighbours;
  std::string __cfg_dir_path;
  unsigned  int  __cfg_forest_size;
  unsigned int  __cfg_number_identities;

  unsigned int __saved_faces;

  FaceRecognizer *__facerecog;

};


#endif
