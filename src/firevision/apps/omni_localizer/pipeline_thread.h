
/***************************************************************************
 *  pipeline_thread.h - omni localizer pipeline thread
 *
 *  Created: ???
 *  Copyright  2008  Volker Krause <volker.krause@rwth-aachen.de>
 *
 *  $Id: pipeline_thread.cpp 1049 2008-04-24 22:40:12Z beck $
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

#ifndef __FIREVISION_APPS_OMNI_LOCALIZER_PIPELINE_THREAD_H_
#define __FIREVISION_APPS_OMNI_LOCALIZER_PIPELINE_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

#include <blackboard/interface_observer.h>
#include <blackboard/interface_listener.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/roi.h>

#include <vector>

class Camera;
class ScanlineStar;
class ColorModel;
class LineClassifier;
class SharedMemoryImageBuffer;
class MirrorModel;
class MCL;
namespace fawkes {
  class MotorInterface;
  class ObjectPositionInterface;
}

class FvOmniLocalizerPipelineThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::VisionAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceObserver,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  FvOmniLocalizerPipelineThread();
  virtual ~FvOmniLocalizerPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual void bb_interface_created(const char *type, const char *id) throw();
  virtual void bb_interface_writer_removed(fawkes::Interface *interface, unsigned int instance_serial) throw();

 private:
  Camera* mCamera;
  unsigned char* mMask;
  ScanlineStar* mScanlineModel;
  ColorModel* mColorModel;
  LineClassifier* mClassifier;
  SharedMemoryImageBuffer* mShmBuffer;
  unsigned char* mBuffer;
  MirrorModel *mMirror;
  MCL *mMCL;
  fawkes::MotorInterface *mMotorInterface;
  fawkes::ObjectPositionInterface *mPositionInterface;

  unsigned int mImageWidth, mImageHeight;
  float mLowerRange, mUpperRange;

  colorspace_t mColorspaceFrom, mColorspaceTo;

  bool mUseBallPosition;
  std::vector<fawkes::ObjectPositionInterface*> mBallInterfaces;

  bool mUseObstaclePositions;
  std::vector<fawkes::ObjectPositionInterface*> mObstacleInterfaces;
};


#endif
