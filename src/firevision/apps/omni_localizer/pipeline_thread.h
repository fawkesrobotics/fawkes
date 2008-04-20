/*
    Copyright (c) 2008 Volker Krause <volker.krause@rwth-aachen.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#ifndef FVOMNILOCALIZERPIPELINETHREAD_H
#define FVOMNILOCALIZERPIPELINETHREAD_H

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/roi.h>

class Camera;
class ScanlineStar;
class ColorModel;
class LineClassifier;
class SharedMemoryImageBuffer;
class MirrorModel;
class MCL;
class MotorInterface;
class ObjectPositionInterface;

class FvOmniLocalizerPipelineThread
: public Thread,
  public LoggingAspect,
  public VisionAspect,
  public ConfigurableAspect,
  public BlackBoardAspect
{
 public:
  FvOmniLocalizerPipelineThread();
  virtual ~FvOmniLocalizerPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

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
  MotorInterface *mMotorInterface;
  ObjectPositionInterface *mPositionInterface;

  unsigned int mImageWidth, mImageHeight;
  float mLowerRange, mUpperRange;

  colorspace_t mColorspaceFrom, mColorspaceTo;
};


#endif
