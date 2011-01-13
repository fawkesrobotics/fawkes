
/***************************************************************************
 *  vision_master.h - FireVision Vision Master
 *
 *  Created: Wed May 30 10:28:06 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __FIREVISION_FVUTILS_BASE_VISION_MASTER_H_
#define __FIREVISION_FVUTILS_BASE_VISION_MASTER_H_

#include <fvutils/color/colorspaces.h>
#include <fvcams/control/control.h>
#include <core/utils/refptr.h>
#include <core/exceptions/software.h>

#include <typeinfo>

namespace fawkes {
  class Thread;
  class TypeMismatchException;
}

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Camera;

class VisionMaster
{
 public:
  virtual ~VisionMaster();

  virtual Camera *  register_for_camera(const char *camera_string,
					fawkes::Thread *thread,
					colorspace_t cspace = YUV422_PLANAR) = 0;
  virtual Camera *  register_for_raw_camera(const char *camera_string,
					    fawkes::Thread *thread)          = 0;
  virtual void      unregister_thread(fawkes::Thread *thread)                = 0;

  virtual CameraControl *acquire_camctrl(const char *cam_string)             = 0;
  virtual void           release_camctrl(CameraControl *cc)                  = 0;


 /** Retrieve a typed camera control instance.
  * Like the non-template method this class will try to instantiate the camera
  * control based on the camera string (see there for details on the two possible
  * contents of the string). The camera control will be verified to be of the
  * desired type.
  * @param camera_string camera string of camera for the control or the argument
  * string for a new instance. See documentation of non-template method.
  * @return typed camera control instance
  * @exception TypeMismatchException thrown if requested camera control does not
  * match the requested type.
  */
  template <class CC>
    CC *
    acquire_camctrl(const char *camera_string);

 /** Retrieve a typed camera control instance.
  * Like the non-template method this class will try to instantiate the camera
  * control based on the camera string (see there for details on the two possible
  * contents of the string). The camera control will be verified to be of the
  * desired type.
  * Unlike the other template method you do not need to explicitly state the type
  * of the requested type as it is inferred based on the \p cc argument. You can
  * write something like this:
  * @code
  * CameraControlImage *cci = vision_master->acquire_camctrl("camstring...", cci);
  * @endcode
  * instead of
  * @code
  * CameraControlImage *cci = vision_master->acquire_camctrl<CameraControlImage>("camstring...");
  * @endcode
  * @param camera_string camera string of camera for the control or the argument
  * string for a new instance. See documentation of non-template method.
  * @param cc reference to pointer of camera control of the intended type, used
  * to automatically infer the template method. On successful return points to
  * the camera control instance
  * @return typed camera control instance (same as \p cc)
  * @exception TypeMismatchException thrown if requested camera control does not
  * match the requested type.
  */
  template <class CC>
    CC *
    acquire_camctrl(const char *camera_string, CC *&cc);


  /** Get typed raw camera.
   * Like the non-template variant this method can be used to get access to
   * the raw camera implementation, without going through a proxy. See the other
   * method for risks and implication of using the raw device.
   * @param camera_string camera that can be used by CameraFactory to open a
   * camera.
   * @param thread thread to register for this camera
   * @return typed raw camera
   */
  template <class CC>
    CC *
    register_for_raw_camera(const char *camera_string, fawkes::Thread *thread);


 protected:
  virtual CameraControl *acquire_camctrl(const char *cam_string,
					 const std::type_info &typeinf) = 0;

};

template <class CC>
CC *
VisionMaster::acquire_camctrl(const char *camera_string, CC *&cc)
{
  const std::type_info &t = typeid(CC);
  CameraControl *pcc = acquire_camctrl(camera_string, t);
  CC *tcc = dynamic_cast<CC *>(pcc);
  if (tcc) {
    if (cc) cc = tcc;
    return tcc;
  } else {
    release_camctrl(tcc);
    throw fawkes::TypeMismatchException("CameraControl defined by string does "
					"not match desired type");
  }
}


template <class CC>
CC *
VisionMaster::acquire_camctrl(const char *camera_string)
{
  const std::type_info &t = typeid(CC);
  CameraControl *pcc = acquire_camctrl(camera_string, t);
  CC *tcc = dynamic_cast<CC *>(pcc);
  if (tcc) {
    return tcc;
  } else {
    release_camctrl(tcc);
    throw fawkes::TypeMismatchException("CameraControl defined by string does "
					"not match desired type");
  }
}

template <class CC>
CC *
VisionMaster::register_for_raw_camera(const char *camera_string, fawkes::Thread *thread)
{
  Camera *camera = register_for_raw_camera(camera_string, thread);
  CC *tcc = dynamic_cast<CC *>(camera);
  if (tcc) {
    return tcc;
  } else {
    unregister_thread(thread);
    throw fawkes::TypeMismatchException("Camera defined by string does "
					"not match desired type");
  }
}

} // end namespace firevision

#endif
