
/***************************************************************************
 *  vision_master.cpp - FireVision Vision Master
 *
 *  Created: Wed May 30 10:52:08 2007
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

#include <fvutils/base/vision_master.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class VisionMaster <fvutils/base/vision_master.h>
 * Vision Master.
 * The vision master shall be the entry point for vision plugins. It shall
 * allow for requesting cameras that are opened in a central place such that
 * the very same camera can be used in multiple plugins.
 *
 * It shall also be responsible for the central timing of all vision threads.
 *
 * @author Tim Niemueller
 *
 * @fn Camera *  VisionMaster::register_for_camera(const char *camera_string, Thread *thread, colorspace_t cspace=YUV422_PLANAR) = 0
 * Register thread for camera.
 * This will register a relation between the given thread and the camera identified
 * by the camera string. If the requested camera has not been opened before this
 * is done and the camera is started. If that fails for whatever reason an exception
 * is thrown. In that case the thread is not registered with the vision master.
 * If the camera is available the thread is registered with the vision master. From
 * then on it is woken up whenever new image data is available and it will wait for
 * the thread to finished computation on that very image. It is a critical error
 * that can not be recovered if the thread fails for whatever reason. If there is
 * a critical error condition in the vision thread it must not stop execution
 * but just the computation.
 * @param camera_string camera that can be used by CameraFactory to open a
 * camera.
 * @param thread thread to register for this camera
 * @param cspace the colorspace in which the images should be provided for the
 * camera. Note that using images in different formats at the same time can cause
 * a severe performance penalty. The default is to produce YUV422_PLANAR images,
 * which is used in the FireVision framework as main image format.
 * @return a pointer to the requested camera. Note that this may not be
 * of the C++ type that you may expect for the requested camera, but it may
 * have layers of indirection. For example when opening a USB camera you could
 * get a shared memory camera to share the camera (image) with multiple threads.
 * Note that using CS_UNKNOWN shall have the similar result as using
 * register_for_raw_camera().
 *
 * @fn Camera *  VisionMaster::register_for_raw_camera(const char *camera_string, Thread *thread)
 * Register thread for camera.
 * This will register a relation between the given thread and the camera identified
 * by the camera string similar to register_for_camera(). However, unlike
 * register_for_camera() this method will provide access to the raw camera
 * implementation, without possibly proxies. Once you gathered the camera, you
 * can dynamically cast it to the expected camera type (or use the template method
 * instead. Raw access to a camera is only granted for a single thread.
 * Note that you may not call capture() or dispose() on the camera, this will
 * still be done by the vision master, as the camera may be used by other
 * threads that registered for the camera with register_for_camera().
 * @param camera_string camera that can be used by CameraFactory to open a
 * camera.
 * @param thread thread to register for this camera
 * @return raw camera instance, which can by dynamically casted to the expected type.
 *
 * @fn void VisionMaster::unregister_thread(Thread *thread) = 0
 * Unregister a thread.
 * The thread is unregistered and it is removed from the internal structures. The
 * thread is no longer called for new image material that can be processed.
 *
 * If the unregistered thread was the last thread accessing the camera, it shall
 * be held open for a specified time, such that if the thread is just being
 * restarted the camera does not have to be re-opened. The time to wait is
 * defined by the implementation.
 * @param thread thread to unregister
 * 
 * @fn CameraControl * VisionMaster::acquire_camctrl(const char *cam_string)
 * Retrieve a CameraControl for the specified camera string.
 * This control (if available) can be used to control certain aspects of the Camera.
 * The \p cam_string argument either is the string that has been used to register
 * for a particular camera, or it is a string denoting a camera control by itself.
 * In the former case the vision master will look if the camera has been registered,
 * and then checks if the camera provides a camera control. If so the control is
 * returned. Note that it might implement multiple different camera controls. If
 * you want a specific camera control use one of the template methods to get a
 * correctly typed and verified control. If no camera that matches the \p cam_string
 * is found, the vision master will try to instantiate a new camera control using
 * the \p cam_string as argument to the CameraControlFactory.
 * @param cam_string Camera argument string, see method description for details
 * @return a pointer to the requested CameraControl.
 * @throws Exception no camera was found matching the \p cam_string and the factory
 * could not instantiate a camera control with the given string.
 *
 * @fn CameraControl * VisionMaster::acquire_camctrl(const char *cam_string, const std::type_info &typeinf)
 * Retrieve a CameraControl for the specified camera string and type info.
 * This utility method is used by the template methods to instantiate the cameras
 * with a specified intended type.
 * @param cam_string Camera argument string, see method description for details
 * @param typeinf type info for intended camera control type
 * @return a pointer to the requested CameraControl.
 * @throws Exception no camera was found matching the \p cam_string and the factory
 * could not instantiate a camera control with the given string.
 *
 * @fn void VisionMaster::release_camctrl(CameraControl *cc)
 * Release a camera control.
 * This has to be called when you are done with the camera control. This will
 * release the control and it is no longer valid. The vision master might collect
 * the memory that has been used for the control.
 * @param cc camera control instance to release
 */

/** Virtual empty destructor. */
VisionMaster::~VisionMaster()
{
}

} // end namespace firevision
