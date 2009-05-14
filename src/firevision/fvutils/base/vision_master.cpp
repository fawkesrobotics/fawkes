
/***************************************************************************
 *  vision_master.cpp - FireVision Vision Master
 *
 *  Created: Wed May 30 10:52:08 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

/** @class VisionMaster vision_master.h <fvutils/base/vision_master.h>
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
 * @fn CameraControl * VisionMaster::register_for_camera_control(const char *camera_string, CameraControl::TypeID type_id)
 * Retrieve a certain CameraControl for the specified Camera.
 * This control (if available) can be used to control certain aspects of the Camera.
 * @param camera_string Camera whose CameraControl shall be returned
 * @param type_id ID of the type of CameraControl which shall be returned
 * @return a pointer to the requested CameraControl.
 * @throws NotImplementedException This Camera doesn't implement the requested type of control
 * @throws UnknownCameraException Unknown Camera (Camera has not been loaded through register_for_camera)
 */

/** Virtual empty destructor. */
VisionMaster::~VisionMaster()
{
}
