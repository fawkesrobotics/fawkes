
/***************************************************************************
 *  acquisition_thread.h - FireVision Acquisition Thread
 *
 *  Created: Wed Jun 06 19:01:10 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include "acquisition_thread.h"

#include "aqt_vision_threads.h"

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/wait_condition.h>
#ifdef FVBASE_TIMETRACKER
#	include <utils/time/clock.h>
#	include <utils/time/tracker.h>
#endif
#include <fvcams/shmem.h>
#include <fvutils/color/conversions.h>
#include <interfaces/SwitchInterface.h>
#include <logging/logger.h>

#ifndef _GNU_SOURCE
#	define _GNU_SOURCE
#endif
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace fawkes;
using namespace firevision;

/** @class FvAcquisitionThread "acquisition_thread.h"
 * FireVision base application acquisition thread.
 * This thread is used by the base application to acquire images from a camera
 * and call dependant threads when new images are available so that these
 * threads can start processing the images.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger
 * @param id id to be used for the shared memory segment and to announce changes
 * to the base thread
 * @param camera camera to manage
 * @param clock clock to use for timeout measurement (system time)
 */
FvAcquisitionThread::FvAcquisitionThread(const char *id,
                                         Camera *    camera,
                                         Logger *    logger,
                                         Clock *     clock)
: Thread("FvAcquisitionThread"), BlackBoardInterfaceListener("FvAcquisitionThread::%s", id)
{
	set_prepfin_conc_loop(true);
	set_name("FvAcquisitionThread::%s", id);

	image_id_ = strdup(id);

	vision_threads        = new FvAqtVisionThreads(clock);
	raw_subscriber_thread = NULL;

	enabled_mutex_    = new Mutex(Mutex::RECURSIVE);
	enabled_waitcond_ = new WaitCondition(enabled_mutex_);

	camera_     = camera;
	width_      = camera_->pixel_width();
	height_     = camera_->pixel_height();
	colorspace_ = camera_->colorspace();

	mode_    = AqtContinuous;
	enabled_ = false;

#ifdef FVBASE_TIMETRACKER
	tt_          = new TimeTracker();
	loop_count_  = 0;
	ttc_capture_ = tt_->add_class("Capture");
	ttc_lock_    = tt_->add_class("Lock");
	ttc_convert_ = tt_->add_class("Convert");
	ttc_unlock_  = tt_->add_class("Unlock");
	ttc_dispose_ = tt_->add_class("Dispose");
#endif
}

/** Destructor. */
FvAcquisitionThread::~FvAcquisitionThread()
{
	camera_->close();

	for (shmit_ = shm_.begin(); shmit_ != shm_.end(); ++shmit_) {
		delete shmit_->second;
	}
	shm_.clear();

	delete vision_threads;
	delete camera_;
	free(image_id_);
	delete enabled_waitcond_;
	delete enabled_mutex_;
}

void
FvAcquisitionThread::init()
{
	logger->log_debug(
	  name(), "Camera opened, w=%u  h=%u  c=%s", width_, height_, colorspace_to_string(colorspace_));

	std::string if_id = std::string("Camera ") + image_id_;
	enabled_if_       = blackboard->open_for_writing<SwitchInterface>(if_id.c_str());
	enabled_if_->set_enabled(enabled_);
	enabled_if_->write();

	bbil_add_message_interface(enabled_if_);
	blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
}

void
FvAcquisitionThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->close(enabled_if_);
}

/** Get a camera instance.
 * This will return a camera instance suitable for accessing the image
 * buffer. Note, that this is not the camera provided to the constructor,
 * but rather a SharedMemoryCamera instance accessing a shared memory buffer
 * where the image is copied to (or a conversion result is posted to).
 * The returned instance has to bee freed using delete when done with it.
 *
 * You can decide whether you want to get access to the raw camera image
 * that has not been modified in any way or to the YUV422_PLANAR image buffer
 * (a conversion is done if needed). Use the raw parameter to decide whether
 * to get the raw image (true) or the YUV422_PLANAR image (false).
 *
 * When a thread is added it is internally put into a waiting queue. Since
 * at the time when it is added the thread is not yet started, and its
 * initialization may even fail. For this reason the acquisition thread
 * registers itself to receive status notifications of the thread. If the
 * thread signals successful startup it is moved to the running queue and
 * from then on woken up when new image material can be processed. If the
 * thread fails for whatever reason it is dropped.
 *
 * The acquisition thread has a timeout. If no thread is in the running or
 * waiting queue for this number of seconds, the base thread is signalled
 * to shut down this acquisition thread (which the base thread may do or
 * deny). This is done so that if a plugin is just unloaded shortly and
 * then quickly loaded again the overhead of closing the camera and then
 * opening it again is avoided.
 *
 * @param cspace the desired colorspace the image should be converted to.
 * See general notes in VisionMaster::register_for_camera().
 * @param deep_copy given to the shared memory camera.
 * @return camera instance
 * @see SharedMemoryCamera
 */
Camera *
FvAcquisitionThread::camera_instance(colorspace_t cspace, bool deep_copy)
{
	const char *img_id = NULL;

	if (cspace == CS_UNKNOWN) {
		if (raw_subscriber_thread) {
			// There may be only one
			throw Exception("Only one vision thread may access the raw camera.");
		} else {
			return camera_;
		}
	} else {
		char *tmp = NULL;
		if (shm_.find(cspace) == shm_.end()) {
			if (asprintf(&tmp, "%s.%zu", image_id_, shm_.size()) == -1) {
				throw OutOfMemoryException("FvAcqThread::camera_instance(): Could not create image ID");
			}
			img_id       = tmp;
			shm_[cspace] = new SharedMemoryImageBuffer(img_id, cspace, width_, height_);
		} else {
			img_id = shm_[cspace]->image_id();
		}

		SharedMemoryCamera *c = new SharedMemoryCamera(img_id, deep_copy);

		if (tmp)
			free(tmp);
		return c;
	}
}

/** Get the Camera of this acquisition thread.
 * This is just used for the camera controls, if you want to access the camera,
 * use camera_instance()
 * @return a pointer to the Camera
 */
Camera *
FvAcquisitionThread::get_camera()
{
	return camera_;
}

/** Set acquisition thread mode.
 * Note that this may only be called on a stopped thread or an
 * exception will be thrown by Thread::set_opmode()!
 * @param mode new acquisition thread mode
 */
void
FvAcquisitionThread::set_aqtmode(AqtMode mode)
{
	if (mode == AqtCyclic) {
		//logger->log_info(name(), "Setting WAITFORWAKEUPMODE");
		set_opmode(Thread::OPMODE_WAITFORWAKEUP);
	} else if (mode == AqtContinuous) {
		//logger->log_info(name(), "Setting CONTINUOUS");
		set_opmode(Thread::OPMODE_CONTINUOUS);
	}
	mode_ = mode;
}

/** Enable or disable image retrieval.
 * When the acquisition thread is enabled image data will be converted or copied
 * to the shared memory buffer, otherwise only the capture/dispose cycle is
 * executed.
 * @param enabled true to enable acquisition thread, false to disable
 */
void
FvAcquisitionThread::set_enabled(bool enabled)
{
	MutexLocker lock(enabled_mutex_);

	if (enabled_ && !enabled) {
		// disabling thread
		camera_->stop();
		enabled_if_->set_enabled(false);
		enabled_if_->write();

	} else if (!enabled_ && enabled) {
		// enabling thread
		camera_->start();
		enabled_if_->set_enabled(true);
		enabled_if_->write();

		enabled_waitcond_->wake_all();
	} // else not state change

	// we can safely do this every time...
	enabled_ = enabled;
}

/** Get acquisition thread mode.
 * @return acquisition thread mode.
 */
FvAcquisitionThread::AqtMode
FvAcquisitionThread::aqtmode()
{
	return mode_;
}

/** Set prepfin hold status for vision threads.
 * @param hold prepfin hold status
 * @see Thread::set_prepfin_hold()
 */
void
FvAcquisitionThread::set_vt_prepfin_hold(bool hold)
{
	try {
		vision_threads->set_prepfin_hold(hold);
	} catch (Exception &e) {
		logger->log_warn(name(),
		                 "At least one thread was being finalized while prepfin hold "
		                 "was about to be acquired");
		throw;
	}
}

void
FvAcquisitionThread::loop()
{
	MutexLocker lock(enabled_mutex_);

	while (!enabled_if_->msgq_empty()) {
		if (enabled_if_->msgq_first_is<SwitchInterface::EnableSwitchMessage>()) {
			// must be re-established
			logger->log_info(name(), "Enabling on blackboard request");
			set_enabled(true);
		} else if (enabled_if_->msgq_first_is<SwitchInterface::DisableSwitchMessage>()) {
			logger->log_info(name(), "Disabling on blackboard request");
			set_enabled(false);
		} else {
			logger->log_warn(name(), "Unhandled message %s ignored", enabled_if_->msgq_first()->type());
		}
		enabled_if_->msgq_pop();
	}

	// We disable cancelling here to avoid problems with the write lock
	Thread::CancelState old_cancel_state;
	set_cancel_state(Thread::CANCEL_DISABLED, &old_cancel_state);

#ifdef FVBASE_TIMETRACKER
	try {
		if (enabled_) {
			tt_->ping_start(ttc_capture_);
			camera_->capture();
			tt_->ping_end(ttc_capture_);

			for (shmit_ = shm_.begin(); shmit_ != shm_.end(); ++shmit_) {
				if (shmit_->first == CS_UNKNOWN)
					continue;
				tt_->ping_start(ttc_lock_);
				shmit_->second->lock_for_write();
				tt_->ping_end(ttc_lock_);
				tt_->ping_start(ttc_convert_);
				convert(
				  colorspace_, shmit_->first, camera_->buffer(), shmit_->second->buffer(), width_, height_);
				try {
					shmit_->second->set_capture_time(camera_->capture_time());
				} catch (NotImplementedException &e) {
					// ignored
				}
				tt_->ping_end(ttc_convert_);
				tt_->ping_start(ttc_unlock_);
				shmit_->second->unlock();
				tt_->ping_end(ttc_unlock_);
			}
		}
	} catch (Exception &e) {
		logger->log_error(name(), "Cannot convert image data");
		logger->log_error(name(), e);
	}
	if (enabled_) {
		tt_->ping_start(ttc_dispose_);
		camera_->dispose_buffer();
		tt_->ping_end(ttc_dispose_);
	}

	if ((++loop_count_ % FVBASE_TT_PRINT_INT) == 0) {
		tt_->print_to_stdout();
	}

#else // no time tracking
	try {
		if (enabled_) {
			camera_->capture();
			for (shmit_ = shm_.begin(); shmit_ != shm_.end(); ++shmit_) {
				if (shmit_->first == CS_UNKNOWN)
					continue;
				shmit_->second->lock_for_write();
				convert(
				  colorspace_, shmit_->first, camera_->buffer(), shmit_->second->buffer(), width_, height_);
				try {
					shmit_->second->set_capture_time(camera_->capture_time());
				} catch (NotImplementedException &e) {
					// ignored
				}
				shmit_->second->unlock();
			}
		}
	} catch (Exception &e) {
		logger->log_error(name(), e);
	}
	if (enabled_) {
		camera_->dispose_buffer();
	}
#endif

	if (mode_ == AqtCyclic && enabled_) {
		vision_threads->wakeup_and_wait_cyclic_threads();
	}

	// reset to the original cancel state, cancelling is now safe
	set_cancel_state(old_cancel_state);

	// in continuous mode wait for signal if disabled
	while (mode_ == AqtContinuous && !enabled_) {
		enabled_waitcond_->wait();
	}
}

bool
FvAcquisitionThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	// in continuous mode wait for signal if disabled
	MutexLocker lock(enabled_mutex_);
	if (mode_ == AqtContinuous && !enabled_) {
		if (message->is_of_type<SwitchInterface::EnableSwitchMessage>()) {
			logger->log_info(name(), "Enabling on blackboard request");
			set_enabled(true);
			return false;
		}
	}

	return true;
}
