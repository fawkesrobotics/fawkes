
/***************************************************************************
 *  openprs.cpp - OpenPRS aspect for Fawkes
 *
 *  Created: Sat Jun 16 14:30:44 2012
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include <plugins/openprs/aspect/openprs.h>
#include <plugins/openprs/utils/openprs_comm.h>
#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenPRSAspect <plugins/openprs/aspect/openprs.h>
 * OpenPRS kernel creation and communication aspect.
 * This aspect allows access to a specific OpenPRS context through the
 * OpenPRSKernel communication wrapper. The context is created if it does
 * not already exist.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes:LockPtr<OpenPRSKernel> OpenPRSAspect::openprs
 * OpenPRS kernel communication wrapper.
 */

/** @var const std::string OpenPRSAspect::openprs_kernel_name
 * The name of the kernel created for this thread.
 */

/** @var const Mode OpenPRSAspect::openprs_kernel_mode
 * The kernel mode, can be OPRS or XOPRS (with graphical interface).
 */

/** @var const std::string OpenPRSAspect::openprs_local_name
 * The local message passer name for communication.
 */

/** Constructor.
 * @param kernel_name the name of the OpenPRS kernel to connect to.
 * The context may not exist, yet.
 * @param mode set to XOPRS to run kernel with graphical user interface,
 * OPRS to run headless (default)
 * @param local_name local name to register with to the message passer.
 * If NULL will be set to "fawkes-|kernel_name|" (where |kernel_name|
 * will be replaced by the value of @p kernel_name).
 */
OpenPRSAspect::OpenPRSAspect(const char *kernel_name, OpenPRSAspect::Mode mode, const char *local_name)
  : openprs_kernel_name(kernel_name), openprs_kernel_mode(mode),
    openprs_local_name(local_name ? local_name : std::string("fawkes-") + kernel_name),
    openprs_gdb_delay_(false)
{
  add_aspect("OpenPRSAspect");
  if (openprs_local_name.find_first_of(" \t\n") != std::string::npos) {
    throw Exception("Local name may not contains spaces");
  }
}


/** Virtual empty destructor. */
OpenPRSAspect::~OpenPRSAspect()
{
}


/** Add an OpenPRS data path.
 * The paths are added to the kernel on intialization and are
 * then searched when including and loading files.
 * Note that this method may only be called in the constructor,
 * i.e. before the aspect is initialized.
 * @param path path to add to search list
 */
void
OpenPRSAspect::add_openprs_data_path(const std::string &path)
{
  if (openprs) {
    throw Exception("OpenPRS kernel has already been intialized");
  }
  openprs_data_paths_.push_back(path);
}


/** Enable/disable GDB delay.
 * This can be used to order mod_utils to wait for a few seconds to allow
 * for connecting to the OPRS kernel before it is actually running.
 * @param enable_gdb_delay true to enable delay, false to disable (default)
 */
void
OpenPRSAspect::set_openprs_gdb_delay(const bool enable_gdb_delay)
{
  if (openprs) {
    throw Exception("OpenPRS kernel has already been intialized");
  }
  openprs_gdb_delay_ = enable_gdb_delay;
}

/** Init OpenPRS aspect.
 * This sets the OpenPRS kernel communication wrapper.
 * It is guaranteed that this is called for a OpenPRS Thread before start
 * is called (when running regularly inside Fawkes).
 * @param oprs_kernel OpenPRS kernel communication wrapper
 */
void
OpenPRSAspect::init_OpenPRSAspect(LockPtr<OpenPRSComm> oprs_comm)
{
  this->openprs = oprs_comm;
}

/** Finalize OpenPRS aspect.
 * This clears the OpenPRS environment.
 */
void
OpenPRSAspect::finalize_OpenPRSAspect()
{
  openprs.clear();
}


} // end namespace fawkes
