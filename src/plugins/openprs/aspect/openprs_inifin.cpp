
/***************************************************************************
 *  openprs_inifin.cpp - Fawkes OpenPRSAspect initializer/finalizer
 *
 *  Created: Mon Aug 18 15:32:20 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include <plugins/openprs/aspect/openprs_inifin.h>
#include <plugins/openprs/aspect/openprs_kernel_manager.h>
#include <plugins/openprs/utils/openprs_comm.h>
#include <plugins/openprs/utils/openprs_server_proxy.h>
#include <core/threading/thread_finalizer.h>
#include <utils/time/time.h>
#include <unistd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenPRSAspectIniFin <plugins/openprs/aspect/openprs_inifin.h>
 * OpenPRSAspect initializer/finalizer.
 * This initializer/finalizer will provide the OpenPRS node handle to
 * threads with the OpenPRSAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
OpenPRSAspectIniFin::OpenPRSAspectIniFin()
  : AspectIniFin("OpenPRSAspect")
{
  openprs_comm_ = NULL;
  // conservative default, better wait a little longer than fail
  kernel_timeout_sec_ = 30.;
}

/** Destructor. */
OpenPRSAspectIniFin::~OpenPRSAspectIniFin()
{
  delete openprs_comm_;
}



void
OpenPRSAspectIniFin::init(Thread *thread)
{
  OpenPRSAspect *openprs_thread;
  openprs_thread = dynamic_cast<OpenPRSAspect *>(thread);
  if (openprs_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "OpenPRSAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  openprs_kernel_mgr_->create_kernel(openprs_thread->openprs_kernel_name,
				     openprs_thread->openprs_kernel_mode == OpenPRSAspect::XOPRS,
				     openprs_thread->openprs_data_paths_,
				     openprs_thread->openprs_gdb_delay_);

  try {
    openprs_thread->openprs = new OpenPRSComm(openprs_thread->openprs_local_name.c_str(),
					      openprs_kernel_mgr_->mp_host().c_str(),
					      openprs_kernel_mgr_->mp_port(),
					      openprs_server_proxy_);
  } catch (Exception &e) {
    openprs_kernel_mgr_->destroy_kernel(openprs_thread->openprs_kernel_name);
    throw;
  }

  fawkes::Time now, start;
  while (! openprs_server_proxy_->has_kernel(openprs_thread->openprs_kernel_name)) {
    now.stamp();
    if ((now - &start) > kernel_timeout_sec_) {
      openprs_kernel_mgr_->destroy_kernel(openprs_thread->openprs_kernel_name);
      throw Exception("OpenPRSAspect: timeout waiting for kernel startup");
    }
    usleep(100000);
  }

  openprs_comm_->transmit_command_f(openprs_thread->openprs_kernel_name,
				    "add (! (= @@FAWKES_MOD_DIR \"%s\"))", OPENPRS_MOD_DIR);
  openprs_comm_->transmit_command_f(openprs_thread->openprs_kernel_name,
				    "add (! (= @@FAWKES_HOST \"%s\"))", fawkes_host_.c_str());
  openprs_comm_->transmit_command_f(openprs_thread->openprs_kernel_name,
				    "add (! (= @@FAWKES_PORT \"%u\"))", fawkes_port_);
  openprs_comm_->transmit_command_f(openprs_thread->openprs_kernel_name,
				    "declare symbol %s",
				    openprs_thread->openprs_local_name.c_str());
  openprs_comm_->transmit_command_f(openprs_thread->openprs_kernel_name,
				    "add (! (= @@FAWKES_MP_NAME %s))",
				    openprs_thread->openprs_local_name.c_str());

  usleep(200000);
}

void
OpenPRSAspectIniFin::finalize(Thread *thread)
{
  OpenPRSAspect *openprs_thread;
  openprs_thread = dynamic_cast<OpenPRSAspect *>(thread);
  if (openprs_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"OpenPRSAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  openprs_kernel_mgr_->destroy_kernel(openprs_thread->openprs_kernel_name);
  openprs_thread->finalize_OpenPRSAspect();
}


/** Prepare OpenPRS aspect initializer.
 * @param fawkes_host Hostname where Fawkes is running
 * @param fawkes_port TCP port where Fawkes listens on
 * @param openprs_kernel_mgr OpenPRS kernel manager
 * @param openprs_server_proxy OpenPRS server proxy
 * @param openprs_mp_proxy OpenPRS Message Passer proxy
 */
void
OpenPRSAspectIniFin::prepare(const std::string &fawkes_host, unsigned short fawkes_port,
			     LockPtr<OpenPRSKernelManager> &openprs_kernel_mgr,
			     OpenPRSServerProxy *openprs_server_proxy,
			     OpenPRSMessagePasserProxy *openprs_mp_proxy)
{
  fawkes_host_ = fawkes_host;
  fawkes_port_ = fawkes_port;
  openprs_kernel_mgr_ = openprs_kernel_mgr;
  openprs_server_proxy_ = openprs_server_proxy;
  openprs_mp_proxy_     = openprs_mp_proxy;

  openprs_comm_ = new OpenPRSComm("OpenPRSAspect",
				  openprs_kernel_mgr_->mp_host().c_str(),
				  openprs_kernel_mgr_->mp_port(),
				  openprs_server_proxy_);
}


/** Set timeout for kernel creation.
 * @param timeout_sec timeout in seconds after which kernel creation
 * is assumed to have failed.
 */
void
OpenPRSAspectIniFin::set_kernel_timeout(float timeout_sec)
{
  kernel_timeout_sec_ = timeout_sec;
}

} // end namespace fawkes
