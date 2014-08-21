
/***************************************************************************
 *  openprs_inifin.h - Fawkes OpenPRSAspect initializer/finalizer
 *
 *  Created: Mon Aug 18 15:32:11 2014
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

#ifndef __PLUGINS_OPENPRS_ASPECT_OPENPRS_INIFIN_H_
#define __PLUGINS_OPENPRS_ASPECT_OPENPRS_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <plugins/openprs/aspect/openprs.h>

#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class OpenPRSKernelManager;
class OpenPRSServerProxy;
class OpenPRSMessagePasserProxy;

class OpenPRSAspectIniFin : public AspectIniFin
{
 public:
  OpenPRSAspectIniFin();
  ~OpenPRSAspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);

  void set_manager(LockPtr<OpenPRSKernelManager> &openprs_kernel_mgr);
  void set_proxies(OpenPRSServerProxy *openprs_server_proxy,
		   OpenPRSMessagePasserProxy *openprs_mp_proxy);

 private:
  LockPtr<OpenPRSKernelManager>  openprs_kernel_mgr_;
  OpenPRSServerProxy            *openprs_server_proxy_;
  OpenPRSMessagePasserProxy     *openprs_mp_proxy_;
};

} // end namespace fawkes

#endif
