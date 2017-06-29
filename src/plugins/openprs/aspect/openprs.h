
/***************************************************************************
 *  openprs.h - OpenPRS aspect for Fawkes
 *
 *  Created: Mon Aug 18 14:33:35 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENPRS_ASPECT_OPENPRS_H_
#define __PLUGINS_OPENPRS_ASPECT_OPENPRS_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>
#include <string>
#include <list>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenPRSComm;
class OpenPRSAspectIniFin;

class OpenPRSAspect : public virtual Aspect
{
 friend OpenPRSAspectIniFin;

 public:
  /** OPRS kernel operation mode. */
  typedef enum {
    OPRS,	///< command line mode
    XOPRS	///< graphical mode
  } Mode;

  OpenPRSAspect(const char *kernel_name, Mode mode = OPRS, const char *local_name = NULL);
  virtual ~OpenPRSAspect();

  void add_openprs_data_path(const std::string &path);
  void set_openprs_gdb_delay(const bool enable_gdb_delay);

 protected:
  LockPtr<OpenPRSComm>   openprs;
  const std::string      openprs_kernel_name;
  const Mode             openprs_kernel_mode;
  const std::string      openprs_local_name;

 private:
  void init_OpenPRSAspect(LockPtr<OpenPRSComm> oprs_comm);
  void finalize_OpenPRSAspect();

  std::list<std::string> openprs_data_paths_;
  bool                   openprs_gdb_delay_;

};

} // end namespace fawkes

#endif
