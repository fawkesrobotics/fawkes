
/***************************************************************************
 *  camargp.h - Camera argument parser
 *
 *  Created: Wed Apr 11 15:44:51 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_SYSTEM_CAMARGP_H_
#define __FIREVISION_FVUTILS_SYSTEM_CAMARGP_H_

#include <fvcams/camera.h>

#include <map>
#include <string>
#include <vector>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser
{
 public:
  CameraArgumentParser(const char *as);
  ~CameraArgumentParser();

  bool                                has(std::string s) const;
  std::string                         get(std::string s) const;
  long int                            get_int(std::string s) const;
  double                              get_float(std::string s) const;
  std::map<std::string, std::string>  parameters() const;
  std::vector<std::string>            arguments() const;
  std::string                         cam_id() const;
  std::string                         cam_type() const;

 private:
  std::string _cam_type;
  std::string _cam_id;
  std::map<std::string, std::string> values;
  std::vector<std::string> args;
};

} // end namespace firevision

#endif
