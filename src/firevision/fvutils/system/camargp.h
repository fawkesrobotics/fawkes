
/***************************************************************************
 *  camargp.h - Camera argument parser
 *
 *  Created: Wed Apr 11 15:44:51 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_FVUTILS_SYSTEM_CAMARGP_H_
#define __FIREVISION_FVUTILS_SYSTEM_CAMARGP_H_

#include <cams/camera.h>

#include <map>
#include <string>
#include <vector>

class CameraArgumentParser
{
 public:
  CameraArgumentParser(const char *as);
  ~CameraArgumentParser();

  bool                                has(std::string s) const;
  std::string                         get(std::string s);
  std::map<std::string, std::string>  parameters() const;
  std::vector<std::string>            arguments() const;
  std::string                         camid() const;

 private:
  std::string _camid;
  std::map<std::string, std::string> values;
  std::vector<std::string> args;
};

#endif
