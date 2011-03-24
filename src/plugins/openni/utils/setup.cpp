
/***************************************************************************
 *  setup.cpp - OpenNI utility methods: setup routines
 *
 *  Created: Thu Mar 24 10:23:27 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <plugins/openni/utils/setup.h>
#include <config/config.h>

namespace fawkes {
  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

void
setup_map_generator(xn::MapGenerator &generator,
		    fawkes::Configuration *config)
{
  
  std::string  cfg_resolution = config->get_string("/plugins/openni/resolution");
  unsigned int cfg_fps  = config->get_uint("/plugins/openni/fps");

  XnResolution res = XN_RES_VGA;

  if (cfg_resolution == "QQVGA") {
    res = XN_RES_QQVGA;
  } else if (cfg_resolution == "CGA") {
    res = XN_RES_CGA;
  } else if (cfg_resolution == "QVGA") {
    res = XN_RES_QVGA;
  } else if (cfg_resolution == "VGA") {
    res = XN_RES_VGA;
  } else if (cfg_resolution == "SVGA") {
    res = XN_RES_SVGA;
  } else if (cfg_resolution == "XGA") {
    res = XN_RES_XGA;
  } else if (cfg_resolution == "720P") {
    res = XN_RES_720P;
  } else if (cfg_resolution == "SXGA") {
    res = XN_RES_SXGA;
  } else if (cfg_resolution == "UXGA") {
    res = XN_RES_UXGA;
  } else if (cfg_resolution == "1080P") {
    res = XN_RES_1080P;
  } else {
    throw Exception("setup_map_generator(): Unknown resolution '%s'",
		    cfg_resolution.c_str());
  }

  xn::Resolution resolution(res);

  XnMapOutputMode output_mode;
  output_mode.nXRes = resolution.GetXResolution();
  output_mode.nYRes = resolution.GetYResolution();
  output_mode.nFPS  = cfg_fps;
  XnStatus st;
  if ((st = generator.SetMapOutputMode(output_mode)) != XN_STATUS_OK) {
    throw Exception("OpenNI: failed to set map output mode: %s",
		    xnGetStatusString(st));
  }
}


} // end namespace fawkes::openni
} // end namespace fawkes
