
/***************************************************************************
 *  laserht_plugin.h - Fawkes Laser Hough Transform plugin
 *
 *  Created: Sat Jul 04 21:34:07 2009 (RoboCup 2009, Graz)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: laser_plugin.h 1486 2008-10-29 18:08:45Z tim $
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

#ifndef __PLUGINS_LASERHT_LASERHT_PLUGIN_H_
#define __PLUGINS_LASERHT_LASERHT_PLUGIN_H_

#include <core/plugin.h>

class LaserHoughTransformPlugin : public fawkes::Plugin
{
 public:
  LaserHoughTransformPlugin(fawkes::Configuration *config);
};

#endif

