
/***************************************************************************
 *  vision_master.h - Vision Master aspect for Fawkes
 *
 *  Created: Tue May 29 14:45:03 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __ASPECT_VISION_MASTER_H_
#define __ASPECT_VISION_MASTER_H_

class VisionMaster;

class VisionMasterAspect
{
 public:
  VisionMasterAspect(VisionMaster *master);
  virtual ~VisionMasterAspect();

  VisionMaster *  vision_master();

 private:
  VisionMaster *master;
};

#endif
