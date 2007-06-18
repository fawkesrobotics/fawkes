
/***************************************************************************
 *  mini_image.h - mini image supplier
 *
 *  Generated: Tue May 16 15:46:19 2006 (Automatica 2006)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_FOUNTAIN_MINI_IMAGE_H_
#define __FIREVISION_APPS_FOUNTAIN_MINI_IMAGE_H_

class Logger;
class Scaler;
class SharedMemoryImageBuffer;

class MiniImageProducer {
 public:

  MiniImageProducer(const char *orig_id, const char *mini_id,
		    Scaler *scaler, Logger *logger);

  ~MiniImageProducer();

  bool isValid();
  void produce();

 private:
  Logger *logger;
  Scaler *scaler;
  SharedMemoryImageBuffer *orig_shmem;
  SharedMemoryImageBuffer *mini_shmem;

  float scale_factor;

};


#endif
