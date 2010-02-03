
/***************************************************************************
 *  mini_image.h - mini image supplier
 *
 *  Generated: Tue May 16 15:46:19 2006 (Automatica 2006)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_FOUNTAIN_MINI_IMAGE_H_
#define __FIREVISION_APPS_FOUNTAIN_MINI_IMAGE_H_

namespace fawkes {
  class Logger;
}
namespace firevision {
  class Scaler;
  class SharedMemoryImageBuffer;
}

class MiniImageProducer {
 public:

  MiniImageProducer(const char *orig_id, const char *mini_id,
		    firevision::Scaler *scaler, fawkes::Logger *logger);

  ~MiniImageProducer();

  bool isValid();
  void produce();

 private:
  fawkes::Logger *logger;
  firevision::Scaler *scaler;
  firevision::SharedMemoryImageBuffer *orig_shmem;
  firevision::SharedMemoryImageBuffer *mini_shmem;

  float scale_factor;

};


#endif
