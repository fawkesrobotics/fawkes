
/***************************************************************************
 *  transfer_thread.h - OpenNI Visualization: network transfer thread
 *
 *  Created: Sat Apr 02 20:00:50 2011
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

#include <core/threading/thread.h>

#include <map>
#include <string>

namespace fawkes {
  class ReadWriteLock;
}

namespace firevision {
  class Camera;
}

class PclViewerTransferThread : public fawkes::Thread
{
 public:
  PclViewerTransferThread();
  ~PclViewerTransferThread();

  void loop();

  void lock_for_read();
  void unlock();
  void add_camera(std::string name, firevision::Camera *cam);

  /** Get buffer for specified camera.
   * @param name the name passed to add_camera.
   * @return buffer that contains a copy of the image
   */
  const unsigned char *  buffer(std::string name)
  { return __buffers[name]; }

 private:
  std::map<std::string, firevision::Camera *> __cams;
  std::map<std::string, unsigned char *> __buffers;
  std::map<std::string, size_t> __buffer_sizes;
  fawkes::ReadWriteLock  *__rwlock;
};
