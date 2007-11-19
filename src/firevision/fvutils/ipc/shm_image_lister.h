
/***************************************************************************
 *  shm_image_lister.h - generates an array containing all the image
 *                       ids of the shm image buffers
 *
 *  Generated: Wed Oct 10 16:34:42 2007
 *  Copyright  2007  Daniel Beck
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

#include <utils/ipc/shm_lister.h>

class ShmImageLister : public SharedMemoryLister
{
 public:
  ShmImageLister(unsigned int max_buffers = 30);
  virtual ~ShmImageLister();

  virtual void printHeader();
  virtual void printFooter();
  virtual void printNoSegments();
  virtual void printNoOrphanedSegments();
  virtual void printInfo( SharedMemoryHeader* header,
			  int shm_id, int semaphore,
			  unsigned int mem_size,
			  void* mem_ptr );

  unsigned int num_buffers() const;
  char** image_ids() const;

 private:
  unsigned int m_max_buffers;
  unsigned int m_num_buffers;
  char** m_image_ids;
};
