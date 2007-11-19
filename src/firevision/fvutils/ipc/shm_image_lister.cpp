
/***************************************************************************
 *  shm_image_lister.cpp - generates an array containing all the image
 *                         ids of the shm image buffers
 *
 *  Generated: Wed Oct 10 16:43:07 2007
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

#include <fvutils/ipc/shm_image_lister.h>
#include <fvutils/ipc/shm_image.h>

#include <stdlib.h>
#include <string.h>

/** @class ShmImageLister <fvutils/ipc/shm_image_lister.h>
 * Lists the names of all shared memory image buffers.
 * @author Daniel Beck
 */


/** Constructor.
 * @param max_buffers the (initial) number of buffers
 */
ShmImageLister::ShmImageLister(unsigned int max_buffers)
{
  m_max_buffers = max_buffers;
  m_num_buffers = 0;
  m_image_ids = (char**) malloc( m_max_buffers * sizeof(char*) );
}

/** Destructor. */
ShmImageLister::~ShmImageLister()
{
  for (unsigned int i = 0; i < m_num_buffers; i++)
    {
      free(m_image_ids[i]);
    }

  free(m_image_ids);
}


/** Noop. */
void
ShmImageLister::printHeader()
{
}


/** Noop. */
void
ShmImageLister::printFooter()
{
}


/** Noop. */
void
ShmImageLister::printNoSegments()
{
}


/** Noop. */
void
ShmImageLister::printNoOrphanedSegments()
{
}


/** Copies the image id for every shared memor image buffer
 * into an array.
 * @param header The data-specific header
 * @param shm_id The id of the shared memory segment
 * @param semaphore Semaphore key of the given shared memory segment
 * @param mem_size the total memory size
 * @param memptr pointer to the data segment.
 */
void
ShmImageLister::printInfo( SharedMemoryHeader* header,
			   int shm_id, int semaphore,
			   unsigned int mem_size,
			   void* memptr )
{
  SharedMemoryImageBufferHeader* h = (SharedMemoryImageBufferHeader*) header;

  char* image_id = strdup( h->image_id() );
  m_image_ids[m_num_buffers] = image_id;
  ++m_num_buffers;

  if (m_num_buffers > m_max_buffers)
    {
      m_max_buffers += 5;

      char** tmp = (char**) malloc( m_max_buffers * sizeof(char*) );

      for (unsigned int i = 0; i < m_num_buffers; ++i)
	{
	  tmp[i] = m_image_ids[i];
	}

      free(m_image_ids);
      m_image_ids = tmp;
    }
}


/** Returns the number of found SHM image buffers.
 * @return the number of found SHM image buffers
 */
unsigned int
ShmImageLister::num_buffers() const
{
  return m_num_buffers;
}


/** Returns the ids of the found SHM image buffers.
 * @return char-array with the ids of the found SHM image buffers
 */
char**
ShmImageLister::image_ids() const
{
  return m_image_ids;
}
