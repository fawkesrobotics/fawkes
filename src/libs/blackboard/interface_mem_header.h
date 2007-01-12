 
/***************************************************************************
 *  interface_mem_header.h - BlackBoard interface memory header
 *
 *  Generated: Fri Oct 20 13:41:29 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_INTERFACE_MEM_HEADER_H_
#define __BLACKBOARD_INTERFACE_MEM_HEADER_H_

#define __INTERFACE_TYPE_SIZE 32
#define __INTERFACE_ID_SIZE 32

class RefCountRWLock;

/** This struct is used as header for interfaces in memory chunks.
 * This header is stored at the beginning of each allocated memory chunk.
 */
typedef struct {
  char             type[__INTERFACE_TYPE_SIZE];	/**< interface type */
  char             id[__INTERFACE_ID_SIZE];	/**< interface identifier */
  unsigned int     flag_writer_active :  1;	/**< true if there is any writer */
  unsigned int     flag_reserved      : 31;	/**< reserved for future use */
  unsigned int     refcount;			/**< reference count */
  unsigned int     serial;			/**< memory serial */
} interface_header_t;


#endif
