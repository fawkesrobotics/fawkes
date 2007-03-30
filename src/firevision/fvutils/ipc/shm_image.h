
/***************************************************************************
 *  shm.h - shared memory image buffer
 *
 *  Generated: Thu Jan 12 13:12:24 2006
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

#ifndef __FIREVISION_FVUTILS_IPC_SHM_IMAGE_H_
#define __FIREVISION_FVUTILS_IPC_SHM_IMAGE_H_

#include <utils/ipc/shm.h>
#include <utils/ipc/shm_lister.h>

#include <fvutils/color/colorspaces.h>

// Not that there is a relation to ITPimage_packet_header_t
/** Shared memory header struct for FireVision images. */
typedef struct {
  unsigned int  image_num;		/**< image number */
  unsigned int  colorspace;		/**< color space */
  unsigned int  width;			/**< width */
  unsigned int  height;			/**< height */
  unsigned int  roi_x;			/**< ROI start x */
  unsigned int  roi_y;			/**< ROI start y */
  unsigned int  roi_width;		/**< ROI width */
  unsigned int  roi_height;		/**< ROI height */
  // Circle relative to ROI
  int           circle_x;		/**< ROI circle center x */
  int           circle_y;		/**< ROI circle center y */
  unsigned int  circle_radius;		/**< ROI circle radius */
  unsigned int  flag_circle_found :  1;	/**< 1 if circle found */
  unsigned int  flag_image_ready  :  1;	/**< 1 if image ready */
  unsigned int  flag_reserved     : 30;	/**< reserved for future use */
} SharedMemoryImageBuffer_header_t;


class SharedMemoryImageBufferHeader : public SharedMemoryHeader {
 public:
  SharedMemoryImageBufferHeader();
  SharedMemoryImageBufferHeader(unsigned int image_num,
				colorspace_t colorspace,
				unsigned int width,
				unsigned int height);
  virtual ~SharedMemoryImageBufferHeader();

  virtual bool         matches(void *memptr);
  virtual unsigned int size();
  virtual void         print_info();
  virtual bool         create();
  virtual void         initialize(void *memptr);
  virtual void         set(void *memptr);
  virtual unsigned int dataSize();

  void                 setImageNumber(unsigned int image_num);
  colorspace_t         getColorspace();
  unsigned int         getWidth();
  unsigned int         getHeight();
  unsigned int         getImageNumber();

  SharedMemoryImageBuffer_header_t * getRawHeader();

 private:
  SharedMemoryImageBuffer_header_t *header;

  unsigned int   image_num;
  colorspace_t   colorspace;
  unsigned int   width;
  unsigned int   height;
};

class SharedMemoryImageBufferLister : public SharedMemoryLister {
 public:
  SharedMemoryImageBufferLister();
  virtual ~SharedMemoryImageBufferLister();

  virtual void printHeader();
  virtual void printFooter();
  virtual void printNoSegments();
  virtual void printNoOrphanedSegments();
  virtual void printInfo(SharedMemoryHeader *header,
			 int shm_id, int semaphore,
			 unsigned int mem_size,
			 void *memptr);
};


class SharedMemoryImageBuffer : public SharedMemory
{

 public:
  SharedMemoryImageBuffer(colorspace_t cspace,
			  unsigned int width, unsigned int height,
			  unsigned int image_num = 0);
  SharedMemoryImageBuffer(unsigned int image_num = 0, bool is_read_only = true);
  ~SharedMemoryImageBuffer();

  unsigned char *  getBuffer();
  colorspace_t     getColorspace();
  unsigned int     getWidth();
  unsigned int     getHeight();
  unsigned int     getROIX();
  unsigned int     getROIY();
  unsigned int     getROIWidth();
  unsigned int     getROIHeight();
  int              getCircleX();
  int              getCircleY();
  unsigned int     getCircleRadius();
  void             setROIX(unsigned int roi_x);
  void             setROIY(unsigned int roi_y);
  void             setROIWidth(unsigned int roi_w);
  void             setROIHeight(unsigned int roi_h);
  void             setROI(unsigned int roi_x, unsigned int roi_y,
			  unsigned int roi_w, unsigned int roi_h);
  void             setCircleX(int circle_x);
  void             setCircleY(int circle_y);
  void             setCircleRadius(unsigned int circle_radius);
  void             setCircle(int x, int y, unsigned int r);
  void             setCircleFound(bool found);
  bool             getCircleFound();
  bool             setImageNumber(unsigned int n);

  static void      list();
  static void      cleanup();
  static bool      exists(unsigned int image_num);
  static void      wipe(unsigned int image_num);

 private:
  void constructor(colorspace_t cspace,
		   unsigned int width, unsigned int height,
		   unsigned int image_num,
		   bool is_read_only);

  SharedMemoryImageBufferHeader    *priv_header;
  SharedMemoryImageBuffer_header_t *raw_header;

  unsigned int   image_num;
  colorspace_t   colorspace;
  unsigned int   width;
  unsigned int   height;


};


#endif
