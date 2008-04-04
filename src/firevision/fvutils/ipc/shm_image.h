
/***************************************************************************
 *  shm.h - shared memory image buffer
 *
 *  Generated: Thu Jan 12 13:12:24 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FIREVISION_FVUTILS_IPC_SHM_IMAGE_H_
#define __FIREVISION_FVUTILS_IPC_SHM_IMAGE_H_

#include <utils/ipc/shm.h>
#include <utils/ipc/shm_lister.h>

#include <fvutils/ipc/defs.h>
#include <fvutils/color/colorspaces.h>


// Magic token to identify FireVision shared memory images
#define FIREVISION_SHM_IMAGE_MAGIC_TOKEN "FireVision Image"

// Not that there is a relation to ITPimage_packet_header_t
/** Shared memory header struct for FireVision images. */
typedef struct {
  char          image_id[IMAGE_ID_MAX_LENGTH];/**< image ID */
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
  SharedMemoryImageBufferHeader(const char *image_id,
				colorspace_t colorspace,
				unsigned int width,
				unsigned int height);
  SharedMemoryImageBufferHeader(const SharedMemoryImageBufferHeader *h);
  virtual ~SharedMemoryImageBufferHeader();

  virtual SharedMemoryHeader *  clone() const;
  virtual bool         matches(void *memptr);
  virtual size_t       size();
  virtual void         print_info();
  virtual bool         create();
  virtual void         initialize(void *memptr);
  virtual void         set(void *memptr);
  virtual void         reset();
  virtual size_t       data_size();
  virtual bool         operator==(const SharedMemoryHeader & s) const;

  void                 set_image_id(const char *image_id);
  colorspace_t         colorspace() const;
  unsigned int         width() const;
  unsigned int         height() const;
  const char *         image_id() const;

  SharedMemoryImageBuffer_header_t * raw_header();

 private:
  char          *_image_id;
  colorspace_t   _colorspace;
  unsigned int   _width;
  unsigned int   _height;

  char          *_orig_image_id;
  colorspace_t   _orig_colorspace;
  unsigned int   _orig_width;
  unsigned int   _orig_height;

  SharedMemoryImageBuffer_header_t *_header;
};

class SharedMemoryImageBufferLister : public SharedMemoryLister {
 public:
  SharedMemoryImageBufferLister();
  virtual ~SharedMemoryImageBufferLister();

  virtual void print_header();
  virtual void print_footer();
  virtual void print_no_segments();
  virtual void print_no_orphaned_segments();
  virtual void print_info(const SharedMemoryHeader *header,
			  int shm_id, int semaphore,
			  unsigned int mem_size,
			  const void *memptr);
};


class SharedMemoryImageBuffer : public SharedMemory
{

 public:
  SharedMemoryImageBuffer(const char *image_id,
			  colorspace_t cspace,
			  unsigned int width, unsigned int height);
  SharedMemoryImageBuffer(const char *image_id, bool is_read_only = true);
  ~SharedMemoryImageBuffer();

  const char *     image_id() const;
  unsigned char *  buffer() const;
  colorspace_t     colorspace() const;
  unsigned int     width() const;
  unsigned int     height() const;
  unsigned int     roi_x() const;
  unsigned int     roi_y() const;
  unsigned int     roi_width() const;
  unsigned int     roi_height() const;
  int              circle_x() const;
  int              circle_y() const;
  unsigned int     circle_radius() const;
  bool             circle_found() const;
  void             set_roi_x(unsigned int roi_x);
  void             set_roi_y(unsigned int roi_y);
  void             set_roi_width(unsigned int roi_w);
  void             set_roi_height(unsigned int roi_h);
  void             set_roi(unsigned int roi_x, unsigned int roi_y,
			   unsigned int roi_w, unsigned int roi_h);
  void             set_circle_x(int circle_x);
  void             set_circle_y(int circle_y);
  void             set_circle_radius(unsigned int circle_radius);
  void             set_circle(int x, int y, unsigned int r);
  void             set_circle_found(bool found);
  bool             set_image_id(const char *image_id);

  static void      list();
  static void      cleanup(bool use_lister = true);
  static bool      exists(const char *image_id);
  static void      wipe(const char *image_id);

 private:
  void constructor(const char *image_id, colorspace_t cspace,
		   unsigned int width, unsigned int height,
		   bool is_read_only);

  SharedMemoryImageBufferHeader    *priv_header;
  SharedMemoryImageBuffer_header_t *raw_header;

  char *         _image_id;
  colorspace_t   _colorspace;
  unsigned int   _width;
  unsigned int   _height;


};


#endif
