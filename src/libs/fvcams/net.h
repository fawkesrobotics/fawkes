
/***************************************************************************
 *  net.h - This header defines an fuse network client camera
 *
 *  Created: Wed Feb 01 12:22:06 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __FIREVISION_CAMS_NET_H_
#define __FIREVISION_CAMS_NET_H_

#include <fvcams/camera.h>
#include <fvutils/net/fuse_client_handler.h>
#include <vector>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;
class FuseClient;
class FuseImageContent;
class FuseNetworkMessage;
class JpegImageDecompressor;

class NetworkCamera : public Camera, public FuseClientHandler
{

 public:
  NetworkCamera(const char *host, unsigned short port, bool jpeg = false);
  NetworkCamera(const char *host, unsigned short port, const char *image_id,
		bool jpeg = false);
  NetworkCamera(const CameraArgumentParser *cap);

  virtual ~NetworkCamera();

  virtual void open();
  virtual void start();
  virtual void stop();
  virtual void close();
  virtual void flush();
  virtual void capture();
  virtual void print_info();
  virtual bool ready();

  virtual unsigned char* buffer();
  virtual unsigned int   buffer_size();
  virtual void           dispose_buffer();

  virtual unsigned int   pixel_width();
  virtual unsigned int   pixel_height();
  virtual colorspace_t   colorspace();

  virtual void           set_image_id(const char *image_id);
  virtual void           set_image_number(unsigned int n);

  virtual fawkes::Time * capture_time();

  virtual std::vector<FUSE_imageinfo_t>& image_list();

  virtual void fuse_invalid_server_version(uint32_t local_version,
					   uint32_t remote_version) throw();
  virtual void fuse_connection_established() throw();
  virtual void fuse_connection_died() throw();
  virtual void fuse_inbound_received(FuseNetworkMessage *m) throw();

 private:
  bool __started;
  bool __opened;

  bool         __connected;
  unsigned int __local_version;
  unsigned int __remote_version;

  char               *__host;
  unsigned short      __port;
  char               *__image_id;

  bool __get_jpeg;
  JpegImageDecompressor *__decompressor;
  unsigned char      *__decompressed_buffer;
  unsigned int        __last_width;
  unsigned int        __last_height;

  FuseClient         *__fusec;
  FuseImageContent   *__fuse_image;
  FuseNetworkMessage *__fuse_message;

  FUSE_imageinfo_t   *__fuse_imageinfo;

  std::vector<FUSE_imageinfo_t> __image_list;
};

} // end namespace firevision

#endif
