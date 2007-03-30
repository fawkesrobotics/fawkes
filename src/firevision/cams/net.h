
/***************************************************************************
 *  net.h - This header defines an fuse network client camera
 *
 *  Generated: Wed Feb 01 12:22:06 2006
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

#ifndef __FIREVISION_CAMS_NET_H_
#define __FIREVISION_CAMS_NET_H_

#include <cams/camera.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/net/fuse_client_tcp.h>

class NetworkCamera : public Camera
{

 public:

  NetworkCamera(char *host, unsigned short port, unsigned int image_num=0, unsigned short proto = PROTOCOL_TCP);

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

  virtual void           set_image_number(unsigned int n);

  static const unsigned short PROTOCOL_TCP;

 private:
  bool started;
  bool opened;

  unsigned int    image_num;
  unsigned short  port;
  char           *host;

  FuseClientTCP  *fusec;

};

#endif
