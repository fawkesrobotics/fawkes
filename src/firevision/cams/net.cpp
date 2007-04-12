
/***************************************************************************
 *  net.cpp - Implementation to access over the network
 *
 *  Generated: Wed Feb 01 12:24:04 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <cams/net.h>

#include <fvutils/net/fuse_client_tcp.h>
#include <fvutils/system/camargp.h>

#include <cstring>

/** TCP for transmitting data. */
const unsigned short NetworkCamera::PROTOCOL_TCP = 1;

/** @class NetworkCamera <cams/net.h>
 * Network camera.
 * Retrieve images via network (FUSE).
 * @see FuseClient
 * @see FuseClientTCP
 */

/** Constructor.
 * @param host host to connect to
 * @param port port to connect to
 * @param image_num image number to retrieve
 * @param proto protocol to use, currently only TCP is supported
 */
NetworkCamera::NetworkCamera(char *host, unsigned short port, unsigned int image_num, unsigned short proto)
{
  this->image_num = image_num;
  this->port = port;
  this->host = strdup(host);

  switch (proto) {
  case PROTOCOL_TCP:
    fusec = new FuseClientTCP( host, port );
    break;
  default:
    throw Exception("Unsupported protocol");
  }
}


/** Constructor.
 * Initialize with parameters from camera argument parser, supported values are:
 * - host=<host>, hostname or IP of host to connect to
 * - port=<port>, port number to connect to
 * - image=<num>, image number of image to retrieve
 */
NetworkCamera::NetworkCamera(CameraArgumentParser *cap)
{
  image_num = 0;
  const char *host = "localhost";
  if ( cap->has("image") ) {
    int i = atoi(cap->get("image").c_str());
    image_num = (i < 0) ? 0 : (unsigned int)i;
  }
  if ( cap->has("host") ) {
    host = strdup(cap->get("host").c_str());
  }
  if ( cap->has("port") ) {
    int i = atoi(cap->get("port").c_str());
    if ( (i < 0) || (i >= 0xFFFF) ) {
      throw IllegalArgumentException("Port must be in the range 0-65535");
    }
    port = (unsigned int)i;
  }

  fusec = new FuseClientTCP(host, port);
}


NetworkCamera::~NetworkCamera()
{
  delete fusec;
  free(host);
}


void
NetworkCamera::open()
{
  if (fusec->connect()) {
    fusec->setImageNumber(image_num);
    fusec->requestImageInfo();
    fusec->recv();
  } else {
    throw Exception("Could not open FUSE client connection");
  }
}


void
NetworkCamera::start()
{
  started = true;
}

void
NetworkCamera::stop()
{
  started = false;
}


void
NetworkCamera::print_info()
{
}


void
NetworkCamera::capture()
{
  if (! fusec->connected()) {
    return;
  }

  fusec->requestImage();
  fusec->recv();
  if ( ! fusec->imageAvailable() ) {
    throw Exception("NetworkCamera: requested image not available");
  }
}


unsigned char *
NetworkCamera::buffer()
{
  if (! fusec->connected()) {
    return NULL;
  }

  if (fusec->imageAvailable()) {
    return fusec->getImageBuffer();
  } else {
    return NULL;
  }
}

unsigned int
NetworkCamera::buffer_size()
{
  if (! fusec->connected()) {
    return 0;
  }

  return colorspace_buffer_size(fusec->getImageColorspace(),
				fusec->getImageWidth(),
				fusec->getImageHeight() );
}

void
NetworkCamera::close()
{
  fusec->disconnect();
  opened = started = false;
}

void
NetworkCamera::dispose_buffer()
{
}

unsigned int
NetworkCamera::pixel_width()
{
  return fusec->getImageWidth();
}

unsigned int
NetworkCamera::pixel_height()
{
  return fusec->getImageHeight();
}


void
NetworkCamera::flush()
{
  if (! fusec->connected()) {
    return;
  }

  for (unsigned short i = 0; (i < 10) && (fusec->imageAvailable()); ++i) {
    fusec->recv();
  }
}


bool
NetworkCamera::ready()
{
  return fusec->connected();
}


unsigned int
NetworkCamera::number_of_images()
{
  return 0;
}


void
NetworkCamera::set_image_number(unsigned int n)
{
  if (! fusec->connected()) {
    throw Exception("Cannot set image number if not connected");
  }

  if (fusec->setImageNumber(n)) {
    this->image_num = n;
  } else {
    throw Exception("Could not set image number");
  }
}


colorspace_t
NetworkCamera::colorspace()
{
  if (! fusec->connected()) return CS_UNKNOWN;

  return fusec->getImageColorspace();
}
