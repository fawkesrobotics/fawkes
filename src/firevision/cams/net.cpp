
/***************************************************************************
 *  net.cpp - Camera to access images over the network
 *
 *  Created: Wed Feb 01 12:24:04 2006
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

#include <cams/net.h>
#include <cams/cam_exceptions.h>

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <fvutils/net/fuse_client.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_image_content.h>
#include <fvutils/system/camargp.h>

#include <cstring>

/** @class NetworkCamera <cams/net.h>
 * Network camera.
 * Retrieve images via network (FUSE).
 * @see FuseClient
 * @author Tim Niemueller
 */

/** Constructor.
 * @param host host to connect to
 * @param port port to connect to
 * @param image_id image ID of image to retrieve
 */
NetworkCamera::NetworkCamera(const char *host, unsigned short port, const char *image_id)
{
  if ( image_id == NULL ) {
    throw NullPointerException("NetworkCamera: image_id must not be NULL");
  }
  if ( host == NULL ) {
    throw NullPointerException("NetworkCamera: host must not be NULL");
  }
  __image_id = strdup(image_id);
  __host = strdup(host);
  __port = port;

  __connected       = false;
  __local_version   = 0;
  __remote_version  = 0;

  __fusec = new FuseClient(__host, __port, this);
}


/** Constructor.
 * Initialize with parameters from camera argument parser, supported values are:
 * - host=HOST, hostname or IP of host to connect to
 * - port=PORT, port number to connect to
 * - image=ID, image ID of image to retrieve
 * @param cap camera argument parser
 */
NetworkCamera::NetworkCamera(const CameraArgumentParser *cap)
{
  if ( cap->has("image") ) {
    __image_id = strdup(cap->get("image").c_str());
  } else {
    throw NullPointerException("image parameter must be set");
  }
  if ( cap->has("host") ) {
    __host = strdup(cap->get("host").c_str());
  } else {
    __host = strdup("localhost");
  }
  if ( cap->has("port") ) {
    int i = atoi(cap->get("port").c_str());
    if ( (i < 0) || (i >= 0xFFFF) ) {
      throw IllegalArgumentException("Port must be in the range 0-65535");
    }
    __port = (unsigned int)i;
  } else {
    __port = 5000;
  }

  __connected       = false;
  __local_version   = 0;
  __remote_version  = 0;

  __fusec = new FuseClient(__host, __port, this);
}


/** Destructor. */
NetworkCamera::~NetworkCamera()
{
  delete __fusec;
  free(__host);
  free(__image_id);
}


void
NetworkCamera::open()
{
  __fusec->connect();
  __fusec->start();
  __fusec->wait();
}


void
NetworkCamera::start()
{
  __started = true;
}

void
NetworkCamera::stop()
{
  __started = false;
}


void
NetworkCamera::print_info()
{
}


void
NetworkCamera::capture()
{
  if (! __connected) {
    throw CaptureException("Capture failed, not connected");
  }
  if ( __fuse_image ) {
    throw CaptureException("You must dispose the buffer before fetching a new image");
  }

  FUSE_imagedesc_message_t *idm = (FUSE_imagedesc_message_t *)malloc(sizeof(FUSE_imagedesc_message_t));
  memset(idm, 0, sizeof(FUSE_imagedesc_message_t));
  strncpy(idm->image_id, __image_id, IMAGE_ID_MAX_LENGTH);
  __fusec->enqueue(FUSE_MT_GET_IMAGE, idm, sizeof(FUSE_imagedesc_message_t));

  __fusec->wait();

  if ( ! __fuse_image ) {
    throw CaptureException("Fetching the image failed, no image received");
  }
}


unsigned char *
NetworkCamera::buffer()
{
  if ( __fuse_image ) {
    return __fuse_image->buffer();
  } else {
    return NULL;
  }
}

unsigned int
NetworkCamera::buffer_size()
{
  if (! __fuse_image) {
    return 0;
  }

  return colorspace_buffer_size((colorspace_t)__fuse_image->colorspace(),
				__fuse_image->pixel_width(),
				__fuse_image->pixel_height());
}

void
NetworkCamera::close()
{
  __fusec->disconnect();
  __fusec->cancel();
  __fusec->join();
  __opened = __started = false;
}

void
NetworkCamera::dispose_buffer()
{
  if ( __fuse_message ) {
    __fuse_message->unref();
    __fuse_message = NULL;
  }
  __fuse_image = NULL;
}

unsigned int
NetworkCamera::pixel_width()
{
  if ( __fuse_image ) {
    return __fuse_image->pixel_width();
  } else {
    return 0;
  }
}

unsigned int
NetworkCamera::pixel_height()
{
  if ( __fuse_image ) {
    return __fuse_image->pixel_height();
  } else {
    return 0;
  }
}


void
NetworkCamera::flush()
{
  if (! __connected)  return;
  dispose_buffer();
}


bool
NetworkCamera::ready()
{
  return __connected;
}


void
NetworkCamera::set_image_number(unsigned int n)
{
  // ignored, has to go away anyway
}


colorspace_t
NetworkCamera::colorspace()
{
  if ( __fuse_image ) {
    return (colorspace_t)__fuse_image->colorspace();
  } else {
    return CS_UNKNOWN;
  }
}


void
NetworkCamera::fuse_invalid_server_version(uint32_t local_version,
					   uint32_t remote_version) throw()
{
  __local_version  = local_version;
  __remote_version = remote_version;
}


void
NetworkCamera::fuse_connection_established() throw()
{
  __connected = true;
}


void
NetworkCamera::fuse_inbound_received(FuseNetworkMessage *m) throw()
{
  try {
    __fuse_image = m->msgc<FuseImageContent>();
    if ( __fuse_image ) {
      __fuse_message = m;
      __fuse_message->ref();
    }
  } catch (Exception &e) {
    __fuse_image = NULL;
    __fuse_message = NULL;
  }
}
