
/***************************************************************************
 *  net.cpp - Camera to access images over the network
 *
 *  Created: Wed Feb 01 12:24:04 2006
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/net.h>
#include <fvcams/cam_exceptions.h>

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <fvutils/net/fuse_client.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_image_content.h>
#include <fvutils/net/fuse_imagelist_content.h>
#include <fvutils/system/camargp.h>
#include <fvutils/compression/jpeg_decompressor.h>

#include <netinet/in.h>
#include <cstdlib>
#include <cstring>

using namespace fawkes;

namespace firevision {

/** @class NetworkCamera <fvcams/net.h>
 * Network camera.
 * Retrieve images via network (FUSE).
 * @see FuseClient
 * @author Tim Niemueller
 */

/** Constructor.
 * Allows to initiate a NetworkCamera w/o specifying an image id. This can be
 * done later with the set_image_id() method.
 * @param host host to connect to
 * @param port port to connect to
 * @param jpeg if true jpeg images will be transferred and automatically be
 * decompressed, otherwise raw images are transferred
 */
NetworkCamera::NetworkCamera(const char *host, unsigned short port, bool jpeg)
{
  if ( host == NULL ) {
    throw NullPointerException("NetworkCamera: host must not be NULL");
  }
  image_id_ = 0;
  host_ = strdup(host);
  port_ = port;
  get_jpeg_ = jpeg;

  connected_       = false;
  opened_          = false;
  local_version_   = 0;
  remote_version_  = 0;
  decompressor_    = NULL;
  decompressed_buffer_ = NULL;
  last_width_ = 0;
  last_height_ = 0;
  fuse_image_ = NULL;
  fuse_message_ = NULL;
  fuse_imageinfo_ = NULL;

  fusec_ = new FuseClient(host_, port_, this);
  if ( get_jpeg_ ) {
    decompressor_ = new JpegImageDecompressor();
  }
}

/** Constructor.
 * @param host host to connect to
 * @param port port to connect to
 * @param image_id image ID of image to retrieve
 * @param jpeg if true jpeg images will be transferred and automatically be
 * decompressed, otherwise raw images are transferred
 */
NetworkCamera::NetworkCamera(const char *host, unsigned short port, const char *image_id,
			     bool jpeg)
{
  if ( image_id == NULL ) {
    throw NullPointerException("NetworkCamera: image_id must not be NULL");
  }
  if ( host == NULL ) {
    throw NullPointerException("NetworkCamera: host must not be NULL");
  }
  image_id_ = strdup(image_id);
  host_ = strdup(host);
  port_ = port;
  get_jpeg_ = jpeg;

  connected_       = false;
  opened_          = false;
  local_version_   = 0;
  remote_version_  = 0;
  decompressor_    = NULL;
  decompressed_buffer_ = NULL;
  last_width_ = 0;
  last_height_ = 0;
  fuse_image_ = NULL;
  fuse_message_ = NULL;
  fuse_imageinfo_ = NULL;

  fusec_ = new FuseClient(host_, port_, this);
  if ( get_jpeg_ ) {
    decompressor_ = new JpegImageDecompressor();
  }
}


/** Constructor.
 * Initialize with parameters from camera argument parser, supported values are:
 * - host=HOST, hostname or IP of host to connect to
 * - port=PORT, port number to connect to
 * - image=ID, image ID of image to retrieve
 * - jpeg=<true|false>, if true JPEGs are recieved and decompressed otherwise
 *   raw images will be transferred (raw is the default)
 * @param cap camera argument parser
 */
NetworkCamera::NetworkCamera(const CameraArgumentParser *cap)
{
  if ( cap->has("image") ) {
    image_id_ = strdup(cap->get("image").c_str());
  } else {
    throw NullPointerException("image parameter must be set");
  }
  if ( cap->has("host") ) {
    host_ = strdup(cap->get("host").c_str());
  } else {
    host_ = strdup("localhost");
  }
  if ( cap->has("port") ) {
    int i = atoi(cap->get("port").c_str());
    if ( (i < 0) || (i >= 0xFFFF) ) {
      throw IllegalArgumentException("Port must be in the range 0-65535");
    }
    port_ = (unsigned int)i;
  } else {
    port_ = 2208;
  }

  get_jpeg_ = ( cap->has("jpeg") && (cap->get("jpeg") == "true"));

  connected_       = false;
  opened_          = false;
  local_version_   = 0;
  remote_version_  = 0;
  decompressor_    = NULL;
  decompressed_buffer_ = NULL;
  last_width_ = 0;
  last_height_ = 0;
  fuse_image_ = NULL;
  fuse_message_ = NULL;
  fuse_imageinfo_ = NULL;

  fusec_ = new FuseClient(host_, port_, this);
  if ( get_jpeg_ ) {
    decompressor_ = new JpegImageDecompressor();
  }
}


/** Destructor. */
NetworkCamera::~NetworkCamera()
{
  close();
  delete fusec_;
  free(host_);
  free(image_id_);
  if ( decompressed_buffer_ != NULL) free(decompressed_buffer_);
  delete decompressor_;
}


void
NetworkCamera::open()
{
  if ( opened_ )  return;

  fusec_->connect();
  fusec_->start();
  fusec_->wait_greeting();

  if ( image_id_) {
    FUSE_imagedesc_message_t *imagedesc = (FUSE_imagedesc_message_t *)calloc(1, sizeof(FUSE_imagedesc_message_t));
    strncpy(imagedesc->image_id, image_id_, IMAGE_ID_MAX_LENGTH-1);
    fusec_->enqueue_and_wait(FUSE_MT_GET_IMAGE_INFO, imagedesc, sizeof(FUSE_imagedesc_message_t));

    if ( ! fuse_imageinfo_ ) {
      throw Exception("Could not receive image info. Image not available?");
    }
  }

  opened_ = true;
}


void
NetworkCamera::start()
{
  started_ = true;
}

void
NetworkCamera::stop()
{
  started_ = false;
}


void
NetworkCamera::print_info()
{
}


void
NetworkCamera::capture()
{
  if (! connected_) {
    throw CaptureException("Capture failed, not connected");
  }
  if ( fuse_image_ ) {
    throw CaptureException("You must dispose the buffer before fetching a new image");
  }
  if ( !image_id_ ) {
    throw CaptureException("You must specify an image id");
  }

  FUSE_imagereq_message_t *irm = (FUSE_imagereq_message_t *)malloc(sizeof(FUSE_imagereq_message_t));
  memset(irm, 0, sizeof(FUSE_imagereq_message_t));
  strncpy(irm->image_id, image_id_, IMAGE_ID_MAX_LENGTH-1);
  irm->format = (get_jpeg_ ? FUSE_IF_JPEG : FUSE_IF_RAW);
  fusec_->enqueue_and_wait(FUSE_MT_GET_IMAGE, irm, sizeof(FUSE_imagereq_message_t));

  if (! connected_) {
    throw CaptureException("Capture failed, connection died while waiting for image");
  }
  if ( ! fuse_image_ ) {
    throw CaptureException("Fetching the image failed, no image received");
  }

  if ( get_jpeg_ ) {
    if ( (fuse_image_->pixel_width() != last_width_) ||
	 (fuse_image_->pixel_height() != last_height_) ) {
      if (decompressed_buffer_ != NULL ) {
	free(decompressed_buffer_);
      }
      size_t buffer_size = colorspace_buffer_size(YUV422_PLANAR, fuse_image_->pixel_width(),
						  fuse_image_->pixel_height());
      decompressed_buffer_ = (unsigned char *)malloc(buffer_size);
      decompressor_->set_decompressed_buffer(decompressed_buffer_, buffer_size);
    }
    decompressor_->set_compressed_buffer(fuse_image_->buffer(), fuse_image_->buffer_size());
    decompressor_->decompress();
  }
}


unsigned char *
NetworkCamera::buffer()
{
  if (get_jpeg_) {
    return decompressed_buffer_;
  } else {
    if ( fuse_image_ ) {
      return fuse_image_->buffer();
    } else {
      return NULL;
    }
  }
}

unsigned int
NetworkCamera::buffer_size()
{
  if ( get_jpeg_ ) {
    return colorspace_buffer_size(YUV422_PLANAR, pixel_width(), pixel_height());
  } else {
    if (! fuse_image_) {
      return 0;
    } else {
      return colorspace_buffer_size((colorspace_t)fuse_image_->colorspace(),
				    fuse_image_->pixel_width(),
				    fuse_image_->pixel_height());
    }
  }
}

void
NetworkCamera::close()
{
  dispose_buffer();
  if ( started_ ) {
    stop();
  }
  if ( fuse_imageinfo_ ) {
    free(fuse_imageinfo_);
    fuse_imageinfo_ = NULL;
  }
  if ( opened_ ) {
    fusec_->disconnect();
    fusec_->cancel();
    fusec_->join();
    opened_ = false;
  }
}

void
NetworkCamera::dispose_buffer()
{
  delete fuse_image_;
  fuse_image_ = NULL;
  if ( fuse_message_ ) {
    fuse_message_->unref();
    fuse_message_ = NULL;
  }
}

unsigned int
NetworkCamera::pixel_width()
{
  if ( fuse_imageinfo_ ) {
    return ntohl(fuse_imageinfo_->width);
  } else {
    throw NullPointerException("No valid image info received");
  }
}

unsigned int
NetworkCamera::pixel_height()
{
  if ( fuse_imageinfo_ ) {
    return ntohl(fuse_imageinfo_->height);
  } else {
    throw NullPointerException("No valid image info received");
  }
}

fawkes::Time *
NetworkCamera::capture_time()
{
  if ( fuse_image_ ) {
    return fuse_image_->capture_time();
  } else {
    throw NullPointerException("No valid image exists");
  }  
}

void
NetworkCamera::flush()
{
  if (! connected_)  return;
  dispose_buffer();
}


bool
NetworkCamera::ready()
{
  return connected_;
}


/** Select the image that is opened.
 * @param image_id the image id
 */
void
NetworkCamera::set_image_id(const char *image_id)
{
  image_id_ = strdup(image_id);

  FUSE_imagedesc_message_t *imagedesc = (FUSE_imagedesc_message_t *)calloc(1, sizeof(FUSE_imagedesc_message_t));
  strncpy(imagedesc->image_id, image_id_, IMAGE_ID_MAX_LENGTH-1);
  fusec_->enqueue_and_wait(FUSE_MT_GET_IMAGE_INFO, imagedesc, sizeof(FUSE_imagedesc_message_t));

  if ( ! fuse_imageinfo_ ) {
    throw Exception("Could not received image info. Image not available?");
  }
}


void
NetworkCamera::set_image_number(unsigned int n)
{
  // ignored, has to go away anyway
}


colorspace_t
NetworkCamera::colorspace()
{
  if ( get_jpeg_ ) {
    return YUV422_PLANAR;
  } else {
    if ( fuse_imageinfo_ ) {
      return (colorspace_t)ntohs(fuse_imageinfo_->colorspace);
    } else {
      return CS_UNKNOWN;
    }
  }
}


/** List the available images.
 * @return a vector containing information about the available images
 */
std::vector<FUSE_imageinfo_t>&
NetworkCamera::image_list()
{
  image_list_.clear();

  if (! connected_) {
    throw CaptureException("Capture failed, not connected");
  }

  fusec_->enqueue_and_wait(FUSE_MT_GET_IMAGE_LIST);

  return image_list_;
}


void
NetworkCamera::fuse_invalid_server_version(uint32_t local_version,
					   uint32_t remote_version) throw()
{
  local_version_  = local_version;
  remote_version_ = remote_version;
}


void
NetworkCamera::fuse_connection_established() throw()
{
  connected_ = true;
}


void
NetworkCamera::fuse_connection_died() throw()
{
  connected_ = false;
}


void
NetworkCamera::fuse_inbound_received(FuseNetworkMessage *m) throw()
{
  switch(m->type()) {

  case FUSE_MT_IMAGE:
    try {
      fuse_image_ = m->msgc<FuseImageContent>();
      if ( fuse_image_ ) {
	fuse_message_ = m;
	fuse_message_->ref();
      }
    } catch (Exception &e) {
      fuse_image_ = NULL;
      fuse_message_ = NULL;
    }
    break;


  case FUSE_MT_IMAGE_INFO:
    try {
      fuse_imageinfo_ = m->msg_copy<FUSE_imageinfo_t>();
    } catch (Exception &e) {
      fuse_imageinfo_ = NULL;
    }
    break;

  case FUSE_MT_IMAGE_INFO_FAILED:
    fuse_imageinfo_ = NULL;
    break;

  case FUSE_MT_GET_IMAGE_FAILED:
    if ( fuse_message_ ) {
      fuse_message_->unref();
    }
    fuse_message_ = NULL;
    fuse_image_ = NULL;
    break;

  case FUSE_MT_IMAGE_LIST:
    try {
      FuseImageListContent* fuse_image_list = m->msgc<FuseImageListContent>();
      if (fuse_image_list ) {
	while ( fuse_image_list->has_next() ) {
	  FUSE_imageinfo_t *iip = fuse_image_list->next();
	  FUSE_imageinfo_t ii;
	  strncpy(ii.image_id, iip->image_id, IMAGE_ID_MAX_LENGTH);
	  ii.colorspace = ntohs(iip->colorspace);
	  ii.width = ntohl(iip->width);
	  ii.height = ntohl(iip->height);
	  ii.buffer_size = ntohl(iip->buffer_size);
	  image_list_.push_back(ii);
	}
      }
    }
    catch (Exception &e) {
    }
    break;

  default:
      break;
  }
}

} // end namespace firevision
