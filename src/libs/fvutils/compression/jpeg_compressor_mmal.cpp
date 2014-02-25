
/***************************************************************************
 *  jpeg_compressor_mmal.cpp - JPEG image compressor (using MMAL)
 *
 *  Created: Wed Feb 05 15:13:30 2014
 *  Copyright  2005-2014  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/compression/jpeg_compressor.h>
#include <fvutils/compression/jpeg_compressor_mmal.h>

#include <core/exception.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>

extern "C" {
#include <bcm_host.h>

#include <mmal/mmal.h>
#include <mmal/mmal_logging.h>
#include <mmal/mmal_buffer.h>
#include <mmal/util/mmal_util.h>
#include <mmal/util/mmal_util_params.h>
#include <mmal/util/mmal_default_components.h>
#include <mmal/util/mmal_connection.h>
}

#include <cstdio>
#include <cerrno>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

///@cond INTERNALS

class JpegImageCompressorMMAL::State {
 public:
  State() {
    frame_complete_ = false;
    frame_complete_mutex_ = new fawkes::Mutex();
    frame_complete_waitcond_ = new fawkes::WaitCondition(frame_complete_mutex_);

    compdest = ImageCompressor::COMP_DEST_MEM;
    file_handle = NULL;
    buffer = NULL;
    encoder_component = NULL;
    encoder_pool_in = NULL;
    encoder_pool_out = NULL;
    reset();
  }

  void reset() {
    jpeg_bytes = 0;
    buffer = jpeg_buffer;
  }

  CompressionDestination compdest;
  FILE *file_handle;
  char          *buffer;

  char          *jpeg_buffer;
  unsigned int   jpeg_buffer_size;
  unsigned int   jpeg_bytes;

  MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component

  MMAL_POOL_T *encoder_pool_in; /// Pointer to the pool of buffers used by encoder output port
  MMAL_POOL_T *encoder_pool_out; /// Pointer to the pool of buffers used by encoder input port

  bool frame_complete_;
  fawkes::Mutex         * frame_complete_mutex_;
  fawkes::WaitCondition * frame_complete_waitcond_;
};

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void
encoder_output_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  bool complete = false;

  // We pass our file handle and other stuff in via the userdata field.
  JpegImageCompressorMMAL::State *state =
    (JpegImageCompressorMMAL::State *)port->userdata;

  if (state) {
    size_t bytes_written = buffer->length;

    if (buffer->length) {
      mmal_buffer_header_mem_lock(buffer);
      if (state->compdest == ImageCompressor::COMP_DEST_FILE && state->file_handle) {
        bytes_written = fwrite(buffer->data, 1, buffer->length, state->file_handle);
      } else if (state->compdest == ImageCompressor::COMP_DEST_MEM && state->buffer) {
        if (state->jpeg_bytes + bytes_written <= state->jpeg_buffer_size) {
          memcpy(state->buffer, buffer->data, buffer->length);
          state->buffer += buffer->length;
          state->jpeg_bytes += buffer->length;
        } else {
	  printf("Buffer overflow: %zu + %zu > %zu\n", state->jpeg_bytes, bytes_written, state->jpeg_buffer_size);
	}
      }
      mmal_buffer_header_mem_unlock(buffer);
    }
    
    // We need to check we wrote what we wanted - it's possible we have run out of storage.
    if (bytes_written != buffer->length) {
      printf("Unable to write buffer to file - aborting");

      complete = true;
    }

    // Now flag if we have completed
    if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)) {
      complete = true;
    }
  }
  else
  {
    printf("Received a encoder buffer callback with no state");
  }

  // release buffer back to the pool
  mmal_buffer_header_release(buffer);

  // and send one back to the port (if still open)
  if (port->is_enabled) {
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_BUFFER_HEADER_T *new_buffer;

    new_buffer = mmal_queue_get(state->encoder_pool_out->queue);

    if (new_buffer) {
      status = mmal_port_send_buffer(port, new_buffer);
    }
    if (!new_buffer || status != MMAL_SUCCESS)
      printf("Unable to return a buffer to the encoder port");
  }

  if (complete) {
    state->frame_complete_mutex_->lock();
    state->frame_complete_ = true;
    state->frame_complete_waitcond_->wake_all();
    state->frame_complete_mutex_->unlock();
  }
}

static void encoder_input_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  // The decoder is done with the data, just recycle the buffer header into its pool
  mmal_buffer_header_release(buffer);
}


/// @endcond

/** @class JpegImageCompressorMMAL <fvutils/compression/jpeg_compressor.h>
 * Jpeg image compressor.
 * This JPEG image compressor implementation uses the MMAL hardware encoder
 * of the Raspberry Pi.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param quality JPEG quality in percent (1-100)
 */
JpegImageCompressorMMAL::JpegImageCompressorMMAL(unsigned int quality)
{
  vflip_ = false;
  width_ = height_ = 0;
  quality_ = quality;
  state_ = new State();
  // we can always do this, it'll just do nothing the second time
  bcm_host_init();
}

/** Destructor. */
JpegImageCompressorMMAL::~JpegImageCompressorMMAL()
{
  destroy_encoder_component();
  delete state_;
}


bool
JpegImageCompressorMMAL::supports_vflip()
{
  return true;
}


void
JpegImageCompressorMMAL::set_vflip(bool enable)
{
  vflip_ = enable;
}

void
JpegImageCompressorMMAL::compress()
{
  state_->reset();

  MMAL_PORT_T *encoder_input = NULL;
  MMAL_PORT_T *encoder_output = NULL;

  MMAL_STATUS_T status = MMAL_SUCCESS;

  //  Enable component
  if (mmal_component_enable(state_->encoder_component) != MMAL_SUCCESS) {
    mmal_component_destroy(state_->encoder_component);
    throw Exception("Unable to enable video encoder component");
  }

  encoder_input  = state_->encoder_component->input[0];
  encoder_output = state_->encoder_component->output[0];

  if (state_->compdest == ImageCompressor::COMP_DEST_FILE) {
    state_->file_handle = fopen(filename_, "wb");
    if (! state_->file_handle) {
      throw Exception(errno, "Failed to open output file");
    }
  }

  state_->frame_complete_mutex_->lock();
  state_->frame_complete_ = false;
  state_->frame_complete_mutex_->unlock();

  encoder_output->userdata = (::MMAL_PORT_USERDATA_T *)state_;

  // Enable the encoder output port and tell it its callback function
  status = mmal_port_enable(encoder_output, encoder_output_buffer_callback);

  // Send all the buffers to the encoder output port
  int num = mmal_queue_length(state_->encoder_pool_out->queue);

  for (int q = 0; q < num; ++q) {
    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state_->encoder_pool_out->queue);

    if (!buffer)
      printf("Unable to get a required buffer %d from pool queue", q);

    if (mmal_port_send_buffer(encoder_output, buffer)!= MMAL_SUCCESS)
      printf("Unable to send a buffer to encoder output port (%d)", q);
  }

  // Enable the encoder output port and tell it its callback function
  status = mmal_port_enable(encoder_input, encoder_input_buffer_callback);

  MMAL_BUFFER_HEADER_T *buffer;
  if ((buffer = mmal_queue_get(state_->encoder_pool_in->queue)) != NULL)
  {
    size_t exp_size = colorspace_buffer_size(YUV422_PLANAR,
					     encoder_input->format->es->video.width,
					     encoder_input->format->es->video.height);
    if (buffer->alloc_size < exp_size) {
      printf("Too small buffer");
    }

    buffer->cmd = 0;
    buffer->offset = 0;

    char *data = (char *)buffer->data;
    char *imgb = (char *)buffer_;

    unsigned int h;
    if (vflip_) {
      for (h = 0; h < encoder_input->format->es->video.height; ++h) {
	memcpy(data, imgb + ((height_ - h - 1) * width_), width_);
	//imgb += width_;
	data += encoder_input->format->es->video.width;
      }

      for (h = 0; h < encoder_input->format->es->video.height; ++h) {
	memcpy(data, imgb + (width_ * height_) + ((height_ - h - 1) * (width_/2)), width_ / 2);
	//imgb += width_ / 2;
	data += encoder_input->format->es->video.width / 2;
      }

      for (h = 0; h < encoder_input->format->es->video.height; ++h) {
	memcpy(data, imgb + (width_ * height_) + ((width_/2) * height_) + ((height_ - h - 1) * (width_/2)), width_ / 2);
	//memcpy(data, imgb, width_ / 2);
	//imgb += width_ / 2;
	data += encoder_input->format->es->video.width / 2;
      }
    } else {
      for (h = 0; h < encoder_input->format->es->video.height; ++h) {
	memcpy(data, imgb, width_);
	imgb += width_;
	data += encoder_input->format->es->video.width;
      }

      for (h = 0; h < encoder_input->format->es->video.height * 2; ++h) {
	memcpy(data, imgb, width_ / 2);
	imgb += width_ / 2;
	data += encoder_input->format->es->video.width / 2;
      }
    }

    buffer->length = (size_t)(data - (char *)buffer->data);
    buffer->flags  = MMAL_BUFFER_HEADER_FLAG_EOS;

    status = mmal_port_send_buffer(encoder_input, buffer);
    if (status != MMAL_SUCCESS) {
      printf("Unable to send input buffer: %x\n", status);
    }

    state_->frame_complete_mutex_->lock();
    while (! state_->frame_complete_) {
      state_->frame_complete_waitcond_->wait();
    }
    state_->frame_complete_mutex_->unlock();
  }

  if (encoder_input && encoder_input->is_enabled)
    mmal_port_disable(encoder_input);
  if (encoder_output && encoder_output->is_enabled)
    mmal_port_disable(encoder_output);

  if (state_->compdest == ImageCompressor::COMP_DEST_FILE) {
    fclose(state_->file_handle);
    state_->file_handle = NULL;
  }
}


void
JpegImageCompressorMMAL::set_image_dimensions(unsigned int width, unsigned int height)
{
  if (width_ != width || height_ != height) {
    width_  = width;
    height_ = height;
    destroy_encoder_component();
    create_encoder_component();
  }
}


void
JpegImageCompressorMMAL::set_image_buffer(colorspace_t cspace, unsigned char *buffer)
{
  if ( cspace == YUV422_PLANAR ) {
    buffer_ = buffer;
  } else {
    throw Exception("JpegImageCompressorMMAL: can only accept YUV422_PLANAR buffers");
  }
}


void
JpegImageCompressorMMAL::set_compression_destination(ImageCompressor::CompressionDestination cd)
{
  state_->compdest = cd;
}


bool
JpegImageCompressorMMAL::supports_compression_destination(ImageCompressor::CompressionDestination cd)
{
  return true;
}


void
JpegImageCompressorMMAL::set_destination_buffer(unsigned char *buf, unsigned int buf_size)
{
  state_->jpeg_buffer      = (char *)buf;
  state_->jpeg_buffer_size = buf_size;
}


size_t
JpegImageCompressorMMAL::compressed_size()
{
  return state_->jpeg_bytes;
}

size_t
JpegImageCompressorMMAL::recommended_compressed_buffer_size()
{
  return width_ * height_ * 2;
}


void
JpegImageCompressorMMAL::set_filename(const char *filename)
{
  filename_ = filename;
}

/** Create the encoder component, set up its ports */
void
JpegImageCompressorMMAL::create_encoder_component()
{
  MMAL_COMPONENT_T *encoder = 0;
  MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
  MMAL_STATUS_T status;
  MMAL_POOL_T *pool;

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

  if (status != MMAL_SUCCESS) {
    if (encoder)
      mmal_component_destroy(encoder);
    throw Exception("Unable to create JPEG encoder component");
  }

  if (!encoder->input_num || !encoder->output_num) {
    mmal_component_destroy(encoder);
    throw Exception("JPEG encoder doesn't have input/output ports");
  }

  encoder_input = encoder->input[0];
  encoder_output = encoder->output[0];

  memset(&encoder_input->format->es->video, 0, sizeof(MMAL_VIDEO_FORMAT_T));
  encoder_input->format->es->video.width  = width_;
  encoder_input->format->es->video.height = height_;
  encoder_input->format->es->video.crop.x = 0;
  encoder_input->format->es->video.crop.y = 0;
  encoder_input->format->es->video.crop.width = width_;
  encoder_input->format->es->video.crop.height = height_;
  encoder_input->format->es->video.frame_rate.num = 1;
  encoder_input->format->es->video.frame_rate.den = 1;

  // We want same format on input and output
  mmal_format_copy(encoder_output->format, encoder_input->format);

  // Specify input format
  encoder_input->format->flags  = 0;
  encoder_input->format->encoding  = MMAL_ENCODING_I422;

  // Specify out output format
  encoder_output->format->encoding = MMAL_ENCODING_JPEG;

  encoder_output->buffer_size = encoder_output->buffer_size_recommended * 2;
  if (encoder_output->buffer_size < encoder_output->buffer_size_min)
    encoder_output->buffer_size = encoder_output->buffer_size_min;

  // Set the JPEG quality level
  status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, quality_);
  if (status != MMAL_SUCCESS) {
    mmal_component_destroy(encoder);
    throw Exception("Unable to set JPEG quality");
  }

  status = mmal_port_parameter_set_boolean(encoder_output,
					   MMAL_PARAMETER_EXIF_DISABLE, 1);
  if (status != MMAL_SUCCESS) {
    mmal_component_destroy(encoder);
    throw Exception("Unable to disable JPEG EXIF data");
  }

  // Commit the port changes to the output port
  status = mmal_port_format_commit(encoder_output);

  if (status != MMAL_SUCCESS) {
    mmal_component_destroy(encoder);
    throw Exception("Unable to set format on video encoder output port");
  }

  // Commit the port changes to the input port
  status = mmal_port_format_commit(encoder_input);

  if (status != MMAL_SUCCESS) {
    mmal_component_destroy(encoder);
    throw Exception("Unable to set format on input encoder port");
  }

  {
    MMAL_PARAMETER_THUMBNAIL_CONFIG_T param_thumb =
      {{MMAL_PARAMETER_THUMBNAIL_CONFIGURATION,
	sizeof(MMAL_PARAMETER_THUMBNAIL_CONFIG_T)}, 0, 0, 0, 0};
    status = mmal_port_parameter_set(encoder->control, &param_thumb.hdr);
  }

  /* Create pool of buffer headers for the output port to consume */
  pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

  if (!pool) {
    mmal_component_destroy(encoder);
    throw Exception("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
  }

  state_->encoder_pool_out = pool;

  /* Create pool of buffer headers for the input port to consume */
  pool = mmal_port_pool_create(encoder_input, encoder_input->buffer_num, encoder_input->buffer_size);

  if (!pool) {
    mmal_component_destroy(encoder);
    throw Exception("Failed to create buffer header pool for encoder input port %s", encoder_input->name);
  }

  state_->encoder_pool_in = pool;
  state_->encoder_component = encoder;
}

/** Create the encoder component, set up its ports */
void
JpegImageCompressorMMAL::destroy_encoder_component()
{
  mmal_component_disable(state_->encoder_component);

  if (state_->encoder_pool_in) {
    mmal_port_pool_destroy(state_->encoder_component->input[0], state_->encoder_pool_in);
    state_->encoder_pool_in = NULL;
  }

  if (state_->encoder_pool_out) {
    mmal_port_pool_destroy(state_->encoder_component->output[0], state_->encoder_pool_out);
    state_->encoder_pool_out = NULL;
  }

  if (state_->encoder_component) {
    mmal_component_destroy(state_->encoder_component);
    state_->encoder_component = NULL;
  }
}

} // end namespace firevision
