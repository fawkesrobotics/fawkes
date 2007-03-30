
/***************************************************************************
 *  v4l.cpp - Implementation to access V4L cam
 *
 *  Generated: Fri Mar 11 17:48:27 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include <cams/v4l.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/rgb.h>

#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>	/* gettimeofday() */
#include <fcntl.h>
#include <unistd.h>
#include <linux/types.h>
#include <errno.h>
#include <cstring>
#include <iostream>

using namespace std;


/** @class V4LCamera <cams/v4l.h>
 * Video4Linux camera implementation.
 */


/** Constructor.
 * @param device_name device file name (e.g. /dev/video0)
 */
V4LCamera::V4LCamera(const char *device_name)
{
  started = opened = false;
  this->device_name = device_name;
}


/** Destructor. */
V4LCamera::~V4LCamera()
{
}


void
V4LCamera::open()
{
  opened = false;

  dev = ::open(device_name, O_RDWR);
  if (dev < 0) {
    throw Exception("V4LCam: Could not open device");
  }

  // getting grabber info in capabilities struct
  if ( (ioctl(dev, VIDIOCGCAP, &capabilities)) == -1 ) {
    throw Exception("V4LCam: Could not get capabilities");
  }

  // Capture window information
  if ( (ioctl(dev, VIDIOCGWIN, &window)) == -1) {
    throw Exception("V4LCam: Could not get window information");
  }

  // Picture information
  if ( (ioctl(dev, VIDIOCGPICT, &picture)) == -1) {
    throw Exception("V4LCam: Could not get window information");
  }

  ///Video Channel Information or Video Sources
  ///Allocate space for each channel
  channel = (struct video_channel*)malloc(sizeof(struct video_channel)*(capabilities.channels+1));
  for(int ch = 0; ch < capabilities.channels; ch++) {
    channel[ch].norm = 0;
    if ( (ioctl(dev, VIDIOCSCHAN, &channel[ch])) == -1) {
	printf("V4LCam: Could not get channel information for channel %i: %s", ch, strerror(errno));
    }
  }

  ///Trying to capture through read()
  if (ioctl (dev, VIDIOCGMBUF, captured_frame_buffer) == -1) {
    capture_method = READ;
    frame_buffer = (unsigned char *)malloc(window.width * window.height * RGB_PIXEL_SIZE);
  } else {
    capture_method = MMAP;
    frame_buffer = (unsigned char*)mmap (0, captured_frame_buffer.size, PROT_READ | PROT_WRITE, MAP_SHARED, dev, 0);
    if ((unsigned char *) -1 == (unsigned char *)frame_buffer) {
      throw Exception("V4LCam: Cannot initialize mmap region");
    }
  }

  buf_v4l = NULL;

  opened = true;
}


void
V4LCamera::start()
{

  started = false;
  if (!opened) {
    throw Exception("V4LCam: Trying to start closed cam!");
  }

  started = true;
}

void
V4LCamera::print_info()
{

  if (! opened) return;

  cout << endl << "CAPABILITIES" << endl
       << "===========================================================================" << endl;

  if(capabilities.type & VID_TYPE_CAPTURE)
    cout << " + Can capture to memory" << endl;
  if(capabilities.type & VID_TYPE_TUNER)
    cout << " + Has a tuner of some form" << endl;
  if(capabilities.type & VID_TYPE_TELETEXT)
    cout << " + Has teletext capability" << endl;
  if(capabilities.type & VID_TYPE_OVERLAY)
    cout << " + Can overlay its image onto the frame buffer" << endl;
  if(capabilities.type & VID_TYPE_CHROMAKEY)
    cout << " + Overlay is Chromakeyed" << endl;
  if(capabilities.type & VID_TYPE_CLIPPING)
    cout << " + Overlay clipping is supported" << endl;
  if(capabilities.type & VID_TYPE_FRAMERAM)
    cout << " + Overlay overwrites frame buffer memory" << endl;
  if(capabilities.type & VID_TYPE_SCALES)
    cout << " + The hardware supports image scaling" << endl;
  if(capabilities.type & VID_TYPE_MONOCHROME)
    cout << " + Image capture is grey scale only" << endl;
  if(capabilities.type & VID_TYPE_SUBCAPTURE)
    cout << " + Can subcapture" << endl;

  cout << endl;
  cout << " Number of Channels ='" << capabilities.channels << "'" << endl;
  cout << " Number of Audio Devices ='" << capabilities.audios << "'" << endl;
  cout << " Maximum Capture Width ='" << capabilities.maxwidth << "'" << endl;
  cout << " Maximum Capture Height ='" << capabilities.maxheight << "'" << endl;
  cout << " Minimum Capture Width ='" << capabilities.minwidth << "'" << endl;
  cout << " Minimum Capture Height ='" << capabilities.minheight << "'" << endl;




  cout << endl << "CAPTURE WINDOW INFO" << endl
       << "===========================================================================" << endl;

  cout << " X Coord in X window Format:  " << window.x << endl;
  cout << " Y Coord in X window Format:  " << window.y << endl;
  cout << " Width of the Image Capture:  " << window.width << endl;
  cout << " Height of the Image Capture: " << window.height << endl;
  cout << " ChromaKey:                   " << window.chromakey  << endl;




  cout << endl << "DEVICE PICTURE INFO" << endl
       << "===========================================================================" << endl;

  cout << " Picture Brightness: " << picture.brightness << endl;
  cout << " Picture        Hue: " << picture.hue << endl;
  cout << " Picture     Colour: " << picture.colour << endl;
  cout << " Picture   Contrast: " << picture.contrast << endl;
  cout << " Picture  Whiteness: " << picture.whiteness << endl;
  cout << " Picture      Depth: " << picture.depth << endl;
  cout << " Picture    Palette: " << picture.palette << " (";

  if(picture.palette == VIDEO_PALETTE_GREY)
    cout << "VIDEO_PALETTE_GRAY";
  if(picture.palette == VIDEO_PALETTE_HI240)
    cout << "VIDEO_PALETTE_HI240";
  if(picture.palette == VIDEO_PALETTE_RGB565)
    cout << "VIDEO_PALETTE_RGB565";
  if(picture.palette == VIDEO_PALETTE_RGB555)
    cout << "VIDEO_PALETTE_RGB555";
  if(picture.palette == VIDEO_PALETTE_RGB24)
    cout << "VIDEO_PALETTE_RGB24";
  if(picture.palette == VIDEO_PALETTE_RGB32)
    cout << "VIDEO_PALETTE_RGB32";
  if(picture.palette == VIDEO_PALETTE_YUV422)
    cout << "VIDEO_PALETTE_YUV422";
  if(picture.palette == VIDEO_PALETTE_YUYV)
    cout << "VIDEO_PALETTE_YUYV";
  if(picture.palette == VIDEO_PALETTE_UYVY)
    cout << "VIDEO_PALETTE_UYVY";
  if(picture.palette == VIDEO_PALETTE_YUV420)
    cout << "VIDEO_PALETTE_YUV420";
  if(picture.palette == VIDEO_PALETTE_YUV411)
    cout << "VIDEO_PALETTE_YUV411";
  if(picture.palette == VIDEO_PALETTE_RAW)
    cout << "VIDEO_PALETTE_RAW";
  if(picture.palette == VIDEO_PALETTE_YUV422P)
    cout << "VIDEO_PALETTE_YUV422P";
  if(picture.palette == VIDEO_PALETTE_YUV411P)
    cout << "VIDEO_PALETTE_YUV411P";

  cout << ")" << endl;



  cout << endl << "VIDEO SOURCE INFO" << endl
       << "===========================================================================" << endl;

  cout << " Channel Number or Video Source Number: " << channel->channel << endl;
  cout << " Channel Name:                          " << channel->name << endl;
  cout << " Number of Tuners for this source:      " << channel->tuners << endl;
  cout << " Channel Norm:                          " << channel->norm << endl;
  if(channel->flags & VIDEO_VC_TUNER)
    cout << " + This channel source has tuners" << endl;
  if(channel->flags & VIDEO_VC_AUDIO)
    cout << " + This channel source has audio" << endl;
  if(channel->type & VIDEO_TYPE_TV)
    cout << " + This channel source is a TV input" << endl;
  if(channel->type & VIDEO_TYPE_CAMERA)
    cout << " + This channel source is a Camera input" << endl;




  cout << endl << "FRAME BUFFER INFO" << endl
       << "===========================================================================" << endl;

  cout << " Base Physical Address:  " << vbuffer.base << endl;
  cout << " Height of Frame Buffer: " << vbuffer.height << endl;
  cout << " Width of Frame Buffer:  " << vbuffer.width << endl;
  cout << " Depth of Frame Buffer:  " << vbuffer.depth << endl;
  cout << " Bytes Per Line:         " << vbuffer.bytesperline << endl;



  /* Which channel!?
  cout << endl << "CHANNEL INFO" << endl
       << "===========================================================================" << endl;

  cout << " Channel:          " << ch << " - " << channel[ch].name << endl;
  cout << " Number of Tuners: " << channel[0].tuners << endl;
  cout << " Input Type:       " << channel[ch].type << endl;
  cout << " Flags: " << endl;
  if(channel[0].flags & VIDEO_VC_TUNER)
    cout << " + This Channel Source has Tuners" << endl;
  if(channel[0].flags & VIDEO_VC_AUDIO)
    cout << " + This Channel Source has Audio" << endl;
  //  if(channel[0].flags & VIDEO_VC_NORM)
  //cout << " \tThis Channel Source has Norm\n");
  cout << " Norm for Channel: '" << channel[0].norm << "'" << endl;
  */

}


void
V4LCamera::capture()
{

  if (capture_method == READ) {
    int len = read(dev, frame_buffer, window.width * window.height * RGB_PIXEL_SIZE);
    if (len < 0) {
      throw Exception("V4LCam: Could not capture frame");
    }
  } else {

    buf_v4l = (struct video_mmap*)malloc(captured_frame_buffer.frames * sizeof(struct video_mmap));

    ///Setting up the palette, size of frame and which frame to capture
    buf_v4l[0].format = picture.palette;
    buf_v4l[0].frame  = 0;
    buf_v4l[0].width  = window.width;
    buf_v4l[0].height = window.height;

    if (ioctl (dev, VIDIOCMCAPTURE, &(buf_v4l[0])) == -1) {
      throw Exception("V4LCam: Could not capture frame (VIDIOCMCAPTURE)");
    }
    ///Waiting for the frame to finish
    int Frame = 0;
    if (ioctl (dev, VIDIOCSYNC, &Frame) == -1) {
      throw Exception("V4LCam: Could not capture frame (VIDIOCSYNC)");
    }
  }
}


void
V4LCamera::dispose_buffer()
{
  if (capture_method == MMAP) {
    if (buf_v4l != NULL) {
      free(buf_v4l);
      buf_v4l = NULL;
    }
    munmap(frame_buffer, captured_frame_buffer.size);
  }
}


unsigned char*
V4LCamera::buffer()
{
  return frame_buffer;
}

unsigned int
V4LCamera::buffer_size()
{
  return colorspace_buffer_size(RGB, window.width, window.height);
}

void
V4LCamera::close()
{
  if (started) {
    ::close(dev);
  }
}

unsigned int
V4LCamera::pixel_width()
{
  if (opened) {
    return window.width;
  } else {
    throw Exception("V4LCam::getWidth(): Camera not opened");
  }
}

unsigned int
V4LCamera::pixel_height()
{
  if (opened) {
    return window.height;
  } else {
    throw Exception("V4LCam::getHeight(): Camera not opened");
  }
}


colorspace_t
V4LCamera::colorspace()
{
  return BGR;
}


void
V4LCamera::flush()
{
}


bool
V4LCamera::ready()
{
  return started;
}


void
V4LCamera::set_image_number(unsigned int n)
{
}
