
/***************************************************************************
 *  fuse_client.h - network image transport client interface
 *
 *  Generated: Mon Jan 09 15:33:59 2006
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_CLIENT_H_
#define __FIREVISION_FVUTILS_NET_FUSE_CLIENT_H_

#include <fvutils/color/colorspaces.h>

#include <string>

class FuseClient {
 public:
  virtual ~FuseClient();

  virtual bool             connect()                                                    = 0;
  virtual void             recv()                                                       = 0;
  virtual void             disconnect()                                                 = 0;
  virtual bool             connected()                                                  = 0;
  virtual void             send()                                                       = 0;

  virtual bool             setImageNumber(unsigned int num)                             = 0;
  virtual colorspace_t     getImageColorspace() const                                   = 0;
  virtual unsigned int     getImageWidth() const                                        = 0;
  virtual unsigned int     getImageHeight() const                                       = 0;
  virtual void             getImageDimensions(unsigned int *w, unsigned int *h) const   = 0;
  virtual unsigned char *  getImageBuffer()                                             = 0;
  virtual void             requestImage(bool request)                                   = 0;
  virtual unsigned int     getImageNumber() const                                       = 0;
  virtual bool             imageAvailable() const                                       = 0;
  virtual bool             isImageApplAlive() const                                     = 0;
  virtual std::string      getImageDeadAppName() const                                  = 0;
  virtual void             requestImageInfo(bool request = true)                        = 0;

  virtual unsigned int     getRoiX() const                                              = 0;
  virtual unsigned int     getRoiY() const                                              = 0;
  virtual unsigned int     getRoiWidth() const                                          = 0;
  virtual unsigned int     getRoiHeight() const                                         = 0;
  virtual int              getCircleX() const                                           = 0;
  virtual int              getCircleY() const                                           = 0;
  virtual unsigned int     getCircleRadius() const                                      = 0;
  virtual bool             getCircleFound() const                                       = 0;

  virtual bool             setLutID(unsigned int lut_id)                                = 0;
  virtual unsigned int     getLutWidth() const                                          = 0;
  virtual unsigned int     getLutHeight() const                                         = 0;
  virtual unsigned int     getLutBytesPerCell() const                                   = 0;
  virtual unsigned char *  getLutBuffer() const                                         = 0;
  virtual unsigned int     getLutBufferSize() const                                     = 0;
  virtual void             requestLut(bool request)                                     = 0;
  virtual unsigned int     getLutID() const                                             = 0;
  virtual bool             lutAvailable() const                                         = 0;
  virtual bool             isLutApplAlive() const                                       = 0;
  virtual std::string      getLutDeadAppName() const                                    = 0;

  virtual void             setLutUpload(unsigned int lut_id,
					unsigned int width, unsigned int height,
					unsigned int bytes_per_cell,
					unsigned char *buffer,
					unsigned int buffer_size)                       = 0;
  virtual void             requestLutUpload(bool request = true)                        = 0;
  virtual bool             lutUploadSuccess()                                           = 0;

  virtual void             subscribeMessageQueue(unsigned int msgqid, long mtype,
						 unsigned int data_size)                = 0;
  virtual void             unsubscribeMessageQueue(unsigned int msgqid, long mtype)     = 0;
  virtual void             sendMessage(unsigned int msgqid,
				       char *msg, unsigned int msg_size)                = 0;
  virtual bool             isMessageAvailable()                                         = 0;
  virtual void             getMessage(unsigned int *msgqid,
				      char **msg, unsigned int *msg_size)               = 0;

  virtual void             freeMessage()                                                = 0;
  virtual void             dropMessage()                                                = 0;

};


#endif
