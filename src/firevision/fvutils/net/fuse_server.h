
/***************************************************************************
 *  fuse_server.h - network image transport server interface
 *
 *  Generated: Mon Jan 09 15:26:27 2006
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_SERVER_H_
#define __FIREVISION_FVUTILS_NET_FUSE_SERVER_H_

#include <fvutils/color/colorspaces.h>
#include <vector>
#include <map>
#include <utility>
#include <string>

class FuseServer {
 public:

  /** Copy mode for images. */
  enum FuseServer_copy_mode_t {
    DEEP_COPY   = 1,	/**< deep copy, creates a real copy of the memory. */
    SHALLOW_COPY = 2	/**< shallow copy, just copies the pointer to the data, not
			 * the data itself, */
  };

  virtual ~FuseServer();

  virtual void          bind()                                                  = 0;
  virtual bool          accept()                                                = 0;
  virtual void          close()                                                 = 0;
  virtual void          send()                                                  = 0;
  virtual bool          connected()                                             = 0;
  virtual void          disconnect(unsigned int client = 0xFFFFFFFF)                                            = 0;
  virtual void          process()                                               = 0;
  virtual void          setImageCopyMode(FuseServer_copy_mode_t mode)           = 0;


  // Image related
  virtual void          setImageColorspace(unsigned int image_num,
					   colorspace_t cspace)                 = 0;
  virtual void          setImageWidth(unsigned int image_num,
				      unsigned int width)                       = 0;
  virtual void          setImageHeight(unsigned int image_num,
				       unsigned int height)                     = 0;
  virtual void          setImageDimensions(unsigned int image_num,
					   unsigned int w, unsigned int h)      = 0;
  virtual void          setImageROI(unsigned int image_num,
				    unsigned int x, unsigned int y,
				    unsigned int w, unsigned int h)             = 0;
  virtual void          setImageCircle(unsigned int image_num,
				       int x, int y, unsigned int r)            = 0;
  virtual void          setImageCircleFound(unsigned int image_num,
					    bool found)                         = 0;
  virtual void          setImageBuffer(unsigned int image_num,
				       unsigned char *buffer)                   = 0;
  virtual void          setImageAvailable(unsigned int image_num,
					  bool available)                       = 0;
  virtual void          setImageAppAlive(unsigned int image_num, bool alive,
					 std::string appl)                      = 0;
  virtual std::vector<unsigned int>  getImageNumbers()                          = 0;
  virtual std::vector<unsigned int>  getRequestedImageNumbers()                 = 0;

  // for LUT download (server to client)
  virtual void          setLUT(unsigned int lut_id,
			       unsigned int width, unsigned int height,
			       unsigned int bytes_per_cell,
			       unsigned char *data)                             = 0;
  virtual std::vector<unsigned int> getLutIDs()                                 = 0;
  virtual std::vector<unsigned int> getRequestedLutIDs()                        = 0;
  virtual void          setLutAvailable(unsigned int lut_id, bool available)    = 0;
  virtual void          setLutAppAlive(unsigned int lut_id, bool alive,
				       std::string appl)                        = 0;

  // for LUT upload (client to server)
  virtual void            setUploadLutAppAlive(unsigned int lut_id, bool alive,
					       std::string appl)                = 0;
  virtual std::vector<unsigned int> getUploadedLutIDs()                         = 0;
  virtual unsigned char * getUploadedLutBuffer(unsigned int lut_id)             = 0;
  virtual unsigned int    getUploadedLutBufferSize(unsigned int lut_id)         = 0;
  virtual unsigned int    getUploadedLutWidth(unsigned int lut_id)              = 0;
  virtual unsigned int    getUploadedLutHeight(unsigned int lut_id)             = 0;
  virtual unsigned int    getUploadedLutBytesPerCell(unsigned int lut_id)       = 0;
  virtual void            setUploadLutSuccess(unsigned int lut_id,
					      bool success)                     = 0;
  virtual void            removeMessageQueueSubscriptions( unsigned int msgqid )= 0;

  // messaging stuff (client to server)
  virtual std::map< unsigned int, std::vector< std::pair< long, unsigned int > > >
    getMessageSubscriptions()                                                    = 0;
  virtual bool            isMessageAvailable()                                  = 0;
  virtual void            getMessage(unsigned int *msgqid,
				     char **msg, unsigned int *msg_size)        = 0;
  virtual void            freeMessage()                                         = 0;
  virtual void            sendMessage(unsigned int msgqid, long mtype,
				      char *msg, unsigned int msg_size)         = 0;

};


#endif
