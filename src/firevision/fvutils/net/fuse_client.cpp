
/***************************************************************************
 *  fuse_client.cpp - network image transport client interface
 *
 *  Generated: Thu Mar 29 00:47:24 2007
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

#include <fvutils/net/fuse_client.h>


/** @class FuseClient <fvutils/net/fuse_client.h>
 * FUSE client interface.
 * FUSE is the FireVision protocol to retrieve information, images and lookup
 * tables from vision processes and to send control commands to these systems.
 * The client is used in the retrieving or controlling process.
 * @author Tim Niemueller
 *
 * @fn bool FuseClient::connect()
 * Connect to the server.
 * Connect to the server, optionally non-blocking with timeout
 * @return returns true if the connection has been established, false otherwise
 *
 * @fn void FuseClient::recv()
 * Receive all data waiting in the queue.
 *
 * @fn void FuseClient::disconnect()
 * Disconnect from server.
 *
 * @fn bool FuseClient::connected()
 * Check if client is (still) connected.
 *
 * @fn void FuseClient::send()
 * Send data in queue.
 *
 * @fn bool FuseClient::setImageNumber(unsigned int num)
 * Set image number to retrieve next.
 * @param num image number
 *
 * @fn colorspace_t FuseClient::getImageColorspace() const
 * Get colorspace of last retrieved image.
 * @return colorspace
 *
 * @fn unsigned int FuseClient::getImageWidth() const
 * Get image width.
 * @return image width
 *
 * @fn unsigned int FuseClient::getImageHeight() const
 * Get image height.
 * @return image height
 *
 * @fn void FuseClient::getImageDimensions(unsigned int *w, unsigned int *h) const
 * Get image dimension.
 * @param w contains width upon return
 * @param h contains height upon return
 *
 * @fn unsigned char *  FuseClient::getImageBuffer()
 * Get image buffer of last retrieved image.
 * @return imae buffer.
 *
 * @fn void FuseClient::requestImage(bool request)
 * Set if image should be requested on next send().
 * @param request true to request, false otherwise
 *
 * @fn unsigned int     FuseClient::getImageNumber() const
 * Get image number of last retrieved image.
 * @return image number.
 *
 * @fn bool             FuseClient::imageAvailable() const
 * Check if image is available from last recv().
 * @return true if image is available, false otherwise.
 *
 * @fn bool             FuseClient::isImageApplAlive() const
 * Check if the application that provides the requested image is alive.
 * @return true if application is alive, false otherwise
 *
 * @fn std::string      FuseClient::getImageDeadAppName() const
 * Get name of dead application.
 * @return name of application that should have supplied the requested image
 *
 * @fn void             FuseClient::requestImageInfo(bool request = true)
 * Request image info.
 * @param request true to request image info, false otherwise
 *
 * @fn unsigned int     FuseClient::getRoiX() const
 * Get ROI X.
 * @return ROI X
 *
 * @fn unsigned int     FuseClient::getRoiY() const
 * Get ROI Y.
 * @return ROI Y
 *
 * @fn unsigned int     FuseClient::getRoiWidth() const
 * Get ROI width
 * @return ROI width
 *
 * @fn unsigned int     FuseClient::getRoiHeight() const
 * Get ROI height.
 * @return ROI height
 *
 * @fn int              FuseClient::getCircleX() const
 * Get circle X.
 * @return circle X
 *
 * @fn int              FuseClient::getCircleY() const
 * Get circle Y.
 * @return circle Y
 *
 * @fn unsigned int     FuseClient::getCircleRadius() const
 * Get circle radius.
 * @return circle radius
 *
 * @fn bool             FuseClient::getCircleFound() const
 * Check if circle was found.
 * @return true if circle found, false otherwise
 *
 * @fn bool             FuseClient::setLutID(unsigned int lut_id)
 * Set LUT ID.
 * @param lut_id LUT ID to request next.
 * @return true if succeeded, false otherwise
 *
 * @fn unsigned int     FuseClient::getLutWidth() const
 * Get LUT width.
 * @return LUT width
 *
 * @fn unsigned int     FuseClient::getLutHeight() const
 * Get LUT height.
 * @return LUT height
 *
 * @fn unsigned int     FuseClient::getLutBytesPerCell() const
 * Get LUT bytes per cell.
 * @return LUT bytes per cell
 *
 * @fn unsigned char *  FuseClient::getLutBuffer() const
 * Get LUT buffer.
 * @return LUT buffer
 *
 * @fn unsigned int     FuseClient::getLutBufferSize() const
 * Get LUT buffer size.
 * @return LUT buffer size
 *
 * @fn void             FuseClient::requestLut(bool request)
 * Request LUT.
 * @param request true to request, false otherwise
 *
 * @fn unsigned int     FuseClient::getLutID() const
 * Get LUT ID.
 * @return LUT ID
 *
 * @fn bool             FuseClient::lutAvailable() const
 * Check if LUT is available from last recv().
 * @return true if LUT is available, false otherwise
 *
 * @fn bool             FuseClient::isLutApplAlive() const
 * Check if the application that provides the requested LUT is alive.
 * @return true if application is alive, false otherwise
 *
 * @fn std::string      FuseClient::getLutDeadAppName() const
 * Get name of dead application.
 * @return name of application that should have supplied the requested LUT
 *
 * @fn void             FuseClient::setLutUpload(unsigned int lut_id, unsigned int width, unsigned int height, unsigned int bytes_per_cell,	unsigned char *buffer,unsigned int buffer_size)
 * Set LUT upload.
 * @param lut_id LUT ID
 * @param width LUT width
 * @param height LUT height
 * @param bytes_per_cell bytes per cell
 * @param buffer LUT buffer
 * @param buffer_size LUT buffer size
 *
 * @fn void             FuseClient::requestLutUpload(bool request = true)
 * Request LUT upload.
 * @param request true to request LUT upload, false otherwise
 *
 * @fn bool             FuseClient::lutUploadSuccess()
 * Check if LUT upload was a success.
 * @return true if LUT was uploaded successfully, false otherwise.
 *
 * @fn void             FuseClient::subscribeMessageQueue(unsigned int msgqid, long mtype, unsigned int data_size)
 * Subscribe to message queue.
 * @param msgqid message queue ID
 * @param mtype message type
 * @param data_size maximum data size
 *
 * @fn void             FuseClient::unsubscribeMessageQueue(unsigned int msgqid, long mtype)
 * Unsubscribe from message queue.
 * @param msgqid message queue ID
 * @param mtype message type
 *
 * @fn void             FuseClient::sendMessage(unsigned int msgqid, char *msg, unsigned int msg_size)
 * Send message.
 * @param msgqid message queue ID
 * @param msg message data
 * @param msg_size message data size
 *
 * @fn bool             FuseClient::isMessageAvailable()
 * Check if message is availabe.
 * @return true if message is available, false otherwise
 *
 * @fn void             FuseClient::getMessage(unsigned int *msgqid, char **msg, unsigned int *msg_size)
 * Get message.
 * @param msgqid queue to retrieve message from
 * @param msg contains message data upon return
 * @param msg_size contains message size upon return
 *
 * @fn void             FuseClient::freeMessage()
 * This frees a message from the queue.
 * The memory reserved for this message will be freed.
 *
 * @fn void             FuseClient::dropMessage()
 * Drop message from queue front.
 * The message currently in the top position of the queue will be dropped. The memory
 * reserved for this message will NOT be freed. You have to call free() on the msg
 * by yourself later.
 */

/** Empty virtual destructor. */
FuseClient::~FuseClient()
{
}
