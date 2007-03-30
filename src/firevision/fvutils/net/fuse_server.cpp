
/***************************************************************************
 *  fuse_server.tcp - network image transport server interface
 *
 *  Generated: Mon Mar 19 15:56:22 2007
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

#include <fvutils/net/fuse_server.h>

/** @class FuseServer <fvutils/net/fuse_server.h>
 * FireVision FUSE protocol server interface.
 * This interfaces defines all interactions between a data providing
 * application and a FUSE server.
 *
 * @author Tim Niemueller
 *
 * @fn void FuseServer::bind()
 * Bind server port if necessary.
 *
 * @fn bool FuseServer::accept()
 * Accept connection.
 * @return true, if a connetion has been accepted, false otherwise
 *
 * @fn void FuseServer::close()
 * Close down the server.
 *
 * @fn void FuseServer::send()
 * Send all enqueued outgoing data.
 *
 * @fn bool FuseServer::connected()
 * Check if a client is connected.
 * @return true, if at least one client is connected, false otherwise.
 *
 * @fn void FuseServer::disconnect(unsigned int client = 0xFFFFFFFF)
 * Disconnect one or all clients.
 * @param client ID of the client to disconnect, if client is 0xFFFFFFFF then all
 * connected clients are disconnected.
 *
 * @fn void FuseServer::process()
 * Process all incoming queues.
 *
 * @fn void FuseServer::setImageCopyMode(FuseServer_copy_mode_t mode)
 * Set the image copy mode.
 * @param mode new copy mode.
 *
 * @fn void FuseServer::setImageColorspace(unsigned int image_num, colorspace_t cspace)
 * Set colorspace of outgoing image.
 * @param image_num image number of image
 * @param cspace color space of image
 *
 * @fn void FuseServer::setImageWidth(unsigned int image_num, unsigned int width)
 * Set image width.
 * @param image_num image number of image
 * @param width width of image
 *
 * @fn void FuseServer::setImageHeight(unsigned int image_num, unsigned int height)
 * Set image width.
 * @param image_num image number of image
 * @param height height of image
 *
 * @fn void FuseServer::setImageDimensions(unsigned int image_num, unsigned int w, unsigned int h)
 * Set image width.
 * @param image_num image number of image
 * @param w width of image
 * @param h height of image
 *
 * @fn void FuseServer::setImageROI(unsigned int image_num, unsigned int x, unsigned int y, unsigned int w, unsigned int h)
 * Set image's winning ROI.
 * This sets the ROI of the image that was finally processed and that contains the
 * found or unfound features.
 * @param image_num image number of image
 * @param x x coordinate of ROI
 * @param y y coordinate of ROI
 * @param w width of ROI
 * @param h height of ROI
 *
 * @fn void FuseServer::setImageCircle(unsigned int image_num, int x, int y, unsigned int r)
 * Set the circle found in the image.
 * This is a special function for the front vision and is badly abstracted. It denotes
 * the circle found in the image matching the ball.
 * @see setImageCircleFound()
 * @param image_num image number of image
 * @param x x coordinate of circle center
 * @param y y coordinate of circle center
 * @param r radius of circle
 *
 * @fn void FuseServer::setImageCircleFound(unsigned int image_num, bool found)
 * Set if a circle was found.
 * @see setImageCircle()
 * @param image_num image number of image
 * @param found true, if circle was found, false otherwise
 *
 * @fn void FuseServer::setImageBuffer(unsigned int image_num, unsigned char *buffer)
 * Set buffer containing the desired buffer.
 * This method honors the set image copy mode.
 * @param image_num image number of image
 * @param buffer buffer containing the requested image
 *
 * @fn void FuseServer::setImageAvailable(unsigned int image_num, bool available)
 * Set image availability.
 * @param image_num image number of image
 * @param available true if image is available, false otherwise.
 *
 * @fn void FuseServer::setImageAppAlive(unsigned int image_num, bool alive, std::string appl)
 * Set image providing application aliveness.
 * @param image_num image number of image
 * @param alive true if application is alive, false otherwise.
 * @param appl application name
 *
 * @fn std::vector<unsigned int>  FuseServer::getImageNumbers()
 * Get a list of available image numbers.
 * @return list of available images.
 *
 * @fn std::vector<unsigned int>  FuseServer::getRequestedImageNumbers()
 * Get a list of image numbers requested by clients.
 * @return list of available images.
 *
 * @fn void FuseServer::setLUT(unsigned int lut_id, unsigned int width, unsigned int height, unsigned int bytes_per_cell, unsigned char *data)
 * Set the LUT that is sent to the client.
 * @param lut_id LUT ID
 * @param width width of LUT
 * @param height height of LUT
 * @param bytes_per_cell how many bytes are used per cell
 * @param data LUT buffer
 *
 * @fn std::vector<unsigned int> FuseServer::getLutIDs()
 * Get list of available LUT IDs.
 * @return list of available LUT IDs.
 *
 * @fn std::vector<unsigned int>  FuseServer::getRequestedLutIDs()
 * Get a list of LUT IDs requested by clients.
 * @return list of available images.
 *
 * @fn void FuseServer::setLutAvailable(unsigned int lut_id, bool available)
 * Set availability of LUT.
 * @param lut_id LUT ID
 * @param available true, if LUT is available, false otherwise
 *
 * @fn void FuseServer::setLutAppAlive(unsigned int lut_id, bool alive, std::string appl)
 * Set LUT providing application aliveness.
 * @param lut_id image number of image
 * @param alive true if application is alive, false otherwise.
 * @param appl application name
 *
 * @fn void FuseServer::setUploadLutAppAlive(unsigned int lut_id, bool alive, std::string appl)
 * Set aliveness of application for which an LUT upload was requested.
 * @param lut_id LUT ID
 * @param alive true if application is alive, false otherwise
 * @param appl application name
 *
 * @fn std::vector<unsigned int> FuseServer::getUploadedLutIDs()
 * Get list of uploaded LUT IDs.
 * @return list of uploaded LUT IDs.
 *
 * @fn unsigned char * FuseServer::getUploadedLutBuffer(unsigned int lut_id)
 * Get uploaded LUT buffer.
 * @param lut_id LUT ID
 * @return buffer containing the uploaded LUT
 *
 * @fn unsigned int FuseServer::getUploadedLutBufferSize(unsigned int lut_id)
 * Get size of uploaded LUT buffer.
 * @param lut_id LUT ID
 * @return size in bytes of uploaded LUT buffer. 
 *
 * @fn unsigned int FuseServer::getUploadedLutWidth(unsigned int lut_id)
 * Get width of uploaded LUT buffer.
 * @param lut_id LUT ID
 * @return width as number of cells of uploaded LUT buffer. 
 *
 * @fn unsigned int FuseServer::getUploadedLutHeight(unsigned int lut_id)
 * Get height of uploaded LUT buffer.
 * @param lut_id LUT ID
 * @return height as number of cells of uploaded LUT buffer. 
 *
 * @fn unsigned int FuseServer::getUploadedLutBytesPerCell(unsigned int lut_id)
 * Get bytes per cell of uploaded LUT buffer.
 * @param lut_id LUT ID
 * @return Bytes per cell of uploaded LUT buffer. 
 *
 * @fn void FuseServer::setUploadLutSuccess(unsigned int lut_id, bool success)
 * Set if the LUT was uploaded successfully.
 * @param lut_id LUT ID
 * @param success true, if upload successful, false otherwise
 *
 * @fn void FuseServer::removeMessageQueueSubscriptions(unsigned int msgqid)
 * Remove message queue subscriptions.
 * Call this if a message queue disappears.
 * @param msgqid message queue ID
 *
 * @fn std::map< unsigned int, std::vector< std::pair< long, unsigned int > > > FuseServer::getMessageSubscriptions()
 * Get message queue subscriptions.
 * @return map of subscriptions, first map element contains message queue ID, second element
 * is a vector of pairs of message types and maximum data size per message.
 *
 * @fn bool FuseServer::isMessageAvailable()
 * Check if new message available.
 * @return true, if message available, false otherwise
 *
 * @fn void FuseServer::getMessage(unsigned int *msgqid, char **msg, unsigned int *msg_size)
 * Get first message from queue.
 * Use isMessageAvailable() before to check if a message is available. This throws
 * an exception if you try to get a message if there is none.
 * @param msgqid Message queue ID. Upon return contains message queue ID of message.
 * @param msg Upon return points to message content buffer.
 * @param msg_size Upon return denotes size of message contant.
 * @exception Exception thrown if there is no message available.
 *
 * @fn void FuseServer::freeMessage()
 * Free first message on queue.
 *
 * @fn void FuseServer::sendMessage(unsigned int msgqid, long mtype, char *msg, unsigned int msg_size)
 * Send a message to all matching subscribers.
 * @param msgqid message queue ID
 * @param mtype message type
 * @param msg message content
 * @param msg_size size of message content
 *
 */

/** Empty virtual destructor. */
FuseServer::~FuseServer()
{
}

