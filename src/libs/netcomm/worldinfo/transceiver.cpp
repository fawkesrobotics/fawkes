
/***************************************************************************
 *  transceiver.h - World Info Transceiver
 *
 *  Created: Sun Jan 21 14:15:32 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <core/exceptions/system.h>

#include <netcomm/worldinfo/transceiver.h>
#include <netcomm/worldinfo/messages.h>
#include <netcomm/worldinfo/encrypt.h>
#include <netcomm/worldinfo/decrypt.h>

#include <netcomm/socket/datagram_multicast.h>

#include <netinet/in.h>
#include <iostream>
using namespace std;


/** @class WorldInfoException <netcomm/worldinfo/transceiver.h>
 * Thrown on critical errors in world info handling.
 */

/** Constructor.
 * @param msg message
 */
WorldInfoException::WorldInfoException(const char *msg)
  : Exception(msg)
{
}


/** @class WorldInfoTransceiver netcomm/worldinfo/transceiver.h
 * Class to send and receive world information.
 * An important point in a domain of cooperating soccer robots is transmitting
 * and receiving a robot's belief of its surrounding. The world info
 * transceiver does exactly that. It allows for sending information about the
 * robot's pose and velocity and its perception of the ball and other robots
 * on the field.
 *
 * The unit for distances and positions is meter (m), speed is given in
 * meter per second (m/s), angles are given in radiant (rad). Angles can be in
 * the range 0 to 2 * PI or -PI to PI. Since they can be converted easily
 * between these ranges without further information users of such information
 * shall be able to process both.
 *
 * Coordinates are given in a right-handed coordinate system with the origin in
 * center of the field, X pointing towards the opponent goal, Y to the right
 * and Z downwards.
 *
 * Information is transmitted with a simple protocol via UDP Multicast packets.
 *
 * A call to send() will reset all information, thus all opponents are removed
 * from the list to be sent, positions of robot and ball are marked invalid.
 * You have to call the appropriate set methods before the information is sent.
 * You can thus call send() at any time but only changed information
 * (information set since last send() call) is transmitted over the network.
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param addr multicast address to send information to and receive from
 * @param port UDP port to send information to and receive from
 * @param key encryption key
 * @param iv encryption initialisation vector
 * @exception OutOfMemoryException thrown if internal buffers cannot be created
 */
WorldInfoTransceiver::WorldInfoTransceiver(const char *addr, unsigned short port,
					   const char *key, const char *iv)
{
  try {
    s = new MulticastDatagramSocket(addr, port);
    s->bind();
  } catch (SocketException &e) {
    e.append("WorldInfoTransceiver cannot instantiate socket for %s:%u", addr, port);
    throw;
  }

  in_buffer = malloc(WORLDINFO_MTU);
  out_buffer = malloc(WORLDINFO_MTU);
  if (! in_buffer || ! out_buffer) {
    throw OutOfMemoryException();
  }
  encryptor = new WorldInfoMessageEncryptor((const unsigned char *)key, (const unsigned char *)iv);
  decryptor = new WorldInfoMessageDecryptor((const unsigned char *)key, (const unsigned char *)iv);

  // set maximum size buffer to get valid results from encryptor
  encryptor->set_plain_buffer(out_buffer, WORLDINFO_MTU);

  crypt_buffer_size  = encryptor->recommended_crypt_buffer_size();
  crypted_out_buffer = malloc(crypt_buffer_size);
  crypted_in_buffer  = malloc(crypt_buffer_size);
  covariance         = (float *)malloc(WORLDINFO_COVARIANCE_SIZE * sizeof(float));

  if (! crypted_in_buffer || ! crypted_out_buffer || ! covariance) {
    throw OutOfMemoryException();
  }

  encryptor->set_crypt_buffer(crypted_out_buffer, crypt_buffer_size);

  decryptor->set_plain_buffer(in_buffer, WORLDINFO_MTU);

  out_seq = 0;
}


/** Destructor. */
WorldInfoTransceiver::~WorldInfoTransceiver()
{
  free(out_buffer);
  free(in_buffer);
  free(crypted_out_buffer);
  free(crypted_in_buffer);
  free(covariance);
  delete s;
  delete encryptor;
}


/** Set loopback of sent packets.
 * This sets whether packets should be looped back to local sockets for multicast
 * communication.
 * @param loop true to deliver sent packets to local sockets, false prevent delivering
 * @see MulticastDatagramSocket::set_loop()
 */
void
WorldInfoTransceiver::set_loop(bool loop)
{
  s->set_loop( loop );
}

/** Add a handler for world information.
 * World information will be dispatched to all registered handlers as soon it
 * is received.
 * @param h handler to register
 */
void
WorldInfoTransceiver::add_handler(WorldInfoHandler *h)
{
  handlers.lock();
  handlers.push_back(h);
  handlers.sort();
  handlers.unique();
  handlers.unlock();
}


/** Remove handler for world information.
 * The handler is removed from the list of handlers that incoming information
 * is dispatched to. No error is thrown if the handler was never registered
 * so it is safe to call this for any handler.
 * @param h handler to remove from subscriber list
 */
void
WorldInfoTransceiver::rem_handler(WorldInfoHandler *h)
{
  handlers.remove_locked(h);
}

/** Set global pose of robot.
 * Global pose of sensing robot (x, y, theta) with the origin in the
 * middle of the field, right handed coordinate system (y to opponent goal,
 * x to the right, z pointing upwards, same as in simulation league).
 * Theta points in y direction (theta = 0 iff robot front points to opponent
 * goal).
 * The confidence about the robot's pose is transmitted as a 3x3 covariance
 * matrix.
 * @param x x position of robot
 * @param y y position of robot
 * @param theta rotation of robot
 * @param covariance covariance matrix with 9 entries, ordered as three concatenated
 * rows (first row, three floats, second row, three floats, third row, three
 * floats). No length check or whatsoever is done. This will crash if c is not
 * long enough! c will not be copied but referenced so it has to exist when
 * send() is called!
 */
void
WorldInfoTransceiver::set_pose(float x, float y, float theta, float *covariance)
{
  pose_x          = x;
  pose_y          = y;
  pose_theta      = theta;
  pose_covariance = covariance;
  pose_changed    = true;
}


/** Set velocity of the robot.
 * Set the current velocity of the robot.
 * @param vel_x velocity in x direction
 * @param vel_y velocity in y direction
 * @param vel_theta rotational velocity, positive velocity means clockwise
 * rotation, negative velocity means counter-clockwise.
 */
void
WorldInfoTransceiver::set_velocity(float vel_x, float vel_y, float vel_theta)
{
  this->vel_x       = vel_x;
  this->vel_y       = vel_y;
  this->vel_theta   = vel_theta;
  this->vel_changed = true;
}


/** Set ball position.
 * Set the ball perception relative to the current robot position.
 * Note that the ball position is given in polar coordinates in
 * 3D space!
 * The confidence about the ball position is transmitted as a 3x3 covariance
 * matrix.
 * @param dist distance to ball in meters
 * @param pitch pitch angle to ball
 * @param yaw yaw angle to ball
 * @param covariance covariance matrix with 9 entries, ordered as three concatenated
 * rows (first row, three floats, second row, three floats, third row, three
 * floats). No length check or whatsoever is done. This will crash if c is not
 * long enough! c will not be copied but referenced so it has to exist when
 * send() is called!
 */
void
WorldInfoTransceiver::set_ball_pos(float dist, float pitch, float yaw, float *covariance)
{
  ball_dist       = dist;
  ball_pitch      = pitch;
  ball_yaw        = yaw;
  ball_covariance = covariance;
  ball_changed    = true;
}


/** Set ball velocity.
 * Set the current velocity of the robot.
 * @param vel_x velocity in x direction
 * @param vel_y velocity in y direction
 * @param vel_z velocity in z direction
 */
void
WorldInfoTransceiver::set_ball_velocity(float vel_x, float vel_y, float vel_z)
{
  ball_vel_x       = vel_x;
  ball_vel_y       = vel_y;
  ball_vel_z       = vel_z;
  ball_vel_changed = true;
}


/** Clear opponents list.
 * Clear the list of opponents that has to be transmitted. This is done
 * implicitly in send().
 */
void
WorldInfoTransceiver::clear_opponents()
{
  opponents.clear();
}


/** Add opponent to transmit list.
 * Add an opponent to the list of opponents to be transmitted on next send()
 * call. Opponents are given in a 2D polar coordinate system (assumption is that
 * robots don't fly in the soccer domain).
 * @param distance to opponent
 * @param angle angle to opponent (angle is zero if opponent is in front of robot,
 * positive if right of robot, negative if left of robot).
 */
void
WorldInfoTransceiver::add_opponent(float distance, float angle)
{
  opponent_t o = { distance, angle };
  opponents.push_back(o);
}


/** Append packet to outbound buffer.
 * @param msg_type message type
 * @param msg message buffer
 * @param msg_size size of message buffer
 * @exception OutOfMemoryException thrown if message is too big or if the
 * remaining size in the outbound buffer is not big enough
 */
void
WorldInfoTransceiver::append_outbound(uint16_t msg_type,
				      void *msg, uint16_t msg_size)
{
  worldinfo_message_header_t mh;

  if ( (outbound_bytes + sizeof(mh) + msg_size ) > WORLDINFO_MTU ) {
    throw OutOfMemoryException();
  }

  // per message header
  mh.type = htons(msg_type);
  mh.size = htons(msg_size);
  memcpy(outbound_buffer, &mh, sizeof(mh));

  outbound_bytes  += sizeof(mh);
  outbound_buffer += sizeof(mh);
  
  // message body
  memcpy(outbound_buffer, msg, msg_size);
  outbound_bytes  += msg_size;
  outbound_buffer += msg_size;
  ++outbound_num_msgs;
}


/** Reset outbound buffer.
 */
void
WorldInfoTransceiver::reset_outbound()
{
  worldinfo_header_t *header = (worldinfo_header_t *)out_buffer;
  header->beef = htons(0xBEEF);
  header->version  = WORLDINFO_VERSION;

  outbound_buffer   = (unsigned char *)out_buffer + sizeof(worldinfo_header_t);
  outbound_bytes    = sizeof(worldinfo_header_t);
  outbound_num_msgs = 0;
}


/** Send information.
 * All information that has been set since last call is sent over the network.
 * This implicitly resets all information and flushes the opponent list.
 */
void
WorldInfoTransceiver::send()
{
  worldinfo_header_t *header = (worldinfo_header_t *)out_buffer;

  reset_outbound();

  if ( pose_changed ) {
    worldinfo_pose_message_t pm;
    pm.x = pose_x;
    pm.y = pose_y;
    pm.theta = pose_theta;
    memcpy(&(pm.covariance), pose_covariance, sizeof(pm.covariance));
    pose_changed = false;

    append_outbound(WORLDINFO_MSGTYPE_POSE, &pm, sizeof(pm));
  }

  if ( vel_changed ) {
    worldinfo_velocity_message_t vm;
    vm.vel_x     = vel_x;
    vm.vel_y     = vel_y;
    vm.vel_theta = vel_theta;
    vel_changed = false;

    append_outbound(WORLDINFO_MSGTYPE_VELO, &vm, sizeof(vm));
  }

  if ( ball_changed ) {
    worldinfo_relballpos_message_t bm;
    bm.dist  = ball_dist;
    bm.pitch = ball_pitch;
    bm.yaw   = ball_yaw;
    memcpy(&(bm.covariance), ball_covariance, sizeof(bm.covariance));
    ball_changed = false;

    append_outbound(WORLDINFO_MSGTYPE_RELBALL, &bm, sizeof(bm));
  }

  if ( ball_vel_changed ) {
    worldinfo_relballvelo_message_t rbvm;
    rbvm.vel_x = ball_vel_x;
    rbvm.vel_y = ball_vel_y;
    rbvm.vel_z = ball_vel_z;
    ball_vel_changed = false;

    append_outbound(WORLDINFO_MSGTYPE_RELBALLVELO, &rbvm, sizeof(rbvm));
  }

  // Append opponents
  for ( oppit = opponents.begin(); oppit != opponents.end(); ++oppit) {
    worldinfo_opppose_message_t opm;
    opm.dist  = (*oppit).distance;
    opm.angle = (*oppit).angle;

    append_outbound(WORLDINFO_MSGTYPE_OPP_POSE, &opm, sizeof(opm));
  }
  opponents.clear();

  if ( outbound_num_msgs > 0 ) {
    header->seq      = htonl(out_seq++);

    encryptor->set_plain_buffer(out_buffer, outbound_bytes);
    crypted_out_bytes = encryptor->encrypt();

    s->send(crypted_out_buffer, crypted_out_bytes);
  }

}


/** Receive information.
 * This checks if there is information on the network waiting to be received
 * and if so receives and processes the information and dispatches it to all
 * registered handlers. If you order it to block this method will block until
 * information has been received and dispatched (useful if running in a
 * thread).
 *
 * Received packets will be ignored if
 * - they do not start with 0xBEEF
 * - they are of an incompatible version
 * - the sequence number is smaller or equal to an already received packet
 * They will only be partially handled if
 * - a packet has been truncated (truncated message is ignored)
 * - an unknown message type is encountered (message is ignored)
 * - a message size does not match the expected size for a given type (message is ignored)
 *
 * @param block set to true for blocking operation, in this case recv() will
 * block until data is available, false for non-blocking operation where recv()
 * will immediately return if there is no data available
 * @param max_num_msgs maximum number of messages to process in a single
 * call to recv(). Set to 0 for an unlimited number of messages per call (this
 * can block for an infinite time if messages are coming in fast).
 */
void
WorldInfoTransceiver::recv(bool block, unsigned int max_num_msgs)
{
  if ( ! block ) {
    if ( ! s->available() ) {
      return;
    }
  }

  handlers.lock();

  unsigned int num_msgs = (max_num_msgs == 0 ? 0 : 1);
  do {
    struct sockaddr_in from;
    size_t addr_len = sizeof(addr_len);
    size_t bytes = crypt_buffer_size;

    if ( max_num_msgs != 0 )  ++num_msgs;

    s->recv(crypted_in_buffer, &bytes, (struct sockaddr *)&from, &addr_len);

    // decryptor decrypts to in_buffer, see constructor
    decryptor->set_crypt_buffer(crypted_in_buffer, bytes);
    try {
      inbound_bytes = decryptor->decrypt();
    } catch (MessageDecryptionException &e) {
      e.printTrace();
      cout << "Message decryption failed, ignoring" << endl;
      continue;
    }

    /*
    cout << "Plain:";
    for (size_t i = 0; i < inbound_bytes; ++i) {
      unsigned int u = *((unsigned char *)in_buffer + i); 
      printf("%02x ", u);
    }
    cout << endl;
    */

    // Process
    worldinfo_header_t *header = (worldinfo_header_t *)in_buffer;
    if ( ntohs(header->beef) != 0xBEEF ) {
      // throw WorldInfoException("Incorrect message received, wrong key?");
      cout << "Invalid message received, ignoring" << endl;
      continue;
    }

    if ( header->version != WORLDINFO_VERSION ) {
      cout << "Unsupported version of world info data received, ignoring" << endl;
      continue;
    }
    
    // Sequence number handling per client, IPv4 only, for IPv6 in the pre-128-bit era
    // we would need a custom compare function
    unsigned int cseq = ntohl(header->seq);
    if ( sequence_numbers.find(from.sin_addr.s_addr) != sequence_numbers.end() ) {
      if ( cseq <= sequence_numbers[from.sin_addr.s_addr] ) {
	// Already received (loop) or replay attack, just ignore
	cout << "Received packet twice, ignoring" << endl;
	continue;
      }
    }
    sequence_numbers[from.sin_addr.s_addr] = cseq;

    inbound_bytes -= sizeof(worldinfo_header_t);
    inbound_buffer = (unsigned char *)in_buffer + sizeof(worldinfo_header_t);

    // Go through messages
    while ( inbound_bytes > 0 ) {
      worldinfo_message_header_t *msgh = (worldinfo_message_header_t *)inbound_buffer;
      inbound_bytes  -= sizeof(worldinfo_message_header_t);
      inbound_buffer += sizeof(worldinfo_message_header_t);
      uint16_t msg_type = ntohs(msgh->type);
      uint16_t msg_size = ntohs(msgh->size);
      //cout << "Message type: " << msg_type << "  size: " << msg_size
      //     << "  ntype: " << msgh->type << "  nsize: " << msgh->size << endl;
      if ( inbound_bytes < msg_size ) {
	cout << "Truncated packet received or protocol error, ignoring rest of packet" << endl;
	break;
      }
      switch ( msg_type ) {
      case WORLDINFO_MSGTYPE_POSE:
	if ( msg_size == sizeof(worldinfo_pose_message_t) ) {
	  worldinfo_pose_message_t *pose_msg = (worldinfo_pose_message_t *)inbound_buffer;
	  for ( hit = handlers.begin(); hit != handlers.end(); ++hit ) {
	    memcpy(covariance, pose_msg->covariance, WORLDINFO_COVARIANCE_SIZE * sizeof(float));
	    (*hit)->pose_rcvd("test",
			      pose_msg->x, pose_msg->y, pose_msg->theta, covariance);
	  }
	} else {
	  cout << "Received pose message of invalid size, ignoring" << endl;
	}
	break;

      case WORLDINFO_MSGTYPE_VELO:
	if ( msg_size == sizeof(worldinfo_velocity_message_t) ) {
	  worldinfo_velocity_message_t *velo_msg = (worldinfo_velocity_message_t *)inbound_buffer;
	  for ( hit = handlers.begin(); hit != handlers.end(); ++hit ) {
	    (*hit)->velocity_rcvd("test",
				  velo_msg->vel_x, velo_msg->vel_y, velo_msg->vel_theta);
	  }
	} else {
	  cout << "Received velocity message of invalid size, ignoring" << endl;
	}
	break;

      case WORLDINFO_MSGTYPE_RELBALL:
	if ( msg_size == sizeof(worldinfo_relballpos_message_t) ) {
	  worldinfo_relballpos_message_t *ball_msg = (worldinfo_relballpos_message_t *)inbound_buffer;
	  for ( hit = handlers.begin(); hit != handlers.end(); ++hit ) {
	    memcpy(covariance, ball_msg->covariance, WORLDINFO_COVARIANCE_SIZE * sizeof(float));
	    (*hit)->ball_pos_rcvd("test",
				  ball_msg->dist, ball_msg->pitch, ball_msg->yaw,
				  covariance);
	  }
	} else {
	  cout << "Received relative ball pos message of invalid size, ignoring" << endl;
	}
	break;

      case WORLDINFO_MSGTYPE_RELBALLVELO:
	if ( msg_size == sizeof(worldinfo_relballvelo_message_t) ) {
	  worldinfo_relballvelo_message_t *bvel_msg = (worldinfo_relballvelo_message_t *)inbound_buffer;
	  for ( hit = handlers.begin(); hit != handlers.end(); ++hit ) {
	    (*hit)->ball_velocity_rcvd("test",
				       bvel_msg->vel_x, bvel_msg->vel_y, bvel_msg->vel_z);
	  }
	} else {
	  cout << "Received relative ball velocity message of invalid size, ignoring" << endl;
	}
	break;

      case WORLDINFO_MSGTYPE_OPP_POSE:
	if ( msg_size == sizeof(worldinfo_opppose_message_t) ) {
	  worldinfo_opppose_message_t *oppp_msg = (worldinfo_opppose_message_t *)inbound_buffer;
	  for ( hit = handlers.begin(); hit != handlers.end(); ++hit ) {
	    (*hit)->opponent_pose_rcvd("test",
				       oppp_msg->dist, oppp_msg->angle);
	  }
	} else {
	  cout << "Received relative ball pos message of invalid size, ignoring" << endl;
	}
	break;


      default:
	cout << "Unknown message type " << msg_type << " received, ignoring" << endl;
      }
      // there is more to process
      inbound_bytes  -= msg_size;
      inbound_buffer += msg_size;
    }

  } while ( s->available() && (num_msgs <= max_num_msgs) );

  handlers.unlock();
}


/** Get last sent plain buffer.
 * This method is meant to be used for debugging and testing purposes only.
 * @return last plain text message buffer
 */
void *
WorldInfoTransceiver::last_sent_plain_buffer()
{
  return out_buffer;
}


/** Get last sent plain buffer size.
 * This method is meant to be used for debugging and testing purposes only.
 * @return last plain text message buffer size
 */
size_t
WorldInfoTransceiver::last_sent_plain_buffer_size()
{
  return outbound_bytes;
}


/** Get last sent crypted buffer.
 * This method is meant to be used for debugging and testing purposes only.
 * @return last crytped message buffer
 */
void *
WorldInfoTransceiver::last_sent_crypted_buffer()
{
  return crypted_out_buffer;
}


/** Get last sent crypted buffer size.
 * This method is meant to be used for debugging and testing purposes only.
 * @return last crypted message buffer size
 */
size_t
WorldInfoTransceiver::last_sent_crypted_buffer_size()
{
  return crypted_out_bytes;
}
