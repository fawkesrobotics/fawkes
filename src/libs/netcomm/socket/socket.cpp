
/***************************************************************************
 *  socket.h - Fawkes socket base class
 *
 *  Created: Thu Nov 09 14:30:56 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/socket/socket.h>

#include <core/exceptions/system.h>
#include <utils/time/time.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
// include <linux/in.h>
#include <netinet/in.h>
#include <netinet/ip.h> 
#include <poll.h>

// Until this is integrated from linux/in.h to netinet/in.h
#ifdef __linux
  #ifndef IP_MTU
    #define IP_MTU 14
  #endif
#endif  // __linux

#include <stdio.h>

/** @class SocketException netcomm/socket/socket.h
 * Socket exception.
 * Thrown if an exception occurs in a socket. If the error was caused by
 * a system call that sets errno this is given to the exception.
 * @ingroup NetComm
 */

/** Constructor.
 * @param msg reason that caused the exception.
 */
SocketException::SocketException(const char *msg)
  : Exception(msg)
{
}


/** Constructor.
 * @param msg reason of the exception
 * @param _errno error number (errno returned by a syscall)
 */
SocketException::SocketException(const char *msg, int _errno)
  : Exception(msg, _errno)
{
}


/** @class Socket netcomm/socket/socket.h
 * Socket base class.
 * This is the base class for all sockets. You cannot use it directly
 * but you have to use one of the derivatives like StreamSocket or
 * DatagramSocket.
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** @var Socket::sock_fd
 * Socket file descriptor.
 */
/** @var Socket::timeout
 * Timeout in seconds for various operations. If the timeout is non-zero
 * the socket is initialized non-blocking and operations are aborted after
 * timeout seconds have passed.
 */
/** @var Socket::client_addr
 * Client address, set if connected.
 */
/** @var Socket::client_addr_len
 * length in bytes of client address.
 */


/** Data can be read. */
const short Socket::POLL_IN    = POLLIN;

/** Writing will not block. */
const short Socket::POLL_OUT   = POLLOUT;

/** There is urgent data to read (e.g., out-of-band data on TCP socket;
 * pseudo-terminal master in packet mode has seen state  change  in slave).
 */
const short Socket::POLL_PRI   = POLLPRI;

/** Stream  socket  peer closed connection, or shut down writing half of
 * connection.  The _GNU_SOURCE feature test macro must be defined
 * in order to obtain this definition (since Linux 2.6.17).
 */
#ifdef POLLRDHUP
const short Socket::POLL_RDHUP = POLLRDHUP;
#else
const short Socket::POLL_RDHUP = 0;
#endif

/** Error condition. */
const short Socket::POLL_ERR   = POLLERR;

/** Hang up. */
const short Socket::POLL_HUP   = POLLHUP;

/** Invalid request. */
const short Socket::POLL_NVAL  = POLLNVAL;


/**  Constructor similar to syscall.
 * This creates a new socket. This is a plain pass-through constructor
 * to the socket() syscall. In most cases this should only be used by
 * a derivate.
 * @param domain communication domain, selects the protocol
 * @param type type of the sockets which specifies communication semantics
 * @param protocol protocol to use, most types support only one and protocol
 * should be 0
 * @param timeout See Socket::timeout.
 * @exception SocketException thrown if socket cannot be opened, check errno for cause
 */
Socket::Socket(int domain, int type, int protocol, float timeout)
{
  this->timeout = timeout;
  if ( (sock_fd = socket(domain, type, protocol)) == -1 ) {
    throw SocketException("Could not open socket", errno);
  }

  if (timeout > 0.f) {
    // set to non-blocking
    if ( fcntl(sock_fd, F_SETFL, O_NONBLOCK) == -1 ) {
      throw SocketException("Could not set socket to non-blocking", errno);
    }
  }

  client_addr = NULL;
  client_addr_len = 0;
}


/** Constructor.
 * Plain constructor. The socket will not be opened. This may only be called by
 * sub-classes and you must ensure that the socket file descriptor is initialized
 * properly.
 */
Socket::Socket()
{
  client_addr = NULL;
  client_addr_len = 0;
  timeout = 0.f;
  sock_fd = -1;
}


/** Copy constructor.
 * @param socket socket to copy
 */
Socket::Socket(Socket &socket)
{
  if ( socket.client_addr != NULL ) {
    client_addr = (struct sockaddr_in *)malloc(socket.client_addr_len);
    client_addr_len = socket.client_addr_len;
    memcpy(client_addr, socket.client_addr, client_addr_len);
  } else {
    client_addr = NULL;
    client_addr_len = 0;
  }    
  timeout = socket.timeout;
  sock_fd = socket.sock_fd;
}


/** Destructor. */
Socket::~Socket()
{
  close();
  if ( client_addr != NULL ) {
    free(client_addr);
    client_addr = NULL;
  }
}


/** Close socket. */
void
Socket::close()
{
  if ( sock_fd != -1 ) {
    ::close(sock_fd);
    sock_fd = -1;
  }
}


/** Connect socket.
 * If called for a stream socket this will connect to the remote address. If
 * you call this on a datagram socket you will tune in to a specific sender and
 * receiver.
 * @param addr_port struct containing address and port to connect to
 * @param struct_size size of addr_port struct
 * @exception SocketException thrown if socket cannot connect, check errno for cause
 */
void
Socket::connect(struct sockaddr *addr_port, unsigned int struct_size)
{
  if ( sock_fd == -1 )  throw SocketException("Trying to connect invalid socket");

  if (timeout == 0.f) {
    if ( ::connect(sock_fd, addr_port, struct_size) < 0 ) {
      throw SocketException("Could not connect", errno);
    }
  } else {
    struct timeval start, now;
    gettimeofday(&start, NULL);
    do {
      if ( ::connect(sock_fd, addr_port, struct_size) < 0 ) {
	if ( (errno != EINPROGRESS) &&
	     (errno != EALREADY) ) {
	  throw SocketException("Could not connect", errno);
	}
      }
      gettimeofday(&now, NULL);
    } while (time_diff_sec(now, start) < timeout);
  }
}


/** Connect socket.
 * If called for a stream socket this will connect to the remote address. If
 * you call this on a datagram socket you will tune in to a specific sender and
 * receiver.
 * @param hostname hostname or textual represenation of IP address to connect to
 * @param port port to connect to
 * @exception SocketException thrown if socket cannot connect, check errno for cause
 */
void
Socket::connect(const char *hostname, unsigned short int port)
{
  if ( sock_fd == -1 )  throw SocketException("Trying to connect invalid socket");

  struct hostent* h;
  struct sockaddr_in host;


  h = gethostbyname(hostname);
  if ( ! h ) {
    throw SocketException("Cannot lookup hostname", h_errno);
  }

  memset(&host, 0, sizeof(host));
  host.sin_family = AF_INET;
  memcpy((char *)&host.sin_addr.s_addr, h->h_addr, h->h_length);
  host.sin_port = htons(port);

  connect((struct sockaddr *)&host, sizeof(host));
}


/** Bind socket.
 * Can only be called on stream sockets.
 * @param port port to bind
 * @exception SocketException thrown if socket cannot bind, check errno for cause
 */
void
Socket::bind(const unsigned short int port)
{
  struct sockaddr_in host;

  host.sin_family = AF_INET;
  host.sin_addr.s_addr = INADDR_ANY;
  host.sin_port = htons(port);

  int reuse = 1;
  if ( setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == -1) {
    throw SocketException("Could not set SO_REUSEADDR", errno);
  }

  if (::bind(sock_fd, (struct sockaddr *) &host, sizeof(host)) < 0) {
    throw SocketException("Could not bind to port", errno);
  }
}


/** Bind socket to a specific address.
 * @param port port to bind
 * @param hostname hostname or textual IP address of a local interface to bind to.
 * @exception SocketException thrown if socket cannot bind, check errno for cause
 */
void
Socket::bind(const unsigned short int port, const char *hostname)
{
  struct hostent* h;
  struct sockaddr_in host;

  h = gethostbyname(hostname);
  if ( ! h ) {
    throw SocketException("Cannot lookup hostname", h_errno);
  }

  memset(&host, 0, sizeof(host));
  host.sin_family = AF_INET;
  memcpy(&host.sin_addr.s_addr, h->h_addr, h->h_length);
  host.sin_port = htons(port);

  host.sin_family = AF_INET;
  host.sin_addr.s_addr = INADDR_ANY;
  host.sin_port = htons(port);

  if (::bind(sock_fd, (struct sockaddr *) &host, sizeof(host)) < 0) {
    throw SocketException("Could not bind to port", errno);
  }
}


/** Listen on socket.
 * This waits for new connections on a bound socket. The backlog is the maximum
 * number of connections waiting for being accepted.
 * @param backlog maximum number of waiting connections
 * @exception SocketException thrown if socket cannot listen, check errno for cause
 * @see bind()
 * @see accept()
 */
void
Socket::listen(int backlog)
{
  if ( ::listen(sock_fd, backlog) ) {
    throw SocketException("Cannot listen on socket", errno);
  }
}


/** Accept connection.
 * Accepts a connection waiting in the queue.
 * @return new socket used to communicate with the remote part
 * @exception SocketException thrown if socket cannot accept, check errno for cause
 */
Socket *
Socket::accept()
{
  struct sockaddr_in  tmp_client_addr;
  unsigned int  tmp_client_addr_len = sizeof(struct sockaddr_in);

  int a_sock_fd = -1;

  a_sock_fd = ::accept(sock_fd, (sockaddr *)&tmp_client_addr, &tmp_client_addr_len);
  if ( a_sock_fd == -1 ) {
    if (errno != EWOULDBLOCK) {
      throw SocketException("Could not accept connection", errno);
    } else {
      return NULL;
    }
  }

  // Does not work, evaluated at compile time, thus need clone()
  //__typeof(this) s = new __typeof(*this);
  //s->timeout = timeout;

  Socket *s = clone();
  s->sock_fd = a_sock_fd;

  if ( s->client_addr != NULL ) {
    free(s->client_addr);
  }
  struct sockaddr_in  *tmp_client_addr_alloc = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
  memcpy(tmp_client_addr_alloc, &tmp_client_addr, sizeof(struct sockaddr_in));
  s->client_addr = tmp_client_addr_alloc;
  s->client_addr_len = tmp_client_addr_len;

  return s;
}


/** Check if data is available.
 * Use this to check if data is available on the socket for reading.
 * @return true, if data can be read, false otherwise
 */
bool
Socket::available()
{
  if (! sock_fd == -1) return false;

  fd_set rfds;
  struct timeval tv;
  int retval = 1;

  FD_ZERO(&rfds);
  FD_SET(sock_fd, &rfds);
  tv.tv_sec = 0;
  tv.tv_usec = 0;

  retval = select(sock_fd + 1, &rfds, NULL, NULL, &tv);
  if ( retval < 0 ) {
    perror("select() failed");
  }

  return (retval > 0);
}


/** Wait for some event on socket.
 * @param timeout timeout in miliseconds to wait. A negative value means to
 * wait forever until an event occurs, zero means just check, don't wait.
 * @param what what to wait for, a bitwise OR'ed combination of POLL_IN,
 * POLL_OUT and POLL_PRI.
 * @return Returns a flag value. Use bit-wise AND with the POLL_* constants
 * in this class.
 * @exception InterruptedException thrown, if poll is interrupted by a signal
 * @exception SocketException thrown for any other error the poll() syscall can cause,
 * see Exception::errno() for the cause of the error.
 * @see Socket::POLL_IN
 * @see Socket::POLL_OUT
 * @see Socket::POLL_PRI
 * @see Socket::POLL_RDHUP
 * @see Socket::POLL_ERR
 * @see Socket::POLL_HUP
 * @see Socket::POLL_NVAL
 */
short
Socket::poll(int timeout, short what)
{
  if ( sock_fd == -1 ) {
    return POLL_ERR;
  }

  struct pollfd pfd;
  pfd.fd = sock_fd;
  pfd.events = what;
  pfd.revents = 0;
  if ( ::poll(&pfd, 1, timeout) == -1 ) {
    if ( errno == EINTR ) {
      throw InterruptedException();
    } else {
      throw SocketException("poll() failed", errno);
    }
  } else {
    return pfd.revents;
  }
}


/** Write to the socket.
 * Write to the socket. This method can only be used on streams.
 * @param buf buffer to write
 * @param count number of bytes to write from buf
 * @exception SocketException if the data could not be written or if a timeout occured.
 */
void
Socket::write(const void *buf, size_t count)
{
  int retval = 0;
  unsigned int bytes_written = 0;
  struct timeval start, now;

  gettimeofday(&start, NULL);

  do {
    retval = ::write(sock_fd, (char *)buf + bytes_written, count - bytes_written);
    if (retval == -1) {
      if (errno != EAGAIN) {
	throw SocketException("Could not write data", errno);
      } else {
	// just to meet loop condition
	retval = 0;
      }
    } else {
      bytes_written += retval;
      // reset timeout
      gettimeofday(&start, NULL);
    }
    gettimeofday(&now, NULL);
    usleep(0);
  } while ((bytes_written < count) && (time_diff_sec(now, start) < timeout) );

  if ( bytes_written < count) {
    throw SocketException("Write timeout");
  }
}


/** Read from socket.
 * Read from the socket. This method can only be used on streams.
 * @param buf buffer to write from
 * @param count length of buffer, number of bytes to write to stream
 * @see write
 * @exception SocketException thrown for any error during reading
 */
void
Socket::read(void *buf, size_t count)
{
  int retval = 0;
  unsigned int bytes_read = 0;

  if ( timeout > 0 ) {
    struct timeval start, now;

    gettimeofday(&start, NULL);

    do {
      retval = ::read(sock_fd, (char *)buf + bytes_read, count - bytes_read);
      if (retval == -1) {
	if (errno != EAGAIN) {
	  throw SocketException("Could not read data", errno);
	} else {
	  // just to meet loop condition
	  retval = 0;
	}
      } else {
	bytes_read += retval;
	// reset timeout
	gettimeofday(&start, NULL);
      }
      gettimeofday(&now, NULL);
      usleep(0);
    } while ((bytes_read < count) && (time_diff_sec(now, start) < timeout) );
  } else {
    do {
      retval = ::read(sock_fd, (char *)buf + bytes_read, count - bytes_read);
      if (retval == -1) {
	throw SocketException("Could not read data", errno);
      } else if (retval == 0) {
	throw SocketException("Could not read any data");
      } else {
	bytes_read += retval;
      }
      usleep(0);
    } while (bytes_read < count);
  }

  if ( bytes_read < count) {
    throw SocketException("Read timeout");
  }
}


/** Write to the socket.
 * Write to the socket. This method can be used on streams or on datagram
 * sockets which have been tuned to a specific receiver by using connect().
 * For streams usage of write() is recommended as it is the more intuitive
 * way to deal with a stream.
 * @param buf buffer to write
 * @param buf_len length of buffer, number of bytes to write to stream
 * @see write
 */
void
Socket::send(void *buf, size_t buf_len)
{
  try {
    write(buf,  buf_len);
  } catch (SocketException &e) {
    throw;
  }
}


/** Read from socket.
 * Read from the socket. This method can only be used on streams. Usage of
 * read() is recommended.
 * @param buf buffer to read data into
 * @param buf_len length of buffer, number of bytes to read from stream
 * @exception SocketException thrown if an error occurs or the other side
 * has closed the connection.
 */
size_t
Socket::recv(void *buf, size_t buf_len)
{
  ssize_t rv;
  if ( (rv = ::recv(sock_fd, buf, buf_len, 0)) == -1 ) {
    throw SocketException("recv() failed", errno);
  } else if ( rv == 0 ) {
    throw SocketException("Other side closed the connection");
  }
  return rv;
}


/** Send message.
 * @param buf buffer with data to send
 * @param buf_len length of buffer, all data will be send.
 * @param addr addr to send data to.
 * @param addr_len length of address
 */
void
Socket::send(void *buf, size_t buf_len,
	     const struct sockaddr *addr, socklen_t addr_len)
{
  int retval = 0;
  unsigned int bytes_written = 0;
  struct timeval start, now;

  gettimeofday(&start, NULL);

  do {
    retval = ::sendto(sock_fd, (char *)buf + bytes_written, buf_len - bytes_written, 0,
		      addr, addr_len);
    if (retval == -1) {
      if (errno != EAGAIN) {
	throw SocketException("Could not read data", errno);
      } else {
	// just to meet loop condition
	retval = 0;
      }
    } else {
      bytes_written += retval;
      // reset timeout
      gettimeofday(&start, NULL);
    }
    gettimeofday(&now, NULL);
    usleep(0);
  } while ((bytes_written < buf_len) && (time_diff_sec(now, start) < timeout) );

  if ( bytes_written < buf_len) {
    throw SocketException("Write timeout");
  }
}


/** Receive data.
 * This will use recvfrom() to read data from the socket and returns the
 * number of bytes actually read. It will not wait until the requested
 * number of bytes has been read. Use read() if you need this.
 * @param buf buffer that read data shall be stored in.
 * @param buf_len length of buffer and number of bytes to be read
 * @param addr return parameter, contains address of sender
 * @param addr_len initially has to contain size of address, on return
 * contains the actual bytes used.
 */
size_t
Socket::recv(void *buf, size_t buf_len,
	     struct sockaddr *addr, socklen_t *addr_len)
{
  ssize_t rv = 0;

  if ( (rv = ::recvfrom(sock_fd, buf, buf_len, 0, addr, addr_len)) == -1) {
    throw SocketException("recvfrom() failed", errno);
  } else if ( rv == 0 ) {
    throw SocketException("Peer has closed the connection");
  } else {
    return rv;
  }
}


/** Is socket listening for connections?
 * @return true if socket is listening for incoming connections, false otherwise
 */
bool
Socket::listening()
{
  if ( sock_fd == -1 ) return false;

  int i = 0;
  unsigned int len = sizeof(i);
  if ( getsockopt(sock_fd, SOL_SOCKET, SO_ACCEPTCONN, &i, &len) == -1 ) {
    throw SocketException("Socket::listening(): getsockopt failed", errno);
  }
  return ( i == 1 );
}


/** Maximum Transfer Unit (MTU) of socket.
 * Note that this can only be retrieved of connected sockets!
 * @return MTU in bytes
 */
unsigned int
Socket::mtu()
{
  if ( sock_fd == -1 ) throw SocketException("Cannot get MTU of disconnected socket");

  int m = 0;
  unsigned int len = sizeof(m);
  if ( getsockopt(sock_fd, IPPROTO_IP, IP_MTU, &m, &len) == -1 ) {
    throw SocketException("Socket::mtu(): getsockopt failed", errno);
  }

  if ( m < 0 ) {
    throw SocketException("MTU < 0");
  }

  return m;
}
