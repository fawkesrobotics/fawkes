
/***************************************************************************
 *  socket.h - Fawkes socket base class
 *
 *  Created: Thu Nov 09 12:55:25 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_SOCKET_SOCKET_H_
#define __NETCOMM_SOCKET_SOCKET_H_

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
// just to be safe nobody else can do it
#include <sys/signal.h>

#ifdef POLL_IN
#  undef POLL_IN
#endif
#ifdef POLL_OUT
#  undef POLL_OUT
#endif
#ifdef POLL_PRI
#  undef POLL_PRI
#endif
#ifdef POLL_RDHUP
#  undef POLL_RDHUP
#endif
#ifdef POLL_ERR
#  undef POLL_ERR
#endif
#ifdef POLL_HUP
#  undef POLL_HUP
#endif


namespace fawkes {

class SocketException : public Exception
{
 public:
  SocketException(const char *msg, int _errno);
  SocketException(const char *msg);
};

class Socket
{
 public:

  static const short POLL_IN;
  static const short POLL_OUT;
  static const short POLL_PRI;
  static const short POLL_RDHUP;
  static const short POLL_ERR;
  static const short POLL_HUP;
  static const short POLL_NVAL;

  Socket(int domain, int type, int protocol, float timeout = 0.f);
  Socket(Socket &socket);
  virtual ~Socket();

  virtual void         connect(const char *hostname, const unsigned short int port);
  virtual void         connect(struct sockaddr *addr_port, unsigned int struct_size);

  virtual void         bind(const unsigned short int port);
  virtual void         bind(const unsigned short int port,
			    const char *hostname);

  virtual void         listen(int backlog = 1);
  virtual Socket *     accept();
  virtual void         close();
  virtual bool         available();

  virtual size_t       read(void *buf, size_t count, bool read_all = true);
  virtual void         write(const void *buf, size_t count);
  virtual void         send(void *buf, size_t buf_len);
  virtual void         send(void *buf, size_t buf_len,
			    const struct sockaddr *to_addr, socklen_t addr_len);
  virtual size_t       recv(void *buf, size_t buf_len);
  virtual size_t       recv(void *buf, size_t buf_len,
			    struct sockaddr *from_addr, socklen_t *addr_len);

  /** Clone socket.
   * This method has to be implemented by subclass to correctly clone the instance.
   * @return cloned socket
   */
  virtual Socket *     clone() = 0;

  virtual short        poll(int timeout = -1, short what = POLL_IN | POLL_HUP | POLL_PRI | POLL_RDHUP);

  virtual bool         listening();

  virtual unsigned int mtu();

  /** Accept connection.
   * This method works like accept() but it ensures that the returned socket is of
   * the given type.
   * @return socket to client
   */
  template <class SocketType>
    SocketType *     accept();

 protected:
  Socket();

  int sock_fd;
  float timeout;
  struct ::sockaddr_in  *client_addr;
  unsigned int         client_addr_len;

};


template <class SocketType>
SocketType *
Socket::accept()
{
  Socket *s = accept();
  if (SocketType *ts = dynamic_cast<SocketType *>(s)) {
    return ts;
  } else {
    delete s;
    throw TypeMismatchException("Socket types do not match");
  }
}

} // end namespace fawkes

#endif
