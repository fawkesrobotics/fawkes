
/***************************************************************************
 *  qa_socket_datagram.cpp - Fawkes QA DatagramSocket
 *
 *  Created: Tue Nov 14 11:43:00 2006
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

/// @cond QA

#include <core/threading/thread.h>
#include <netcomm/socket/datagram.h>
#include <utils/system/signal.h>

#include <netdb.h>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>

using namespace fawkes;

class DatagramServerThread : public Thread
{
public:
  DatagramServerThread(unsigned short int port, unsigned int to_port)
    : Thread("DatagramServerThread", Thread::OPMODE_CONTINUOUS)
  {
    i = 0;
    s = new DatagramSocket();
    s->bind(port);

    struct hostent* h;

    h = gethostbyname("127.0.0.1");

    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    memcpy((char *)&to.sin_addr.s_addr, h->h_addr, h->h_length);
    to.sin_port = htons(to_port);

  }

  ~DatagramServerThread()
  {
    printf("Closing server socket\n");
    s->close();
    printf("Closed server socket\n");
    delete s;
  }

  virtual void loop()
  {
    s->send(&i, sizeof(i), (struct sockaddr *)&to, sizeof(to));
    unsigned int ri = 0;
    from_len = sizeof(from);
    s->recv(&ri, sizeof(ri), (struct sockaddr *)&from, &from_len);
    if ( ri != i ) {
	printf("ERROR: sent %u but received %u\n", i, ri);
    } else {
      printf("OK: sent %u and received %u\n", i, ri);
    }
    ++i;
  }

 private:
  unsigned int i;
  DatagramSocket *s;
  struct sockaddr_in to;
  struct sockaddr_in from;
  unsigned int from_len;
};


class DatagramClientThread : public Thread
{
public:
  DatagramClientThread(unsigned short int port, unsigned int to_port)
    : Thread("DatagramClientThread", Thread::OPMODE_CONTINUOUS)
  {
    s = new DatagramSocket();
    s->bind(port);

    struct hostent* h;

    h = gethostbyname("127.0.0.1");

    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    memcpy((char *)&to.sin_addr.s_addr, h->h_addr, h->h_length);
    to.sin_port = htons(to_port);

  }

  ~DatagramClientThread()
  {
    printf("Closing server socket\n");
    s->close();
    printf("Closed server socket\n");
    delete s;
  }

  virtual void loop()
  {
    unsigned int i = 0;
    from_len = sizeof(from);
    s->recv(&i, sizeof(i), (struct sockaddr *)&from, &from_len);
    s->send(&i, sizeof(i), (struct sockaddr *)&to, sizeof(to));
  }

 private:
  DatagramSocket *s;
  struct sockaddr_in to;
  struct sockaddr_in from;
  unsigned int from_len;
};


class DatagramSocketQAMain : public SignalHandler
{
 public:
  DatagramSocketQAMain()
  {
    s = new DatagramServerThread(1910, 1911);
    c = new DatagramClientThread(1911, 1910);
  }

  ~DatagramSocketQAMain()
  {
    delete s;
    delete c;
  }


  virtual void handle_signal(int signum)
  {
    printf("Signal received, cancelling threads\n");
    s->cancel();
    c->cancel();
    printf("Threads cancelled\n");
  }

  void run()
  {
    s->start();
    c->start();
    s->join();
    c->join();
  }

 private:
  DatagramServerThread *s;
  DatagramClientThread *c;

};

int
main(int argc, char **argv)
{
  DatagramSocketQAMain m;
  SignalManager::register_handler(SIGINT, &m);
  SignalManager::ignore(SIGPIPE);

  m.run();
}

/// @endcond
