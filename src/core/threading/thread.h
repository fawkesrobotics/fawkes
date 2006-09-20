
/***************************************************************************
 *  thread.h - base class for threads, implementation based on pthreads
 *
 *  Generated: Thu Sep 14 13:06:18 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __CORE_THREADING_THREAD_H_
#define __CORE_THREADING_THREAD_H_

#define forever while (1)

class Thread {
 public:

  virtual ~Thread();

  bool start();
  void cancel();
  void join();
  void detach();

  bool operator==(const Thread &thread);

 protected:
  void exit();

  void test_cancel();

  virtual void run();

  virtual void loop();

 private:
  static void * entry(void * pthis);

  // Do not use pthread_t here to avoid including pthread.h
  /* pthread_t */ unsigned long int thread_id;

};



#endif
