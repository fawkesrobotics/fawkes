
/***************************************************************************
 *  qa_ipc_shmem_lowlevel.cpp - lowlevel shared memory qa
 *
 *  Generated: Sun Oct 22 23:43:36 2006
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

/* This program reveals a problem with the shmat shmaddr parameter. It shows
 * that this cannot be reliably used to map the shared memory to a specific
 * address even if the REMAP flag has been set. Maybe this just shows a fundamental
 * misunderstanding on my side. Have to study more literature, kernel source did
 * not reveal the problem in an obvious manner to me.
 */

#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <iostream>
#include <signal.h>

using namespace std;
using namespace fawkes;

#define SHMEM_SIZE 2048
#define SHMEM_TOKEN "JustSomeDumbQA"

typedef struct {
  void *ptr;
} header_t;

bool quit = false;

void
signal_handler(int signum)
{
  quit = true;
}

int
main(int argc, char **argv)
{

  signal(SIGINT, signal_handler);

  key_t key = ftok(".", 'b');
  printf("Key: 0x%x\n", key);

  if ( argc == 1 ) {
    // master
    int shmid = shmget(key, SHMEM_SIZE, IPC_CREAT | 0666);
    if ( shmid == -1 ) {
      perror("M: Could not get ID");
      exit(1);
    }
      
    void *shmem = shmat(shmid, NULL, 0);
    if ( shmem == (void *)-1 ) {
      perror("M: Could not attach");
      exit(2);
    }

    memset(shmem, 0, SHMEM_SIZE);

    header_t *header = (header_t *)shmem;
    header->ptr = shmem;

    printf("M: ptr=0x%lx\n", (long unsigned int)shmem);

    while ( ! quit ) {
      usleep(100000);
    }

    shmctl(shmid, IPC_RMID, NULL);
    shmdt(shmem);

  } else {
    // slave
    int shmid = shmget(key, SHMEM_SIZE, 0);
    
    if ( shmid == -1 ) {
      perror("S: Could not get ID");
      exit(1);
    }
      
    void *shmem = shmat(shmid, NULL, 0);
    if ( shmem == (void *)-1 ) {
      perror("S: Could not attach");
      exit(2);
    }

    header_t *header = (header_t *)shmem;

    printf("S: ptr=0x%lx   header->ptr=0x%lx\n", (long unsigned int)shmem,
	   (long unsigned int)header->ptr);

    if ( shmem != header->ptr ) {
      printf("S: pointers differ, re-attaching\n");
      void *ptr = header->ptr;
      shmdt(shmem);
      shmem = shmat(shmid, ptr, SHM_REMAP);
      if ( shmem == (void *)-1 ) {
	perror("S: Could not re-attach");
	exit(3);
      }
      header = (header_t *)shmem;
      printf("S: after re-attach: ptr=0x%lx   header->ptr=0x%lx\n",
	     (long unsigned int)shmem, (long unsigned int)header->ptr);
    }

    /*
    while ( ! quit ) {
      usleep(100000);
    }
    */

    shmdt(shmem);
  }

  return 0;
}

/// @endcond
