
/***************************************************************************
 *  shm_registry.cpp - shared memory registry
 *
 *  Created: Sun Mar 06 12:08:09 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <utils/ipc/shm_registry.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace fawkes {

/** @class SharedMemoryRegistry <utils/ipc/shm_registry.h>
 * Shared memory registry.
 * This class opens a named POSIX shared memory segment, which
 * contains one instance of the MemInfo struct. It is used to detect
 * and maintain existing SysV IPC shared memory segments in a platform
 * independent way. SysV IPC shared memory segments have some advanced
 * functionality, for example reporting how many processes have
 * attached to the segment. For the registry however, we are more
 * interested in using a symbolic name which is the same for registry
 * entries. Therefore, we use this here. The struct is protected by a
 * lock implemented as a semaphore. Whenever a shared memory segment
 * is created, it is registered to the registry so others can find
 * it. On destruction, it is unregistered from the registry.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name name of the shared memory region. Must follow the rules
 * set by shm_open(). If NULL defaults to "/fawkes-shmem-registry".
 */
SharedMemoryRegistry::SharedMemoryRegistry(const char *name)
{
	shm_name_ = name ? strdup(name) : strdup(DEFAULT_SHM_NAME);

	sem_ = sem_open(shm_name_, O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP, 1);

	if (sem_ == SEM_FAILED) {
		free(shm_name_);
		throw Exception(errno, "Failed to init shared memory registry semaphore");
	}

	sem_wait(sem_);

	shmfd_ = shm_open(shm_name_, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);

	bool created = false;

	if ((shmfd_ < 0) && (errno == EEXIST)) {
		shmfd_ = shm_open(shm_name_, O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
	} else {
		if (ftruncate(shmfd_, sizeof(MemInfo)) != 0) {
			close(shmfd_);
			shm_unlink(shm_name_);
			sem_post(sem_);
			sem_close(sem_);
			sem_unlink(shm_name_);
			free(shm_name_);
			throw Exception(errno, "Failed to resize memory for shared memory registry");
		}

		created = true;
	}

	if (shmfd_ < 0) {
		sem_post(sem_);
		sem_close(sem_);
		sem_unlink(shm_name_);
		free(shm_name_);
		throw Exception(errno, "Failed to open shared memory registry");
	}

	meminfo_ = (MemInfo *)mmap(NULL, sizeof(MemInfo), PROT_READ | PROT_WRITE, MAP_SHARED, shmfd_, 0);
	if (meminfo_ == MAP_FAILED) {
		close(shmfd_);
		sem_close(sem_);
		free(shm_name_);
		throw Exception(errno, "Failed to mmap shared memory registry");
	}

	if (created) {
		memset(meminfo_, 0, sizeof(MemInfo));

		for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
			meminfo_->segments[i].shmid = -1;
		}
	}

	master_ = created;

	sem_post(sem_);
}

/** Destructor. */
SharedMemoryRegistry::~SharedMemoryRegistry()
{
	close(shmfd_);
	sem_close(sem_);
	if (master_) {
		shm_unlink(shm_name_);
	}

	free(shm_name_);
}

/** Cleanup existing shared memory segments.
 * @param name shared memory segment name
 */
void
SharedMemoryRegistry::cleanup(const char *name)
{
	shm_unlink(name ? name : DEFAULT_SHM_NAME);
	sem_unlink(name ? name : DEFAULT_SHM_NAME);
}

/** Get a snapshot of currently registered segments.
 * @return list of all currently registered segments
 */
std::list<SharedMemoryRegistry::SharedMemID>
SharedMemoryRegistry::get_snapshot() const
{
	std::list<SharedMemID> rv;

	sem_wait(sem_);

	for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
		if (meminfo_->segments[i].shmid > 0) {
			rv.push_back(meminfo_->segments[i]);
		}
	}

	sem_post(sem_);

	return rv;
}

/** Find segments with particular magic token.
 * @param magic_token magic token to return IDs for
 * @return list of segments that currently exist with the given
 * magic token
 */
std::list<SharedMemoryRegistry::SharedMemID>
SharedMemoryRegistry::find_segments(const char *magic_token) const
{
	std::list<SharedMemID> rv;

	sem_wait(sem_);

	for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
		if ((meminfo_->segments[i].shmid > 0)
		    && (strncmp(magic_token, meminfo_->segments[i].magic_token, MAGIC_TOKEN_SIZE) == 0)) {
			rv.push_back(meminfo_->segments[i]);
		}
	}

	sem_post(sem_);

	return rv;
}

/** Register a segment.
 * @param shmid shared memory ID of the SysV IPC segment
 * @param magic_token magic token for the new segment
 */
void
SharedMemoryRegistry::add_segment(int shmid, const char *magic_token)
{
	sem_wait(sem_);

	bool valid = false;
	for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
		if (meminfo_->segments[i].shmid == shmid) {
			valid = true;
			break;
		}
	}

	for (unsigned int i = 0; !valid && i < MAXNUM_SHM_SEGMS; ++i) {
		if (meminfo_->segments[i].shmid == -1) {
			meminfo_->segments[i].shmid = shmid;
			strncpy(meminfo_->segments[i].magic_token, magic_token, MAGIC_TOKEN_SIZE);
			valid = true;
		}
	}

	sem_post(sem_);

	if (!valid) {
		throw Exception("Maximum number of shared memory segments already registered");
	}
}

/** Remove segment.
 * @param shmid shared memory ID of the segment to remove.
 */
void
SharedMemoryRegistry::remove_segment(int shmid)
{
	sem_wait(sem_);

	for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
		if (meminfo_->segments[i].shmid == shmid) {
			meminfo_->segments[i].shmid = -1;
		}
	}

	sem_post(sem_);
}

} // end namespace fawkes
