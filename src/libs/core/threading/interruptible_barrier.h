
/***************************************************************************
 *  interruptible_barrier.h - Interruptible Barrier
 *
 *  Created: Sat Jan 31 12:27:54 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef _CORE_THREADING_INTERRUPTIBLE_BARRIER_H_
#define _CORE_THREADING_INTERRUPTIBLE_BARRIER_H_

#include <core/threading/barrier.h>
#include <core/utils/refptr.h>

namespace fawkes {

class InterruptibleBarrierData;
class ThreadList;

class InterruptibleBarrier : public Barrier
{
public:
	InterruptibleBarrier(unsigned int count);
	InterruptibleBarrier(Mutex *mutex, unsigned int count);
	virtual ~InterruptibleBarrier();

	bool wait(unsigned int timeout_sec, unsigned int timeout_nanosec);
	virtual inline void
	wait()
	{
		wait(0, 0);
	}

	void interrupt() noexcept;
	void reset() noexcept;

	RefPtr<ThreadList> passed_threads();

	bool no_threads_in_wait();

private:
	InterruptibleBarrier(const InterruptibleBarrier &b);
	InterruptibleBarrier(const InterruptibleBarrier *b);
	InterruptibleBarrier &operator=(const InterruptibleBarrier &b);
	InterruptibleBarrier &operator=(const InterruptibleBarrier *b);

private:
	InterruptibleBarrierData *data_;
	RefPtr<ThreadList>        passed_threads_;

	bool interrupted_;
	bool timeout_;
	bool wait_at_barrier_;
	int  num_threads_in_wait_function_;
};

} // end namespace fawkes

#endif
