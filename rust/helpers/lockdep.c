// SPDX-License-Identifier: GPL-2.0

#include <linux/instruction_pointer.h>
#include <linux/lockdep.h>

void rust_helper_lock_acquire_ret(struct lockdep_map *lock, unsigned int subclass,
				  int trylock, int read, int check,
				  struct lockdep_map *nest_lock)
{
	lock_acquire(lock, subclass, trylock, read, check, nest_lock, _RET_IP_);
}

void rust_helper_lock_release_ret(struct lockdep_map *lock)
{
	lock_release(lock, _RET_IP_);
}
