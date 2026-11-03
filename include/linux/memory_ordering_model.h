/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_MEMORY_ORDERING_MODEL_H
#define __ASM_MEMORY_ORDERING_MODEL_H

/* Arch hooks to implement the PR_{GET_SET}_MEM_MODEL prctls */

struct task_struct;
int arch_prctl_mem_model_get(struct task_struct *t);
int arch_prctl_mem_model_set(struct task_struct *t, unsigned long val);

#endif
