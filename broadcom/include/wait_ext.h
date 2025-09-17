#ifndef WAIT_EXT_H
#define WAIT_EXT_H

#include <linux/wait.h>
#include <linux/mutex.h>

/*
 * These wait helpers below are copied from newer verisions of linux kernel 
 * to provide supports for some useful wait_event_xxx cases
 */

#ifndef __wait_event_killable_cmd

#define __wait_event_killable_cmd(wq, condition, ret, cmd)      \
do {                  \
  DEFINE_WAIT(__wait);            \
                  \
  for (;;) {              \
    prepare_to_wait(&wq, &__wait, TASK_KILLABLE);    \
    if (condition)            \
      break;            \
    if (!fatal_signal_pending(current)) {      \
      cmd;         \
      schedule();          \
      continue;          \
    }              \
    ret = -ERESTARTSYS;          \
    break;              \
  }                \
  finish_wait(&wq, &__wait);          \
} while (0)

#endif // __wait_event_killable_cmd

#ifndef wait_event_killable_cmd

#define wait_event_killable_cmd(wq, condition, cmd) \
({ \
    int __ret = 0; \
    if (!(condition)) \
        __wait_event_killable_cmd(wq, condition, __ret, cmd); \
    __ret; \
})

#endif // wait_event_killable_cmd

#ifndef __wait_event_locked_timeout

#define __wait_event_locked_timeout(wq, condition, ret)    \
do {                  \
  DEFINE_WAIT(__wait);            \
                  \
  for (;;) {              \
		if (likely(list_empty(&__wait.task_list)))		\
			__add_wait_queue_tail(&(wq), &__wait);		\
		set_current_state(TASK_UNINTERRUPTIBLE);			\
    if (condition)            \
      break;            \
    spin_unlock(&(wq).lock);          \
    ret = schedule_timeout(ret);        \
    spin_lock(&(wq).lock);          \
    if (!ret)            \
      break;            \
  }                \
  if (!ret && (condition))          \
    ret = 1;            \
	__set_current_state(TASK_RUNNING);				\
	__remove_wait_queue(&(wq), &__wait);				\
} while (0)

#endif // __wait_event_locked_timeout

#ifndef wait_event_locked_timeout

#define wait_event_locked_timeout(wq, condition, timeout)    \
({                  \
  unsigned long __ret = timeout;            \
  if (!(condition))            \
    __wait_event_locked_timeout(    \
          wq, condition, __ret);  \
  __ret;                \
})

#endif // wait_event_locked_timeout

#endif // WAIT_EXT_H
