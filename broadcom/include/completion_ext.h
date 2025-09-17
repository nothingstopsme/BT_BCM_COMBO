#ifndef COMPLETION_EXT_H
#define COMPLETION_EXT_H

#include <linux/completion.h>

/*
 * Older versions of linux kernel define INIT_COMPLETION() as opposed to reinit_completion();
 * the macro below makes sure reinit_completion() also exists
 */ 
#ifdef INIT_COMPLETION

#define reinit_completion(x) INIT_COMPLETION(*(x))

#endif // INIT_COMPLETION

#endif // COMPLETION_EXT_H
