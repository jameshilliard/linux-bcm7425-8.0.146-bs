#ifndef _FS_COREDUMP_H
#define _FS_COREDUMP_H

extern int __get_dumpable(unsigned long mm_flags);
extern int coredump_wait(int exit_code, struct core_state *core_state);
extern void coredump_finish(struct mm_struct *mm, bool core_dumped);

#endif
