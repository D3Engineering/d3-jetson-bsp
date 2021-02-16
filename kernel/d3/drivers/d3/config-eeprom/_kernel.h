#ifndef _LINUX_KERNEL_H
#define _LINUX_KERNEL_H


#include <stdarg.h>
#include <linux/stddef.h>
#include <linux/types.h>


#ifdef __CHECKER__
#define __must_be_array(a)	0
#else
/* &a[0] degrades to a pointer: a different type from an array */
#define __must_be_array(a)	BUILD_BUG_ON_ZERO(__same_type((a), &(a)[0]))
#endif


/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})


/* Rebuild everything on CONFIG_FTRACE_MCOUNT_RECORD */
#ifdef CONFIG_FTRACE_MCOUNT_RECORD
# define REBUILD_DUE_TO_FTRACE_MCOUNT_RECORD
#endif

/* Permissions on a sysfs file: you didn't miss the 0 prefix did you? */
#define VERIFY_OCTAL_PERMISSIONS(perms)						\
	(BUILD_BUG_ON_ZERO((perms) < 0) +					\
	 BUILD_BUG_ON_ZERO((perms) > 0777) +					\
	 /* USER_READABLE >= GROUP_READABLE >= OTHER_READABLE */		\
	 BUILD_BUG_ON_ZERO((((perms) >> 6) & 4) < (((perms) >> 3) & 4)) +	\
	 BUILD_BUG_ON_ZERO((((perms) >> 3) & 4) < ((perms) & 4)) +		\
	 /* USER_WRITABLE >= GROUP_WRITABLE */					\
	 BUILD_BUG_ON_ZERO((((perms) >> 6) & 2) < (((perms) >> 3) & 2)) +	\
	 /* OTHER_WRITABLE?  Generally considered a bad idea. */		\
	 BUILD_BUG_ON_ZERO((perms) & 2) +					\
	 (perms))
#endif
