/* auto-generated by gen_syscalls.py, don't edit */
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic push
#endif
#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
#include <syscalls/log_msg2.h>

extern void z_vrfy_z_log_msg2_runtime_vcreate(uint8_t domain_id, const void * source, uint8_t level, const void * data, size_t dlen, const char * fmt, va_list ap);
uintptr_t z_mrsh_z_log_msg2_runtime_vcreate(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, void *more, void *ssf)
{
	_current->syscall_frame = ssf;
	Z_OOPS(Z_SYSCALL_MEMORY_READ(more, 1 * sizeof(uintptr_t)));
	z_vrfy_z_log_msg2_runtime_vcreate(*(uint8_t*)&arg0, *(const void **)&arg1, *(uint8_t*)&arg2, *(const void **)&arg3, *(size_t*)&arg4, *(const char **)&(((uintptr_t *)more)[0]), *(va_list*)&(((uintptr_t *)more)[1]))
;
	_current->syscall_frame = NULL;
	return 0;
}

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic pop
#endif
