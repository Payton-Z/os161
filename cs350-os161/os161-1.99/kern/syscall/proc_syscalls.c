#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include "opt-A2.h"
#include <mips/trapframe.h>
#include <vfs.h>
#include <kern/fcntl.h>
#include <synch.h>
#if OPT_A2


int sys_execv(const char *program, char **args) {
	struct addrspace *as;
	struct vnode *v;
	vaddr_t entrypoint, stackptr;
	int result;
	int args_count = 0;
	//# of args
	while (args[args_count] != NULL) {
		++args_count;
	}

	size_t name_len = (strlen(program) + 1) * sizeof(char); //size of program name
	char* program_kernel = kmalloc(name_len);      //allocate memory for the kernel
	KASSERT(program_kernel != NULL);

	int i = copyin((const_userptr_t) program, (void *) program_kernel, name_len); 
	KASSERT(i == 0);

	char** kernel_args = kmalloc((args_count + 1) * sizeof(char*)); //allocate mem for the kernel
	KASSERT(kernel_args != NULL);
	
	kernel_args[args_count] = NULL;
	for (int i = 0; i < args_count; i++) {
		size_t size = (strlen(args[i]) + 1) * sizeof(char);
		
		kernel_args[i] = kmalloc(size); 
		KASSERT(kernel_args[i] != NULL);
		
		int j = copyin((const_userptr_t) args[i], (void *) kernel_args[i], size);
		KASSERT(j == 0);
	}


	/* Open the file. */
	result = vfs_open(program_kernel, O_RDONLY, 0, &v);
	if (result) {
		return result;
	}

	/* Create a new address space. */
	as = as_create();
	if (as == NULL) {
		vfs_close(v);
		return ENOMEM;
	}

	/* Switch to it and activate it. */
	struct addrspace* old_as = curproc_setas(as);
	as_activate();

	/* Load the executable. */
	result = load_elf(v, &entrypoint);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		vfs_close(v);
		return result;
	}

	/* Done with the file now. */
	vfs_close(v);

	/* Define the user stack in the address space */
	result = as_define_stack(as, &stackptr);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		return result;
	}

	vaddr_t stack_ptr = stackptr;
	vaddr_t* stack_args = kmalloc((args_count + 1) * sizeof(vaddr_t));
	
	stack_args[args_count] = (vaddr_t) NULL; //set the end to null
	for (int i = args_count - 1; i >= 0; i--) {
		size_t len = ROUNDUP(strlen(kernel_args[i]) + 1, 4);
		size_t size = len * sizeof(char);
		stack_ptr -= size;
		int res = copyout((void *) kernel_args[i], (userptr_t) stack_ptr, len);
		KASSERT(res == 0);
		stack_args[i] = stack_ptr;
	}

	for (int i = args_count; i >= 0; i--) {
		stack_ptr -= sizeof(vaddr_t);
		int res = copyout((void *) &stack_args[i], (userptr_t) stack_ptr, sizeof(vaddr_t));
		KASSERT(res == 0);
	}

	//free
	as_destroy(old_as);
	kfree(program_kernel);
	for (int i = 0; i <= args_count; i++) {
		kfree(kernel_args[i]);
	}
	kfree(kernel_args);

	/* Warp to user mode. */
	enter_new_process(args_count, (userptr_t) stack_ptr, ROUNDUP(stack_ptr, 8), entrypoint);

	/* enter_new_process does not return. */
	panic("enter_new_process returned\n");
	return EINVAL;
}

int sys_fork(struct trapframe *tf, pid_t *retval) {
	struct proc *child = proc_create_runprogram(curproc->p_name);
	KASSERT(child != NULL);
	child->parent = curproc;  //child's parent is the current proc

	if (as_copy(curproc_getas(), &(child->p_addrspace)) != 0) { //it returns 0 if no error
		panic("error occurred when copying address space");
	}
	//copy the addrspace from parent
	struct Child *c = kmalloc(sizeof(struct Child));
	c->p_addr = child;
	c->exit_status = -1;
	c->pid = child->pid;

	//add the child to the array of children
	array_add(curproc->children, c, NULL);

	curproc->tf = kmalloc(sizeof(struct trapframe));
	if (curproc->tf == NULL) {
		panic("couldn't create trapfraME");
	}
	memcpy(curproc->tf, tf, sizeof(struct trapframe));
	thread_fork(child->p_name, child, (void *)&enter_forked_process, curproc->tf, 15);
	*retval = child->pid;
	return 0;

}

#endif


/* this implementation of sys__exit does not do anything with the exit code */
/* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

	struct addrspace *as;
	struct proc *p = curproc;
#if OPT_A2
	if (curproc->parent != NULL) {
		lock_acquire(curproc->parent->child_lk);
		for (unsigned int i = 0; i < array_num(curproc->parent->children); i++) {
			struct Child *child = array_get(curproc->parent->children, i);
			if (curproc->pid == child->pid) {
				child->exit_status = exitcode;
				break;
			}
		}
		lock_release(curproc->parent->child_lk);
		cv_signal(curproc->cv, curproc->child_lk);
	}
#else
	(void)exitcode;
#endif
	DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

	KASSERT(curproc->p_addrspace != NULL);
	as = curproc_setas(NULL);
	as_destroy(as);

	proc_remthread(curthread);
	proc_destroy(p);

	thread_exit();
	panic("return from thread_exit in sys_exit\n");
}


/* handler for getpid() system call                */
int sys_getpid(pid_t *retval) {
#if OPT_A2 
	*retval = curproc->pid;
#else 
	*retval = 1;
#endif 
	return 0;
}

/* handler for waitpid() system call                */

int sys_waitpid(pid_t pid, userptr_t status, int options, pid_t *retval) {
	int exitstatus;
	int result;
#if OPT_A2 
	lock_acquire(curproc->child_lk);
	bool child_exist = false;
	for (unsigned int i = 0; i < array_num(curproc->children); i++) {
		struct Child *child = array_get(curproc->children, i);
		if (pid == child->pid) {
			child_exist = true;
			struct Child *child2 = array_get(curproc->children, i);
			while (child2->exit_status == -1) {
				cv_wait(child2->p_addr->cv, curproc->child_lk);
			}
			exitstatus = _MKWAIT_EXIT(child2->exit_status);
		}
	}
	if (!child_exist) {
		lock_release(curproc->child_lk);
		*retval = -1;
		return (ESRCH);
		//no such process
	}
	lock_release(curproc->child_lk);
#else

	/* this is just a stub implementation that always reports an
	   exit status of 0, regardless of the actual exit status of
	   the specified process.   
	   In fact, this will return 0 even if the specified process
	   is still running, and even if it never existed in the first place.

	   Fix this!
	 */

	exitstatus = 0;

#endif
	if (options != 0) {
		return(EINVAL);
	}
	/* for now, just pretend the exitstatus is 0 */
	result = copyout((void *)&exitstatus,status,sizeof(int));
	if (result) {
		return(result);
	}
	*retval = pid;
	return(0);

}

