#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

struct cpu cpus[NCPU];

struct proc proc[NPROC];

// initial scheduling policy is shortest-job-first
struct sched_policy proc_sched = { .heap_size = 0, .a = 50, .algorithm = 0, .is_preemptive = 0};

struct proc *initproc;

int nextpid = 1;
struct spinlock pid_lock;

extern void forkret(void);
static void freeproc(struct proc *p);

extern char trampoline[]; // trampoline.S

// helps ensure that wakeups of wait()ing
// parents are not lost. helps obey the
// memory model when using p->parent.
// must be acquired before any p->lock.
struct spinlock wait_lock;

// Allocate a page for each process's kernel stack.
// Map it high in memory, followed by an invalid
// guard page.
void
proc_mapstacks(pagetable_t kpgtbl) {
  struct proc *p;
  
  for(p = proc; p < &proc[NPROC]; p++) {
    char *pa = kalloc();
    if(pa == 0)
      panic("kalloc");
    uint64 va = KSTACK((int) (p - proc));
    kvmmap(kpgtbl, va, (uint64)pa, PGSIZE, PTE_R | PTE_W);
  }
}

// initialize the proc table at boot time.
void
procinit(void)
{
  struct proc *p;
  
  initlock(&pid_lock, "nextpid");
  initlock(&wait_lock, "wait_lock");
  for(p = proc; p < &proc[NPROC]; p++) {
      initlock(&p->lock, "proc");
      p->kstack = KSTACK((int) (p - proc));
  }
}

// Must be called with interrupts disabled,
// to prevent race with process being moved
// to a different CPU.
int
cpuid()
{
  int id = r_tp();
  return id;
}

// Return this CPU's cpu struct.
// Interrupts must be disabled.
struct cpu*
mycpu(void) {
  int id = cpuid();
  struct cpu *c = &cpus[id];
  return c;
}

// Return the current struct proc *, or zero if none.
struct proc*
myproc(void) {
  push_off();
  struct cpu *c = mycpu();
  struct proc *p = c->proc;
  pop_off();
  return p;
}

int
allocpid() {
  int pid;
  
  acquire(&pid_lock);
  pid = nextpid;
  nextpid = nextpid + 1;
  release(&pid_lock);

  return pid;
}

// Look in the process table for an UNUSED proc.
// If found, initialize state required to run in the kernel,
// and return with p->lock held.
// If there are no free procs, or a memory allocation fails, return 0.
static struct proc*
allocproc(void)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if(p->state == UNUSED) {
      goto found;
    } else {
      release(&p->lock);
    }
  }
  return 0;

found:
  p->pid = allocpid();
  p->state = USED;
  p->cpu_burst_aprox = 0;
  p->cpu_burst = 0;
  p->timeslice = 0;
  p->put_timestamp = 0;
  p->exe_time = 0;

  // Allocate a trapframe page.
  if((p->trapframe = (struct trapframe *)kalloc()) == 0){
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // An empty user page table.
  p->pagetable = proc_pagetable(p);
  if(p->pagetable == 0){
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // Set up new context to start executing at forkret,
  // which returns to user space.
  memset(&p->context, 0, sizeof(p->context));
  p->context.ra = (uint64)forkret;
  p->context.sp = p->kstack + PGSIZE;

  return p;
}

// free a proc structure and the data hanging from it,
// including user pages.
// p->lock must be held.
static void
freeproc(struct proc *p)
{
  if(p->trapframe)
    kfree((void*)p->trapframe);
  p->trapframe = 0;
  if(p->pagetable)
    proc_freepagetable(p->pagetable, p->sz);
  p->pagetable = 0;
  p->sz = 0;
  p->pid = 0;
  p->parent = 0;
  p->name[0] = 0;
  p->chan = 0;
  p->killed = 0;
  p->xstate = 0;
  p->state = UNUSED;
  p->cpu_burst_aprox = 0;
  p->cpu_burst = 0;
  p->timeslice = 0;
  p->put_timestamp = 0;
  p->exe_time = 0;
}

// Create a user page table for a given process,
// with no user memory, but with trampoline pages.
pagetable_t
proc_pagetable(struct proc *p)
{
  pagetable_t pagetable;

  // An empty page table.
  pagetable = uvmcreate();
  if(pagetable == 0)
    return 0;

  // map the trampoline code (for system call return)
  // at the highest user virtual address.
  // only the supervisor uses it, on the way
  // to/from user space, so not PTE_U.
  if(mappages(pagetable, TRAMPOLINE, PGSIZE,
              (uint64)trampoline, PTE_R | PTE_X) < 0){
    uvmfree(pagetable, 0);
    return 0;
  }

  // map the trapframe just below TRAMPOLINE, for trampoline.S.
  if(mappages(pagetable, TRAPFRAME, PGSIZE,
              (uint64)(p->trapframe), PTE_R | PTE_W) < 0){
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmfree(pagetable, 0);
    return 0;
  }

  return pagetable;
}

// Free a process's page table, and free the
// physical memory it refers to.
void
proc_freepagetable(pagetable_t pagetable, uint64 sz)
{
  uvmunmap(pagetable, TRAMPOLINE, 1, 0);
  uvmunmap(pagetable, TRAPFRAME, 1, 0);
  uvmfree(pagetable, sz);
}

// a user program that calls exec("/init")
// od -t xC initcode
uchar initcode[] = {
  0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x45, 0x02,
  0x97, 0x05, 0x00, 0x00, 0x93, 0x85, 0x35, 0x02,
  0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00,
  0x93, 0x08, 0x20, 0x00, 0x73, 0x00, 0x00, 0x00,
  0xef, 0xf0, 0x9f, 0xff, 0x2f, 0x69, 0x6e, 0x69,
  0x74, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

// Set up first user process.
void
userinit(void)
{
  struct proc *p;

  p = allocproc();
  initproc = p;
  
  // allocate one user page and copy init's instructions
  // and data into it.
  uvminit(p->pagetable, initcode, sizeof(initcode));
  p->sz = PGSIZE;

  // prepare for the very first "return" from kernel to user.
  p->trapframe->epc = 0;      // user program counter
  p->trapframe->sp = PGSIZE;  // user stack pointer

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  //p->state = RUNNABLE;
  put(p);

  release(&p->lock);
}

// Grow or shrink user memory by n bytes.
// Return 0 on success, -1 on failure.
int
growproc(int n)
{
  uint sz;
  struct proc *p = myproc();

  sz = p->sz;
  if(n > 0){
    if((sz = uvmalloc(p->pagetable, sz, sz + n)) == 0) {
      return -1;
    }
  } else if(n < 0){
    sz = uvmdealloc(p->pagetable, sz, sz + n);
  }
  p->sz = sz;
  return 0;
}

// Create a new process, copying the parent.
// Sets up child kernel stack to return as if from fork() system call.
int
fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *p = myproc();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }

  // Copy user memory from parent to child.
  if(uvmcopy(p->pagetable, np->pagetable, p->sz) < 0){
    freeproc(np);
    release(&np->lock);
    return -1;
  }
  np->sz = p->sz;

  // copy saved user registers.
  *(np->trapframe) = *(p->trapframe);

  // Cause fork to return 0 in the child.
  np->trapframe->a0 = 0;

  // increment reference counts on open file descriptors.
  for(i = 0; i < NOFILE; i++)
    if(p->ofile[i])
      np->ofile[i] = filedup(p->ofile[i]);
  np->cwd = idup(p->cwd);

  safestrcpy(np->name, p->name, sizeof(p->name));

  pid = np->pid;

  release(&np->lock);

  acquire(&wait_lock);
  np->parent = p;
  release(&wait_lock);

  /*
  acquire(&np->lock);
  np->state = RUNNABLE;
  release(&np->lock);
   */
  put(np);

  return pid;
}

// Pass p's abandoned children to init.
// Caller must hold wait_lock.
void
reparent(struct proc *p)
{
  struct proc *pp;

  for(pp = proc; pp < &proc[NPROC]; pp++){
    if(pp->parent == p){
      pp->parent = initproc;
      wakeup(initproc);
    }
  }
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait().
void
exit(int status)
{
  struct proc *p = myproc();

  if(p == initproc)
    panic("init exiting");

  // Close all open files.
  for(int fd = 0; fd < NOFILE; fd++){
    if(p->ofile[fd]){
      struct file *f = p->ofile[fd];
      fileclose(f);
      p->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(p->cwd);
  end_op();
  p->cwd = 0;

  acquire(&wait_lock);

  // Give any children to init.
  reparent(p);

  // Parent might be sleeping in wait().
  wakeup(p->parent);
  
  acquire(&p->lock);

  p->xstate = status;
  p->state = ZOMBIE;

  release(&wait_lock);

  // Jump into the sched_policy, never to return.
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
wait(uint64 addr)
{
  struct proc *np;
  int havekids, pid;
  struct proc *p = myproc();

  acquire(&wait_lock);

  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(np = proc; np < &proc[NPROC]; np++){
      if(np->parent == p){
        // make sure the child isn't still in exit() or swtch().
        acquire(&np->lock);

        havekids = 1;
        if(np->state == ZOMBIE){
          // Found one.
          pid = np->pid;
          if(addr != 0 && copyout(p->pagetable, addr, (char *)&np->xstate,
                                  sizeof(np->xstate)) < 0) {
            release(&np->lock);
            release(&wait_lock);
            return -1;
          }
          freeproc(np);
          release(&np->lock);
          release(&wait_lock);
          return pid;
        }
        release(&np->lock);
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || p->killed){
      release(&wait_lock);
      return -1;
    }
    
    // Wait for a child to exit.
    sleep(p, &wait_lock);  //DOC: wait-sleep
  }
}

// Per-CPU process sched_policy.
// Each CPU calls sched_policy() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run.
//  - swtch to start running that process.
//  - eventually that process transfers control
//    via swtch back to the sched_policy.
void
scheduler(void)
{
  struct proc *p;
  struct cpu *c = mycpu();
  
  c->proc = 0;
  for(;;) {
      // Avoid deadlock by ensuring that devices can interrupt.
      intr_on();

      /*
      for(p = proc; p < &proc[NPROC]; p++) {
        acquire(&p->lock);
        if(p->state == RUNNABLE) {
          // Switch to chosen process.  It is the process's job
          // to release its lock and then reacquire it
          // before jumping back to us.
          p->state = RUNNING;
          c->proc = p;
          swtch(&c->context, &p->context);

          // Process is done running for now.
          // It should have changed its p->state before coming back.
          c->proc = 0;
        }
        release(&p->lock);
      }
      */

      p = get();
      if (p != 0) {
          acquire(&p->lock);
          if (p->state == RUNNABLE) {
              p->state = RUNNING;
              c->proc = p;
              swtch(&c->context, &p->context);

              if (c->proc != 0 && c->proc->state == RUNNABLE) put(c->proc);
              c->proc = 0;
          }
          release(&p->lock);
      }
  }
}

// Switch to sched_policy.  Must hold only p->lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->noff, but that would
// break in the few places where a lock is held but
// there's no process.
void
sched(void)
{
  int intena;
  struct proc *p = myproc();

  if(!holding(&p->lock))
    panic("sched p->lock");
  if(mycpu()->noff != 1)
    panic("sched locks");
  if(p->state == RUNNING)
    panic("sched running");
  if(intr_get())
    panic("sched interruptible");

  intena = mycpu()->intena;
  swtch(&p->context, &mycpu()->context);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  struct proc *p = myproc();
  acquire(&p->lock);
  //p->state = RUNNABLE;
  put(p);
  sched();
  release(&p->lock);
}

// A fork child's very first scheduling by sched_policy()
// will swtch to forkret.
void
forkret(void)
{
  static int first = 1;

  // Still holding p->lock from sched_policy.
  release(&myproc()->lock);

  if (first) {
    // File system initialization must be run in the context of a
    // regular process (e.g., because it calls sleep), and thus cannot
    // be run from main().
    first = 0;
    fsinit(ROOTDEV);
  }

  usertrapret();
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
void
sleep(void *chan, struct spinlock *lk)
{
  struct proc *p = myproc();
  
  // Must acquire p->lock in order to
  // change p->state and then call sched.
  // Once we hold p->lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup locks p->lock),
  // so it's okay to release lk.

  acquire(&p->lock);  //DOC: sleeplock1
  release(lk);

  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;

  sched();

  // Tidy up.
  p->chan = 0;

  // Reacquire original lock.
  release(&p->lock);
  acquire(lk);
}

// Wake up all processes sleeping on chan.
// Must be called without any p->lock.
void
wakeup(void *chan)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    if(p != myproc()){
      acquire(&p->lock);
      if(p->state == SLEEPING && p->chan == chan) {
        //p->state = RUNNABLE;
        put(p);
      }
      release(&p->lock);
    }
  }
}

// Kill the process with the given pid.
// The victim won't exit until it tries to return
// to user space (see usertrap() in trap.c).
int
kill(int pid)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++){
    acquire(&p->lock);
    if(p->pid == pid){
      p->killed = 1;
      if(p->state == SLEEPING){
        // Wake process from sleep().
        //p->state = RUNNABLE;
        put(p);
      }
      release(&p->lock);
      return 0;
    }
    release(&p->lock);
  }
  return -1;
}

// Copy to either a user address, or kernel address,
// depending on usr_dst.
// Returns 0 on success, -1 on error.
int
either_copyout(int user_dst, uint64 dst, void *src, uint64 len)
{
  struct proc *p = myproc();
  if(user_dst){
    return copyout(p->pagetable, dst, src, len);
  } else {
    memmove((char *)dst, src, len);
    return 0;
  }
}

// Copy from either a user address, or kernel address,
// depending on usr_src.
// Returns 0 on success, -1 on error.
int
either_copyin(void *dst, int user_src, uint64 src, uint64 len)
{
  struct proc *p = myproc();
  if(user_src){
    return copyin(p->pagetable, dst, src, len);
  } else {
    memmove(dst, (char*)src, len);
    return 0;
  }
}

// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void
procdump(void)
{
  static char *states[] = {
  [UNUSED]    "unused",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  struct proc *p;
  char *state;

  printf("\n");
  for(p = proc; p < &proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    printf("%d %s %s", p->pid, state, p->name);
    printf("\n");
  }
}

///////////////////////////

void heapify_up(struct proc** arr, int n, int algo)
{
    if (n == 1) return;
    int curr = n - 1;
    int parent = curr / 2;

    while (1)
    {
        if ((algo == 0 && arr[curr]->cpu_burst_aprox < arr[parent]->cpu_burst_aprox) ||
            (algo != 0 && arr[curr]->exe_time < arr[parent]->exe_time)) {
            struct proc *tmp = arr[curr];
            arr[curr] = arr[parent];
            arr[parent] = tmp;
        }
        else break;
        if (parent == 0) break;
        curr = parent;
        parent = (curr - 1) / 2;
    }
}

void heapify_down(struct proc** arr, int n, int algo)
{
    heapify_down_i(arr,n,0,algo);
}

void heapify_down_i(struct proc** arr, int n, int i, int algo)
{
    if (n == 1) return;
    int curr = i;
    int left_child = curr * 2 + 1;
    int right_child = curr * 2 + 2;

    while(1)
    {
        if ((algo == 0 && left_child < n && arr[curr]->cpu_burst_aprox > arr[left_child]->cpu_burst_aprox) ||
            (algo != 0 && left_child < n && arr[curr]->exe_time > arr[left_child]->exe_time))
        {
            struct proc* tmp = arr[curr];
            arr[curr] = arr[left_child];
            arr[left_child] = tmp;
            curr = left_child;
        }
        else if ((algo == 0 && right_child < n && arr[curr]->cpu_burst_aprox > arr[right_child]->cpu_burst_aprox) ||
                 (algo != 0 && right_child < n && arr[curr]->cpu_burst_aprox > arr[right_child]->cpu_burst_aprox))
        {
            struct proc* tmp = arr[curr];
            arr[curr] = arr[right_child];
            arr[right_child] = tmp;
            curr = right_child;
        }
        else break;
        left_child = curr * 2 + 1;
        right_child = curr * 2 + 2;
    }
}

void put(struct proc *p)
{
    if (p == 0) return;
    int cpu_already_locked_the_lock = 1;
    if(!holding(&p->lock)) // same cpu can't aquire the same lock twice
    {
        acquire(&p->lock);
        cpu_already_locked_the_lock = 0;
    }
    acquire(&proc_sched.lock);
    // critical section

    // exponential averaging
    if (proc->state != RUNNING)
        p->cpu_burst_aprox = (proc_sched.a * p->cpu_burst + (100 - proc_sched.a) * p->cpu_burst_aprox) / 100;

    if (p->state == RUNNING) p->exe_time += p->cpu_burst;
    else p->exe_time = 0;  // when process is suspended or has never been executed on the cpu, exe_time should reset

    p->put_timestamp = ticks;

    p->state = RUNNABLE;

    proc_sched.heap[proc_sched.heap_size] = p;
    proc_sched.heap_size += 1;
    heapify_up((struct proc**) &proc_sched.heap, proc_sched.heap_size, proc_sched.algorithm);

    //printf("put | pid: %d | cpu_burst: %d\n", p->pid, p->cpu_burst);

    // end of critical section
    release(&proc_sched.lock);
    if (!cpu_already_locked_the_lock)
        release(&p->lock);
}

struct proc* get()
{
    struct proc* ret = 0;
    acquire(&proc_sched.lock);

    if (proc_sched.heap_size == 0) goto exit_get;
    ret = proc_sched.heap[0];
    ret->cpu_burst = 0;
    proc_sched.heap[0] = proc_sched.heap[proc_sched.heap_size - 1];
    proc_sched.heap[proc_sched.heap_size - 1] = 0;
    proc_sched.heap_size -= 1;
    heapify_down((struct proc**) &proc_sched.heap, proc_sched.heap_size, proc_sched.algorithm);

    if (proc_sched.algorithm == 1) {
        int cfs_timeslice = (ticks - ret->put_timestamp) / (proc_sched.heap_size + 1); // +1 for zero-division prevention
        if (cfs_timeslice == 0) cfs_timeslice++;
        ret->timeslice = cfs_timeslice;
    }

    exit_get:    release(&proc_sched.lock);
    return ret;
}

//////////////////

void rearrange_heap(struct proc** arr, int n, int algo)
{
    int i_init = n / 2 - 1; // index of last non-leaf element

    for (int i = i_init; i >= 0; i--)
        heapify_down_i(arr, n, i, algo);
}

/*
// when changing the process scheduling policy, we must sort the heap by different criteria
int change_sched(int algo, int is_preemptive, int a){
    acquire(&proc_sched.lock);

    int new_algo = (proc_sched.algorithm == 0 ? 1 : 0);
    proc_sched.algorithm = new_algo;

    rearrange_heap((struct proc **) &proc_sched.heap, proc_sched.heap_size, new_algo);

    release(&proc_sched.lock);
    return new_algo;
}
 */

int change_sched(int algo, int is_preemptive, int a){
    if (algo < 0 || algo > 1 || is_preemptive<0) return -2;
    if (algo == 0 && (a<0 || a>100)) return -3;
    acquire(&proc_sched.lock);

    proc_sched.algorithm = algo;
    proc_sched.is_preemptive = is_preemptive;
    proc_sched.a = a;

    rearrange_heap((struct proc **) &proc_sched.heap, proc_sched.heap_size, algo);

    release(&proc_sched.lock);
    return 0;
}

//////////////////////////

// timer interrupt routine called from trap.c
void timer_routine(struct proc* p)
{
    p->cpu_burst += 1;

    //printf("timer | pid: %d | cpu_burst: %d\n", myproc()->pid, myproc()->cpu_burst);

    if ((p->timeslice != 0 && p->cpu_burst == p->timeslice) ||
        (proc_sched.algorithm == 0 && proc_sched.is_preemptive==1))
        yield();
}

