/**
 * @file uthreads.cpp
 * @author  Jason Elter <jason.elter@mail.huji.ac.il>
 * @version 1.0
 * @date 30 April 2020
 *
 * @brief User-Level Threads Library (uthreads)
 */

#include <deque>
#include <iostream>
#include <setjmp.h>
#include <signal.h>
#include <sys/time.h>
#include <vector>
#include <queue>
#include "uthreads.h"


// -------------------------- Constants -------------------------
#define SUCCESS 0
#define FAILURE -1
#define SKIP -1
#define MAIN_ID 0
#define AFTER_JUMP 1
#define MICRO_TO_SECONDS 1000000
#define LIB_ERROR "thread library error: "
#define INPUT_ERROR "Invalid input to "
#define THREAD_COUNT_ERROR "Attempting to create too many threads at "
#define OS_ERROR "system error: "
#define SIG_ERROR "Signal error"

// -------------------------- Utilities -------------------------
#ifdef __x86_64__
/* code for 64 bit Intel arch */

typedef unsigned long address_t;
#define JB_SP 6
#define JB_PC 7

/* A translation is required when using an address of a variable.
   Use this as a black box in your code. */
address_t translate_address(address_t addr)
{
    address_t ret;
    asm volatile("xor    %%fs:0x30,%0\n"
                 "rol    $0x11,%0\n"
    : "=g" (ret)
    : "0" (addr));
    return ret;
}

#else
/* code for 32 bit Intel arch */

typedef unsigned int address_t;
#define JB_SP 4
#define JB_PC 5

/* A translation is required when using an address of a variable.
   Use this as a black box in your code. */
address_t translate_address(address_t addr)
{
    address_t ret;
    asm volatile("xor    %%gs:0x18,%0\n"
        "rol    $0x9,%0\n"
                 : "=g" (ret)
                 : "0" (addr));
    return ret;
}

#endif

/* Crash program in case of system error and print error string. */
static void crash(std::exception &e)
{
    std::cerr << OS_ERROR << e.what() << std::endl;
    exit(EXIT_FAILURE);
}

/* Handle library error and print error string. */
static int handle_error(const char *what, const char *where)
{
    std::cerr << LIB_ERROR << what << where << std::endl;
    return FAILURE;
}

/* Exception for Signal errors. */
class SignalException : public std::exception
{
    const char *what() const noexcept override
    {
        return SIG_ERROR;
    }
};

static void timer_handler(int) noexcept; // Forward deceleration for timer handler needed.
static void signal_crash(SignalException &e) noexcept; // Forward deceleration needed.

// --------------------------- Classes --------------------------
/* Class that represents a single thread in a thread library. */
class Thread
{
    bool _blocked;
    int _quantum_seconds, _quantum_micro, _quantum_count;
    char _stack[STACK_SIZE];
    sigjmp_buf _env;

public:
    /* Constructor for thread without a function (used for main thread). */
    explicit Thread(int quantum) noexcept : _blocked(false), _quantum_seconds(quantum / MICRO_TO_SECONDS),
                                            _quantum_micro(quantum % MICRO_TO_SECONDS), _quantum_count(0), _stack(),
                                            _env()
    {}

    /* Constructor for thread with a function (not used for main thread). */
    Thread(int quantum, void (*f)()) noexcept;

    /* Updates the quantum count and the timer every time the thread starts running. */
    void activate(struct itimerval &timer);

    /* Set the quantum for this thread. */
    void set_quantum(int quantum) noexcept
    {
        _quantum_seconds = quantum / MICRO_TO_SECONDS;
        _quantum_micro = quantum % MICRO_TO_SECONDS;
    }

    /* Returns true if this thread is currently blocked. Otherwise, returns false. */
    bool is_blocked() const noexcept
    {
        return _blocked;
    }

    /* Block this thread. */
    void block() noexcept
    {
        _blocked = true;
    }

    /* Unblocks this thread. */
    void resume() noexcept
    {
        _blocked = false;
    }

    /* Returns how many times this thread has run. */
    int get_quantum_count() const noexcept
    {
        return _quantum_count;
    }

    /* Returns this thread's environment. */
    sigjmp_buf &get_environment() noexcept
    {
        return _env;
    }
};

/* Constructor for thread with a function (not used for main thread). */
Thread::Thread(int quantum, void (*f)()) noexcept : _blocked(false), _quantum_seconds(quantum / MICRO_TO_SECONDS),
                                                    _quantum_micro(quantum % MICRO_TO_SECONDS), _quantum_count(0),
                                                    _stack(), _env()
{
    // Setup jump parameters for later.
    address_t sp, pc;

    sp = (address_t) _stack + STACK_SIZE - sizeof(address_t);
    pc = (address_t) f;
    sigsetjmp(_env, 1);
    (_env->__jmpbuf)[JB_SP] = translate_address(sp);
    (_env->__jmpbuf)[JB_PC] = translate_address(pc);
    sigemptyset(&_env->__saved_mask);
}

/* Updates the quantum count and the timer every time the thread starts running. */
void Thread::activate(struct itimerval &timer)
{
    _quantum_count++;
    timer.it_value.tv_sec = _quantum_seconds; // first time interval, seconds part
    timer.it_value.tv_usec = _quantum_micro; // first time interval, microseconds part
    timer.it_interval.tv_sec = timer.it_interval.tv_usec = 0;    // Don't repeat.
}


/* Class that represents a user thread library. */
class ThreadLib
{
    int _size, _count, _total_quantum_count, _current_thread;
    int *_priority_to_quantum;
    sigset_t _sigset;
    Thread *_to_delete;
    std::vector<Thread *> threads;
    std::deque<int> _ready_queue;
    std::priority_queue<int, std::vector<int>, std::greater<int>> _empty_id_min_heap;

    /* Blocks the virtual timer till unblock_timer is called. */
    void block_timer() const
    {
        // Block thread switching.
        if (sigprocmask(SIG_BLOCK, &_sigset, nullptr) < 0)
        {
            throw SignalException();
        }
    }

    /* Unblocks the virtual timer that block_timer blocked. */
    void unblock_timer() const
    {
        // Unblock thread switching.
        if (sigprocmask(SIG_UNBLOCK, &_sigset, nullptr) < 0)
        {
            throw SignalException();
        }
    }

    /* Start a virtual timer for a thread when it starts running. */
    static void start_timer(Thread *to_start);

public:
    /* Ctor for thread library. */
    ThreadLib(int *quantum_usecs, int size);

    /* Dtor for thread library. */
    ~ThreadLib() noexcept;

    /* Switches the current thread with the next one. */
    void switch_threads() noexcept;

    /* Creates a new thread with the given function to run and with the given priority. */
    int spawn(void (*f)(), int priority);

    /* Changes the priority of the thread with the given id to the given priority (only affects next run). */
    int change_priority(int tid, int priority) noexcept;

    /* Terminates the thread with the given id. */
    int terminate(int tid);

    /* Blocks the thread with the given id. */
    int block_thread(int tid);

    /* Unblocks the thread with the given id. */
    int resume_thread(int tid);

    /* Returns the id of the currently running thread. */
    int get_current_id() const noexcept
    {
        return _current_thread;
    }

    /* Returns the total number of thread-runs that have happened so far. */
    int get_total_quantum_count() const noexcept
    {
        return _total_quantum_count;
    }

    /* Returns the total number of times that the thread with given id ran so far. */
    int get_thread_quantum_count(int tid) const noexcept
    {
        if (tid < 0 || tid >= (int) threads.size() || threads[tid] == nullptr)
        {
            return handle_error(INPUT_ERROR, "uthread_get_quantums");
        }

        return threads[tid]->get_quantum_count();
    }

};

/* Start a virtual timer for a thread when it starts running. */
void ThreadLib::start_timer(Thread *to_start)
{
    struct itimerval timer{};
    to_start->activate(timer);

    // Start a virtual timer. It counts down whenever this process is executing.
    if (setitimer(ITIMER_VIRTUAL, &timer, nullptr))
    {
        throw SignalException();
    }
}

/* Ctor for thread library. */
ThreadLib::ThreadLib(int *quantum_usecs, int size) : _size(size), _count(1), _total_quantum_count(1),
                                                     _current_thread(MAIN_ID), _priority_to_quantum(quantum_usecs),
                                                     _sigset(), _to_delete(nullptr), threads(), _ready_queue(),
                                                     _empty_id_min_heap()
{
    sigemptyset(&_sigset);
    sigaddset(&_sigset, SIGVTALRM);

    // Install timer_handler as the signal handler for SIGVTALRM.
    struct sigaction sa{};
    sa.sa_handler = &timer_handler;
    if (sigaction(SIGVTALRM, &sa, nullptr) < 0)
    {
        throw SignalException();
    }

    // Create main thread.
    auto *newThread = new Thread(_priority_to_quantum[MAIN_ID]);
    threads.push_back(newThread);
    start_timer(newThread);
}

/* Dtor for thread library. */
ThreadLib::~ThreadLib() noexcept
{
    for (Thread *current : threads)
    {
        if (current != nullptr)
        {
            delete (current);
        }
    }
}

/* Switches the current thread with the next one. */
void ThreadLib::switch_threads() noexcept
{
    // First check if current thread is valid and if it is blocked.
    Thread *current;
    if (_current_thread <= (int) threads.size() && (current = threads[_current_thread]) != nullptr)
    {
        if (!current->is_blocked()) // Only necessary to move thread to ready state if it isn't blocked.
        {
            _ready_queue.push_back(_current_thread);
        }
        int after_jump = sigsetjmp(current->get_environment(), 1); // Setup jump for later.
        if (after_jump == AFTER_JUMP)
        {
            // Deal with self-terminating threads.
            if (_to_delete != nullptr)
            {
                delete (_to_delete);
                _to_delete = nullptr;
            }
            return;
        }
    }

    // Find next valid unblocked thread and move to current status.
    do
    {
        _current_thread = _ready_queue.front();
        _ready_queue.pop_front();
    } while (_current_thread == SKIP || _current_thread > (int) threads.size() ||
             ((current = threads[_current_thread]) == nullptr) || current->is_blocked());

    _total_quantum_count++;
    try
    {
        start_timer(current);
    }
    catch (SignalException &e)
    {
        signal_crash(e); // Prevents bugs from deleting object within itself.
    }
    siglongjmp(current->get_environment(), AFTER_JUMP);
}

/* Creates a new thread with the given function to run and with the given priority. */
int ThreadLib::spawn(void (*f)(), int priority)
{
    if (_count == MAX_THREAD_NUM)
    {
        return handle_error(THREAD_COUNT_ERROR, "uthread_spawn");
    }

    block_timer();
    int newId;
    if (_empty_id_min_heap.empty()) // There are no available ids so make new one.
    {
        newId = _count;
        threads.push_back(new Thread(_priority_to_quantum[priority], f));
    }
    else
    { // There's an available id that isn't taken that we can use.
        newId = _empty_id_min_heap.top();
        _empty_id_min_heap.pop();
        threads[newId] = new Thread(_priority_to_quantum[priority], f);
    }

    _count++;
    _ready_queue.push_back(newId);
    unblock_timer();
    return newId;
}

/* Changes the priority of the thread with the given id to the given priority (only affects next run). */
int ThreadLib::change_priority(int tid, int priority) noexcept
{
    if (priority < 0 || priority >= _size || tid < 0 || tid >= (int) threads.size() || threads[tid] == nullptr)
    {
        return handle_error(INPUT_ERROR, "uthread_change_priority");
    }

    threads[tid]->set_quantum(_priority_to_quantum[priority]);
    return SUCCESS;
}

/* Terminates the thread with the given id. */
int ThreadLib::terminate(int tid)
{
    int threadsSize = threads.size();
    if (tid < 0 || tid >= threadsSize || threads[tid] == nullptr)
    {
        return handle_error(INPUT_ERROR, "uthread_terminate");
    }

    block_timer();
    Thread *to_delete = threads[tid];
    if (tid == threadsSize - 1) // Deleting last thread.
    {
        threads.pop_back();
    }
    else
    { // Deleting non-last thread so we mark the spot as available.
        threads[tid] = nullptr;
        _empty_id_min_heap.push(tid);
        for (int &cell : _ready_queue)
        {
            if (cell == tid)
            {
                cell = SKIP;
                break;
            }
        }
    }
    _count--;

    // If we terminate the current thread then we must switch and delete it from a different thread.
    if (tid == _current_thread)
    {
        _to_delete = to_delete;
        unblock_timer();
        switch_threads();
    }

    // This part never executes if a thread switch is called (it is deleted immediately after switch).
    delete (to_delete);
    unblock_timer();
    return SUCCESS;
}

/* Blocks the thread with the given id. */
int ThreadLib::block_thread(int tid)
{
    if (tid <= 0 || tid >= (int) threads.size() || threads[tid] == nullptr)
    {
        return handle_error(INPUT_ERROR, "uthread_block");
    }

    block_timer();
    Thread *to_block = threads[tid];
    if (!to_block->is_blocked())
    {
        threads[tid]->block();
        if (tid == _current_thread) // Stop executing thread immediately.
        {
            unblock_timer();
            switch_threads();
            return SUCCESS;
        }
        for (int &cell : _ready_queue) // Else find thread and mark it to skip.
        {
            if (cell == tid)
            {
                cell = SKIP;
                break;
            }
        }
    }
    unblock_timer();
    return SUCCESS;
}

/* Unblocks the thread with the given id. */
int ThreadLib::resume_thread(int tid)
{
    if (tid < 0 || tid >= (int) threads.size() || threads[tid] == nullptr)
    {
        return handle_error(INPUT_ERROR, "uthread_resume");
    }

    block_timer();
    Thread *to_block = threads[tid];
    if (to_block->is_blocked())
    {
        to_block->resume();
        _ready_queue.push_back(tid);
    }
    unblock_timer();
    return SUCCESS;
}


// --------------------------- Globals --------------------------
static ThreadLib *threadLibrary = nullptr; // Singleton.


// -------------------------- Functions -------------------------

/* Handler for timer signal that switches to the next thread. */
static void timer_handler(int) noexcept
{
    try
    {
        threadLibrary->switch_threads();
    }
    catch (SignalException &e)
    {
        delete (threadLibrary);
        crash(e);
    }
}

/* Special crash function to be called in case of signal error during thread switching. */
static void signal_crash(SignalException &e) noexcept
{
    delete (threadLibrary);
    crash(e);
}

/*
 * Description: This function initializes the thread library.
 * You may assume that this function is called before any other thread library
 * function, and that it is called exactly once. The input to the function is
 * an array of the length of a quantum in micro-seconds for each priority.
 * It is an error to call this function with an array containing non-positive integer.
 * size - is the size of the array.
 * Return value: On success, return 0. On failure, return -1.
*/
int uthread_init(int *quantum_usecs, int size)
{
    if (size <= 0)
    {
        return handle_error(INPUT_ERROR, "uthread_init");
    }
    for (int i = 0; i < size; i++)
    {
        if (quantum_usecs[i] <= 0)
        {
            return handle_error(INPUT_ERROR, "uthread_init");
        }
    }

    try
    {
        threadLibrary = new ThreadLib(quantum_usecs, size);
    }
    catch (std::exception &e)
    {
        if (threadLibrary != nullptr)
        {
            delete (threadLibrary);
        }
        crash(e);
    }
    return SUCCESS;
}


/*
 * Description: This function creates a new thread, whose entry point is the
 * function f with the signature void f(void). The thread is added to the end
 * of the READY threads list. The uthread_spawn function should fail if it
 * would cause the number of concurrent threads to exceed the limit
 * (MAX_THREAD_NUM). Each thread should be allocated with a stack of size
 * STACK_SIZE bytes.
 * priority - The priority of the new thread.
 * Return value: On success, return the ID of the created thread.
 * On failure, return -1.
*/
int uthread_spawn(void (*f)(), int priority)
{
    try
    {
        return threadLibrary->spawn(f, priority);
    }
    catch (std::exception &e)
    {
        delete (threadLibrary);
        crash(e);
    }
    return FAILURE;
}


/*
 * Description: This function changes the priority of the thread with ID tid.
 * If this is the current running thread, the effect should take place only the
 * next time the thread gets scheduled.
 * Return value: On success, return 0. On failure, return -1.
*/
int uthread_change_priority(int tid, int priority)
{
    return threadLibrary->change_priority(tid, priority);
}


/*
 * Description: This function terminates the thread with ID tid and deletes
 * it from all relevant control structures. All the resources allocated by
 * the library for this thread should be released. If no thread with ID tid
 * exists it is considered an error. Terminating the main thread
 * (tid == 0) will result in the termination of the entire process using
 * exit(0) [after releasing the assigned library memory].
 * Return value: The function returns 0 if the thread was successfully
 * terminated and -1 otherwise. If a thread terminates itself or the main
 * thread is terminated, the function does not return.
*/
int uthread_terminate(int tid)
{
    try
    {
        if (tid == MAIN_ID)
        {
            // Ignore alarms.
            struct sigaction sa{};
            sa.sa_handler = SIG_IGN;
            if (sigaction(SIGVTALRM, &sa, nullptr) < 0)
            {
                throw SignalException();
            }
            delete (threadLibrary);
            exit(SUCCESS);
        }
        return threadLibrary->terminate(tid);
    }
    catch (SignalException &e)
    {
        delete (threadLibrary);
        crash(e);
    }
    return FAILURE;
}


/*
 * Description: This function blocks the thread with ID tid. The thread may
 * be resumed later using uthread_resume. If no thread with ID tid exists it
 * is considered as an error. In addition, it is an error to try blocking the
 * main thread (tid == 0). If a thread blocks itself, a scheduling decision
 * should be made. Blocking a thread in BLOCKED state has no
 * effect and is not considered an error.
 * Return value: On success, return 0. On failure, return -1.
*/
int uthread_block(int tid)
{
    try
    {
        return threadLibrary->block_thread(tid);
    }
    catch (SignalException &e)
    {
        delete (threadLibrary);
        crash(e);
    }
    return FAILURE;
}


/*
 * Description: This function resumes a blocked thread with ID tid and moves
 * it to the READY state. Resuming a thread in a RUNNING or READY state
 * has no effect and is not considered as an error. If no thread with
 * ID tid exists it is considered an error.
 * Return value: On success, return 0. On failure, return -1.
*/
int uthread_resume(int tid)
{
    try
    {
        return threadLibrary->resume_thread(tid);
    }
    catch (SignalException &e)
    {
        delete (threadLibrary);
        crash(e);
    }
    return FAILURE;
}


/*
 * Description: This function returns the thread ID of the calling thread.
 * Return value: The ID of the calling thread.
*/
int uthread_get_tid()
{
    return threadLibrary->get_current_id();
}


/*
 * Description: This function returns the total number of quantums since
 * the library was initialized, including the current quantum.
 * Right after the call to uthread_init, the value should be 1.
 * Each time a new quantum starts, regardless of the reason, this number
 * should be increased by 1.
 * Return value: The total number of quantums.
*/
int uthread_get_total_quantums()
{
    return threadLibrary->get_total_quantum_count();
}


/*
 * Description: This function returns the number of quantums the thread with
 * ID tid was in RUNNING state. On the first time a thread runs, the function
 * should return 1. Every additional quantum that the thread starts should
 * increase this value by 1 (so if the thread with ID tid is in RUNNING state
 * when this function is called, include also the current quantum). If no
 * thread with ID tid exists it is considered an error.
 * Return value: On success, return the number of quantums of the thread with ID tid.
 * 			     On failure, return -1.
*/
int uthread_get_quantums(int tid)
{
    return threadLibrary->get_thread_quantum_count(tid);
}